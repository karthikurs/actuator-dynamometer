// Copyright 2020 Josh Pieper, jjp@pobox.com.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// @file
///
/// This is a simple application that demonstrates how to efficiently
/// monitor and control multiple moteus servos at a high rate using
/// the pi3hat.
///
/// It is contained in a single file for the purposes of
/// demonstration.  A real application should likely be implemented in
/// multiple translation units or structured for longer term
/// maintenance.

#include <sys/mman.h>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <future>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <random>
#include <algorithm>

#include "mjbots/moteus/moteus_protocol.h"
#include "mjbots/moteus/pi3hat_moteus_interface.h"

#include "Adafruit_ADS1X15.h"
#include "Adafruit_INA260.h"

#include "dynamometer.h"

#include "libFilter/filters.h"
#include "iir/iir.h"
#include "nlohmann/json.hpp"

using namespace mjbots;

using MoteusInterface = moteus::Pi3HatMoteusInterface;
using json = nlohmann::json;

namespace {

void LockMemory() {
  // We lock all memory so that we don't end up having to page in
  // something later which can take time.
  {
    const int r = ::mlockall(MCL_CURRENT | MCL_FUTURE);
    if (r < 0) {
      throw std::runtime_error("Error locking memory");
    }
  }
}

std::pair<double, double> MinMaxVoltage(
    const std::vector<MoteusInterface::ServoReply>& r) {
  double rmin = std::numeric_limits<double>::infinity();
  double rmax = -std::numeric_limits<double>::infinity();

  for (const auto& i : r) {
    if (i.result.voltage > rmax) { rmax = i.result.voltage; }
    if (i.result.voltage < rmin) { rmin = i.result.voltage; }
  }

  return std::make_pair(rmin, rmax);
}

/// This holds the user-defined control logic.
class Dynamometer {
 public:
  Dynamometer(const DynamometerSettings& dynset) :
    dynset_(dynset) {
    // lpf(40, dynset.period_s, IIR::ORDER::OD3, IIR::TYPE::LOWPASS) {
    
    // lpf.dumpParams();
    if (dynset_.actuator_1_id == dynset_.actuator_2_id) {
      throw std::runtime_error("The servos must have unique IDs");
    }
    actuator_a_id = dynset_.actuator_1_id;
    actuator_b_id = dynset_.actuator_2_id;
    
    std::ifstream grp_if("configs/grp.json");
    json grp_j; grp_if >> grp_j;
    lpf_order_ = grp_j["butterworth_order"];
    lpf_fc_ = grp_j["cutoff_frequency_Hz"];
    grp_max_ampl = grp_j["torque_amplitude_Nm"];

    fib_.resize(lpf_order_+1);
    fob_.resize(lpf_order_+1);
    lpf_dcof_ = dcof_bwlp(lpf_order_, 2*lpf_fc_*dynset_.period_s);
    lpf_ccof_ = ccof_bwlp(lpf_order_);
    lpf_sf_ = sf_bwlp(lpf_order_,  2*lpf_fc_*dynset_.period_s);
    for (size_t ii = 0; ii<lpf_order_+1; ++ii) {
      fib_[ii] = 0;
      fob_[ii] = 0;
      std::cout << lpf_dcof_[ii] << '\t' << lpf_ccof_[ii] << '\n';
    }
    std::cout << lpf_sf_ << std::endl;
  }

  /// This is called before any control begins, and must return the
  /// set of servos that are used, along with which bus each is
  /// attached to.
  std::map<int, int> servo_bus_map() const {
    return {
      { dynset_.actuator_1_id, dynset_.actuator_1_bus },
      { dynset_.actuator_2_id, dynset_.actuator_2_bus },
    };
  }

  /// This is also called before any control begins.  @p commands will
  /// be pre-populated with an entry for each servo returned by
  /// 'servo_bus_map'.  It can be used to perform one-time
  /// initialization like setting the resolution of commands and
  /// queries.
  void Initialize(std::vector<MoteusInterface::ServoCommand>* commands) {
    moteus::PositionResolution res;
    res.position = moteus::Resolution::kInt16;
    res.velocity = moteus::Resolution::kInt16;
    res.feedforward_torque = moteus::Resolution::kInt16;
    res.kp_scale = moteus::Resolution::kInt16;
    res.kd_scale = moteus::Resolution::kInt16;
    res.maximum_torque = moteus::Resolution::kIgnore;
    res.stop_position = moteus::Resolution::kIgnore;
    res.watchdog_timeout = moteus::Resolution::kIgnore;
    for (auto& cmd : *commands) {
      cmd.resolution = res;
    }
  }

  moteus::QueryResult Get(const std::vector<MoteusInterface::ServoReply>& replies, int id) {
    for (const auto& item : replies) {
      if (item.id == id) { return item.result; }
    }
    return {};
  }

  /// This is run at each control cycle.  @p status is the most recent
  /// status of all servos (note that it is possible for a given
  /// servo's result to be omitted on some frames).
  ///
  /// @p output should hold the desired output.  It will be
  /// pre-populated with the result of the last command cycle, (or
  /// Initialize to begin with).
  void Run(const std::vector<MoteusInterface::ServoReply>& status,
           std::vector<MoteusInterface::ServoCommand>* output) {
    cycle_count_++;
    auto time_span = std::chrono::steady_clock::now() - t0_;
    t_prog_s_ = double(time_span.count()) * std::chrono::steady_clock::period::num / 
          std::chrono::steady_clock::period::den;

    // This is where your control loop would go.

    if (cycle_count_ < 5) {
      for (auto& cmd : *output) {
        // We start everything with a stopped command to clear faults.
        cmd.mode = moteus::Mode::kStopped;
      }
    }
    else {
      // Then we make the actuator2 servo mirror the actuator1 servo.
      const auto actuator_a = Get(status, actuator_a_id);
      double actuator_a_pos = actuator_a.position;
      const auto actuator_b = Get(status, actuator_a_id);
      double actuator_b_pos = actuator_b.position;
      // We have everything we need to start commanding.
      auto& actuator_b_out = output->at(actuator_b_idx);  // We constructed this, so we know the order.
      auto& actuator_a_out = output->at(actuator_a_idx);

      actuator_a_out.mode = moteus::Mode::kPosition;
      // actuator_a_out.position.position = std::sin(t_prog_s_);
      // actuator_a_out.position.velocity = nan("");
      // actuator_a_out.position.feedforward_torque = 0;

      actuator_b_out.mode = moteus::Mode::kPosition;
      // actuator_b_out.position.position = std::sin(t_prog_s_);
      // actuator_b_out.position.velocity = nan("");
      // actuator_b_out.position.feedforward_torque = 0;
      generate_commands(t_prog_s_, actuator_a_out.position, actuator_b_out.position);
    }
  }

  void set_t0(std::chrono::steady_clock::time_point t0) {
    t0_ = t0;
  }

  void swap_actuators() {
    std::swap(actuator_a_idx, actuator_b_idx);
    std::swap(actuator_a_id, actuator_b_id);
  }

  void generate_commands(double time, mjbots::moteus::PositionCommand &cmda, mjbots::moteus::PositionCommand &cmdb) {
    cmda.position = 0;
    cmda.velocity = 0;
    cmda.feedforward_torque = 0;
    cmdb.position = 0;
    cmdb.velocity = 0;
    cmdb.feedforward_torque = 0;
    // std::cout << "in generate_commands(), dynset_.testmode = " << dynset_.testmode << std::endl;

    switch (dynset_.testmode) {
      case TestMode::kTorqueConstant:
        break;
      case TestMode::kDirectDamping:
        break;
      case TestMode::kGRP: {
        std::mt19937 gen(rd_());
        std::uniform_real_distribution<> dist(-grp_max_ampl, grp_max_ampl);
        float rand_cmd = dist(gen);
        std::cout << '\t' << rand_cmd;
        // rotate input buffer one element to the right to put new input in
        std::rotate(fib_.rbegin(),
          fib_.rbegin()+1,
          fib_.rend());
        // rand_cmd = lpf.filterIn(rand_cmd);
        fib_[0] = rand_cmd;
        rand_cmd = 0;
        // don't rotate output buffer, because you should be using older outputs
        for (size_t ii = 0; ii < lpf_order_; ++ii) {
          rand_cmd += fib_[ii]*lpf_ccof_[ii]*lpf_sf_ - fob_[ii]*lpf_dcof_[ii];
          std::cout << '(' << fib_[ii] << ", " << fob_[ii] << ")\n"; 
        }
        std::cout << std::endl;
        // rotate output buffer now to put new output in
        std::rotate(fob_.rbegin(),
          fob_.rbegin()+1,
          fob_.rend());
        fob_[0] = rand_cmd;
        // rand_cmd = 0;
        rand_cmd = (rand_cmd > 1.5) ? 1.5 : rand_cmd;
        rand_cmd = (rand_cmd < -1.5) ? -1.5 : rand_cmd;
        std::cout << '\t' << rand_cmd << std::endl;
        cmda.kp_scale = 0;
        cmda.kd_scale = 0;
        cmda.feedforward_torque = rand_cmd;
        cmdb.kp_scale = 0;
        cmdb.kd_scale = 0;
        break;
        }
      case TestMode::kTorqueVelSweep: {
        cmda.position = std::sin(t_prog_s_);
        cmda.velocity = nan("");
        cmdb.position = std::sin(t_prog_s_);
        cmdb.velocity = nan("");
        break;
        }
      case TestMode::kManual:
        break;
      case TestMode::kNone:
        break;
    }
  }

 private:
  const DynamometerSettings dynset_;
  uint64_t cycle_count_ = 0;
  double actuator_a_initial_ = std::numeric_limits<double>::quiet_NaN();
  double actuator_b_initial_ = std::numeric_limits<double>::quiet_NaN();

  size_t actuator_a_idx = 0;
  size_t actuator_b_idx = 1;

  uint8_t actuator_a_id = 1;
  uint8_t actuator_b_id = 2;

  float grp_max_ampl = 1.0; //amps

  std::chrono::steady_clock::time_point t0_;
  double t_prog_s_;

  // Filter lpf;

  double *lpf_dcof_;
  int *lpf_ccof_;
  double lpf_sf_;
  uint8_t lpf_order_ = 3;

  float lpf_fc_ = 40;

  std::random_device rd_;
  std::vector<float> fib_;
  std::vector<float> fob_;
};

template <typename Controller>
void Run(const DynamometerSettings& dynset, Controller* controller) {
  // if (dynset.help) {
  //   DisplayUsage();
  //   return;
  // }

  // * SETUP *

  // ** CONFIGURE CPU AND MOTEUS INFRASTRUCTURE **
  moteus::ConfigureRealtime(dynset.main_cpu);
  MoteusInterface::Options moteus_options;
  moteus_options.cpu = dynset.can_cpu;
  moteus_options.servo_bus_map = controller->servo_bus_map();
  MoteusInterface moteus_interface{moteus_options};

  // ** CONTAINER FOR COMMANDS **
  std::vector<MoteusInterface::ServoCommand> commands;
  for (const auto& pair : moteus_options.servo_bus_map) {
    commands.push_back({});
    commands.back().id = pair.first;
  }

  // ** CONTAINER FOR REPLIES **
  std::vector<MoteusInterface::ServoReply> replies{commands.size()};
  std::vector<MoteusInterface::ServoReply> saved_replies;

  // ** INITIALIZE COMMANDS **
  controller->Initialize(&commands);

  // ** PACKAGE COMMANDS AND REPLIES IN moteus_data **
  MoteusInterface::Data moteus_data;
  moteus_data.commands = { commands.data(), commands.size() };
  moteus_data.replies = { replies.data(), replies.size() };

  std::future<MoteusInterface::Output> can_result;

  // ** TEST PERIOD **
  const auto period =
      std::chrono::microseconds(static_cast<int64_t>(dynset.period_s * 1e6));
  auto next_cycle = std::chrono::steady_clock::now() + period;

  // ** TERMINAL STATUS UPDATE PERIOD **
  const auto status_period = std::chrono::milliseconds(100);
  auto next_status = next_cycle + status_period;
  uint64_t cycle_count = 0;
  double total_margin = 0.0;
  uint64_t margin_cycles = 0;

  controller->set_t0(std::chrono::steady_clock::now());

  // * MAIN LOOP *
  // We will run at a fixed cycle time.
  while (true) {
    cycle_count++;
    margin_cycles++;
    // Terminal status update
    {
      const auto now = std::chrono::steady_clock::now();
      if (now > next_status) {
        // NOTE: iomanip is not a recommended pattern.  We use it here
        // simply to not require any external dependencies, like 'fmt'.
        const auto volts = MinMaxVoltage(saved_replies);
        const std::string modes = [&]() {
          std::ostringstream result;
          result.precision(4);
          result << std::fixed;
          for (const auto& item : saved_replies) {
            result << item.id << "/"
                   << static_cast<int>(item.result.mode) << "/"
                   << item.result.position << " ";
          }
          return result.str();
        }();
        std::cout << std::setprecision(6) << std::fixed
                  << "Cycles " << cycle_count
                  << "  margin: " << (total_margin / margin_cycles)
                  << std::setprecision(1)
                  << "  volts: " << volts.first << "/" << volts.second
                  << "  modes: " << modes
                  << "   \r";
        std::cout.flush();
        next_status += status_period;
        total_margin = 0;
        margin_cycles = 0;
      }

      int skip_count = 0;
      while (now > next_cycle) {
        skip_count++;
        next_cycle += period;
      }
      if (skip_count) {
        std::cout << "\nSkipped " << skip_count << " cycles\n";
      }
    }
    
    // Sleep current thread until next control interval, per the period setting.
    {
      const auto pre_sleep = std::chrono::steady_clock::now();
      std::this_thread::sleep_until(next_cycle);
      const auto post_sleep = std::chrono::steady_clock::now();
      std::chrono::duration<double> elapsed = post_sleep - pre_sleep;
      total_margin += elapsed.count();
    }
    next_cycle += period;


    controller->Run(saved_replies, &commands);


    if (can_result.valid()) {
      // Now we get the result of our last query and send off our new
      // one.
      const auto current_values = can_result.get();

      // We copy out the results we just got out.
      const auto rx_count = current_values.query_result_size;
      saved_replies.resize(rx_count);
      std::copy(replies.begin(), replies.begin() + rx_count,
                saved_replies.begin());
    }

    // Then we can immediately ask them to be used again.
    auto promise = std::make_shared<std::promise<MoteusInterface::Output>>();
    // Cycle out commands to drivers
    moteus_interface.Cycle(
        moteus_data,
        [promise](const MoteusInterface::Output& output) {
          // This is called from an arbitrary thread, so we just set
          // the promise value here.
          promise->set_value(output);
        });
    can_result = promise->get_future();
  }
}
}

int main(int argc, char** argv) {
  auto options = dyn_opts();
  auto opts = options.parse(argc, argv);
  std::cout << opts["comment"].as<std::string>() << std::endl;

  DynamometerSettings dynset(opts);

  // Lock memory for the whole process.
  LockMemory();

  Dynamometer dynamometer{dynset};
  return 0;
  Run(dynset, &dynamometer);

  return 0;
}
