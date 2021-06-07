#include <sys/mman.h>

#include <vector>
#include <chrono>
#include <sstream>
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <future>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>

#include <cstdio>

#include "mjbots/moteus/moteus_protocol.h"
#include "mjbots/moteus/pi3hat_moteus_interface.h"

#include "sensors/Adafruit_ADS1X15.h"
#include "sensors/Adafruit_INA260.h"

#include "dynamometer.h"

#include "libFilter/filters.h"
#include "iir/iir.h"
#include "nlohmann/json.hpp"

#define PI 3.1415926

using namespace mjbots;

using MoteusInterface = moteus::Pi3HatMoteusInterface;
using json = nlohmann::json;

char cstr_buffer[128];

std::string stringify_moteus_reply(MoteusInterface::ServoReply& reply) {
  uint8_t id = reply.id;
  auto& data = reply.result;
  std::ostringstream result;
  sprintf(cstr_buffer, "%d, %f, %f, ",
    data.mode, data.position, data.velocity);
  result << cstr_buffer;
  sprintf(cstr_buffer, "%f, %f, %d",
    data.torque, data.temperature, data.fault);
  result << cstr_buffer;
  return result.str();
}

std::string stringify_actuator(MoteusInterface::ServoCommand command,
  MoteusInterface::ServoReply& reply, float gear_reduction) {
  uint8_t id = reply.id;
  std::ostringstream result;
  auto& cmd_data = command.position;
  sprintf(cstr_buffer, "%f, %f, %f, ",
    cmd_data.position*2*PI/gear_reduction,
    cmd_data.velocity*2*PI/gear_reduction,
    cmd_data.feedforward_torque*gear_reduction);
  result << cstr_buffer;

  auto& reply_data = reply.result;
  sprintf(cstr_buffer, "%f, %f, %f",
    reply_data.position*2*PI/gear_reduction,
    reply_data.velocity*2*PI/gear_reduction,
    reply_data.torque*gear_reduction);
  result << cstr_buffer;

  if(command.id != reply.id) {std::cout << "\n\nAAAAAAHHHHHH ID MISMATCH\n\n" << std::endl;}
  return result.str();
}

void Run(Dynamometer* dynamometer) {
  
  // if (dynset.help) {
  //   DisplayUsage();
  //   return;
  // }

  // for convenience
  Dynamometer::DynamometerSettings& dynset = dynamometer->dynset_;

  // * SETUP *

  // ** CONFIGURE CPU AND MOTEUS INFRASTRUCTURE **
  moteus::ConfigureRealtime(dynset.main_cpu);
  MoteusInterface::Options moteus_options;
  moteus_options.cpu = dynset.can_cpu;
  moteus_options.servo_bus_map = dynamometer->servo_bus_map();
  MoteusInterface moteus_interface{moteus_options};

  // ** CONTAINER FOR COMMANDS **
  std::vector<MoteusInterface::ServoCommand> commands;
  std::vector<MoteusInterface::ServoCommand> saved_commands;
  for (const auto& pair : moteus_options.servo_bus_map) {
    commands.push_back({});
    commands.back().id = pair.first;
    saved_commands.push_back({});
    saved_commands.back().id = pair.first;
  }

  // ** CONTAINER FOR REPLIES **
  std::vector<MoteusInterface::ServoReply> replies{commands.size()};
  std::vector<MoteusInterface::ServoReply> saved_replies;

  // ** INITIALIZE COMMANDS **
  dynamometer->Initialize(&commands);

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

  dynamometer->set_t0(std::chrono::steady_clock::now());

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
        // std::cout << std::setprecision(6) << std::fixed
        //           << "Cycles " << cycle_count
        //           << "  margin: " << (total_margin / margin_cycles)
        //           << std::setprecision(1)
        //           << "  volts: " << volts.first << "/" << volts.second
        //           << "  modes: " << modes
        //           << "   \r";
        // std::cout.flush();
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


    dynamometer->Run(saved_replies, &commands);

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

    if (cycle_count > 5 && saved_replies.size() >= 2) {
      uint8_t cmd1_idx = (saved_commands.at(0).id == 1) ? 0 : 1;
      uint8_t cmd2_idx = (cmd1_idx == 0) ? 1 : 0;
      uint8_t rpl1_idx = (saved_replies.at(0).id == 1) ? 0 : 1;
      uint8_t rpl2_idx = (rpl1_idx == 0) ? 1 : 0;

      auto c1_str = stringify_moteus_reply(saved_replies.at(rpl1_idx));
      auto c2_str = stringify_moteus_reply(saved_replies.at(rpl2_idx));
      auto a1_str = stringify_actuator(saved_commands.at(cmd1_idx), saved_replies.at(rpl1_idx), dynset.gear1);
      auto a2_str = stringify_actuator(saved_commands.at(cmd2_idx), saved_replies.at(rpl2_idx), dynset.gear2);


      auto sensor_str = dynamometer->stringify_sensor_data();
      std::cout << a1_str << ",    " << c1_str << ",    "
        << a2_str << ",    " << c2_str << std::endl;
    }
    else {
      std::cout << "missing moteus reply" << std::endl;
    }

    if (cycle_count > 1) std::copy(commands.begin(), commands.end(), saved_commands.begin());
  }
}


int main(int argc, char** argv) {
  auto options = dyn_opts();
  auto opts = options.parse(argc, argv);
  std::cout << opts["comment"].as<std::string>() << std::endl;

  Adafruit_ADS1015 ads;
  Adafruit_INA260 ina1;
  Adafruit_INA260 ina2;
  if (!bcm2835_init()) {
    std::cout << "bcm2835_init failed. Are you running as root??\n" << std::endl;
    return 1;
  }
  bcm2835_i2c_begin();

  LockMemory();

  // Lock memory for the whole process.
  Dynamometer dynamometer(opts, ads, ina1, ina2);
  // for convenience
  Dynamometer::DynamometerSettings& dynset = dynamometer.dynset_;

  ConfigureRealtime(dynset.main_cpu);

  // return 0;
  Run(&dynamometer);

  return 0;
}