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

#include <cstdio>
#include <cstdlib>

#include "mjbots/moteus/moteus_protocol.h"
#include "mjbots/moteus/pi3hat_moteus_interface.h"

#include "sensors/Adafruit_ADS1X15.h"
#include "sensors/Adafruit_INA260.h"

#include "dynamometer.h"

#include "libFilter/filters.h"
#include "iir/iir.h"
#include "nlohmann/json.hpp"

using namespace mjbots;

using MoteusInterface = moteus::Pi3HatMoteusInterface;
using json = nlohmann::json;


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
Dynamometer::Dynamometer(cxxopts::ParseResult dyn_opts, Adafruit_ADS1015 &ads,
Adafruit_INA260 &ina1, Adafruit_INA260 &ina2) : ads_(ads), ina1_(ina1), ina2_(ina2) {

  parse_settings(dyn_opts);

  if (dynset_.actuator_1_id == dynset_.actuator_2_id) {
    throw std::runtime_error("The servos must have unique IDs");
  }
  actuator_a_id = dynset_.actuator_1_id;
  actuator_b_id = dynset_.actuator_2_id;
  
  std::cout << "loading configs... \n";
  std::ifstream grp_if("configs/grp.json");
  json grp_j; grp_if >> grp_j;
  lpf_order_ = grp_j["butterworth_order"];
  lpf_fc_ = grp_j["cutoff_frequency_Hz"];
  grp_max_ampl = grp_j["torque_amplitude_Nm"];

  std::uniform_real_distribution<> dist(-grp_max_ampl, grp_max_ampl);
  realdist = dist;

  std::cout << "restoring calibrations... \n";
  system("python3 -m moteus.moteus_tool --target 1 --pi3hat-cfg '1=1;2=2' --restore-cal /home/pi/embir-modular-leg/moteus-setup/moteus-cal/ri50_cal_1_dyn.log");
  system("python3 -m moteus.moteus_tool --target 2 --pi3hat-cfg '1=1;2=2' --restore-cal /home/pi/embir-modular-leg/moteus-setup/moteus-cal/ri50_cal_2_dyn.log");

  fib_.resize(lpf_order_+1);
  fob_.resize(lpf_order_+1);
  lpf_dcof_ = dcof_bwlp(lpf_order_, 2*lpf_fc_*dynset_.period_s);
  lpf_ccof_ = ccof_bwlp(lpf_order_);
  lpf_sf_ = sf_bwlp(lpf_order_,  2*lpf_fc_*dynset_.period_s);

  std::cout << "setting up sensors... \n";
  ads_.begin(0x48);
  ads_.setGain(adsGain_t::GAIN_ONE);
  ads_.setDataRate(RATE_ADS1015_3300SPS);
  ina1_.begin(0x40);
  ina1_.prime_i2c();
  ina1_.setCurrentConversionTime(INA260_ConversionTime::INA260_TIME_204_us);
  ina1_.setVoltageConversionTime(INA260_ConversionTime::INA260_TIME_204_us);
  ina2_.begin(0x41);
  ina2_.prime_i2c();
  ina2_.setCurrentConversionTime(INA260_ConversionTime::INA260_TIME_204_us);
  ina2_.setVoltageConversionTime(INA260_ConversionTime::INA260_TIME_204_us);

}

/// This is called before any control begins, and must return the
/// set of servos that are used, along with which bus each is
/// attached to.
std::map<int, int> Dynamometer::servo_bus_map() const {
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
void Dynamometer::Initialize(std::vector<MoteusInterface::ServoCommand>* commands) {
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

moteus::QueryResult Dynamometer::Get(const std::vector<MoteusInterface::ServoReply>& replies, int id) {
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
void Dynamometer::Run(const std::vector<MoteusInterface::ServoReply>& status,
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
    // actuator_b_out.mode = moteus::Mode::kPosition;
    actuator_b_out.mode = (dynset_.testmode == TestMode::kGRP) ? moteus::Mode::kZeroVelocity : moteus::Mode::kPosition;
    generate_commands(t_prog_s_, actuator_a_out.position, actuator_b_out.position);
  }
}

void Dynamometer::set_t0(std::chrono::steady_clock::time_point t0) {
  t0_ = t0;
}

void Dynamometer::swap_actuators() {
  std::swap(actuator_a_idx, actuator_b_idx);
  std::swap(actuator_a_id, actuator_b_id);
}

void Dynamometer::generate_commands(double time, mjbots::moteus::PositionCommand &cmda, mjbots::moteus::PositionCommand &cmdb) {
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
      
      float rand_cmd = random_sample;

      // rotate buffers one element to the right to put new data in
      std::rotate(fib_.rbegin(), fib_.rbegin()+1, fib_.rend());
      std::rotate(fob_.rbegin(), fob_.rbegin()+1, fob_.rend());
      fib_[0] = rand_cmd;
      fob_[0] = 0; // this term should cancel below 
      rand_cmd = 0;
      for (size_t ii = 0; ii < lpf_order_+1; ++ii) {
        rand_cmd += fib_[ii]*lpf_ccof_[ii]*lpf_sf_ - fob_[ii]*lpf_dcof_[ii];
      }
      fob_[0] = rand_cmd;
      
      rand_cmd = (rand_cmd > grp_max_ampl) ? grp_max_ampl : rand_cmd;
      rand_cmd = (rand_cmd < -grp_max_ampl) ? -grp_max_ampl : rand_cmd;
      
      cmda.kp_scale = 0; cmda.kd_scale = 0;
      cmda.feedforward_torque = rand_cmd;
      
      cmdb.kp_scale = 0; cmdb.kd_scale = 0;
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
  // sample sensors and store data
  sample_sensors();
}

void Dynamometer::sample_random() {
  std::mt19937 gen(rd_());
  random_sample = realdist(gen);
}

void Dynamometer::sample_sensors() {
  ads_.prime_i2c();
  uint16_t adc = ads_.readADC_SingleEnded(0);
  float torque_volts = ads_.computeVolts(adc);

  // TODO: Add temp read and calc
  sd_.temp1_C = 0;
  sd_.temp2_C = 0;

  float tqsen_gain = (dynset_.tqsen==TorqueSensor::kTRD605_18) ? 18.0/50 : 5.0/5.0;
  sd_.torque_Nm = (torque_volts - 5.0/3.0) * 3.0;

  if (dynset_.testmode == TestMode::kGRP) {
    // run faster in GRP mode
    sd_.ina1_voltage_V = 0;
    sd_.ina1_current_A = 0;
    sd_.ina1_power_W = 0;

    sd_.ina2_voltage_V = 0;
    sd_.ina2_current_A = 0;
    sd_.ina2_power_W = 0;
  }
  else {
    ina1_.prime_i2c();
    sd_.ina1_voltage_V = ina1_.readBusVoltage()/1000;
    sd_.ina1_current_A = ina1_.readCurrent()/1000;
    sd_.ina1_power_W = sd_.ina1_current_A * sd_.ina1_voltage_V;

    ina2_.prime_i2c();
    sd_.ina2_voltage_V = ina2_.readBusVoltage()/1000;
    sd_.ina2_current_A = ina2_.readCurrent()/1000;
    sd_.ina2_power_W = sd_.ina2_current_A * sd_.ina2_voltage_V;
  }
}

std::string Dynamometer::stringify_sensor_data() {
  std::ostringstream result;
  sprintf(cstr_buffer, "%f", sd_.torque_Nm);
  result << cstr_buffer;
  if(dynset_.testmode == TestMode::kGRP) return result.str();

  sprintf(cstr_buffer, ",%f,%f,", sd_.temp1_C, sd_.temp2_C);
  result << cstr_buffer;
  sprintf(cstr_buffer, "%f,%f,%f,", sd_.ina1_voltage_V, sd_.ina1_current_A, sd_.ina1_power_W);
  result << cstr_buffer;
  sprintf(cstr_buffer, "%f,%f,%f", sd_.ina2_voltage_V, sd_.ina2_current_A, sd_.ina2_power_W);
  result << cstr_buffer;
  return result.str();
}

std::string Dynamometer::stringify_sensor_data_headers() {
  std::ostringstream result;
  result << ((dynset_.tqsen == TorqueSensor::kTRD605_18) ? "trd605-18 torque [Nm]" : "trs605-5 torque [Nm]");
  if(dynset_.testmode == TestMode::kGRP) return result.str();

  result << ",motor temp [C],housing temp [C],ina1 voltage [V],ina1 current [A],ina1 power [W],ina2 voltage [V],ina2 current [A],ina2 power [W]";
  return result.str();
}

double Dynamometer::get_program_time() {return t_prog_s_;}

void Dynamometer::parse_settings(cxxopts::ParseResult dyn_opts) {
  dynset_.dyn_opts = dyn_opts;
  dynset_.period_s = 1.0/dyn_opts["frequency"].as<float>();
  dynset_.gear1 = dyn_opts["gear1"].as<float>();
  dynset_.gear2 = dyn_opts["gear2"].as<float>();
  dynset_.actuator_1_id = dyn_opts["actuator-1-id"].as<uint8_t>();
  dynset_.actuator_2_id = dyn_opts["actuator-2-id"].as<uint8_t>();
  dynset_.actuator_1_bus = dyn_opts["actuator-1-bus"].as<uint8_t>();
  dynset_.actuator_2_bus = dyn_opts["actuator-2-bus"].as<uint8_t>();

  auto test_str = dyn_opts["test-mode"].as<std::string>();

  // std::cout << test_str << (test_str == std::string("TV-sweep")) << std::endl;
  if (test_str == std::string("KT")) {
    dynset_.testmode = TestMode::kTorqueConstant; std::cout << "test mode " << test_str << " selected" << std::endl;
  } else if (test_str == std::string("GRP")) {
    dynset_.testmode = TestMode::kGRP; std::cout << "test mode " << test_str << " selected" << std::endl;
  } else if (test_str == std::string("direct-damping")) {
    dynset_.testmode = TestMode::kDirectDamping; std::cout << "test mode " << test_str << " selected" << std::endl;
  } else if (test_str == std::string("TV-sweep")) {
    dynset_.testmode = TestMode::kTorqueVelSweep; std::cout << "test mode " << test_str << " selected" << std::endl;
  } else if (test_str == std::string("manual")) {
    dynset_.testmode = TestMode::kManual; std::cout << "test mode " << test_str << " selected" << std::endl;
  } else {
    dynset_.testmode = TestMode::kNone;  std::cout << "no test mode selected" << std::endl;
  }
  
  auto tqsen_str = dyn_opts["torquesensor"].as<std::string>();
  if (tqsen_str == std::string("trd605-18")) dynset_.tqsen = TorqueSensor::kTRD605_18;
  else if (tqsen_str == std::string("trs605-5")) dynset_.tqsen = TorqueSensor::kTRS605_5;
  else dynset_.tqsen = TorqueSensor::kTRS605_5;

  dynset_.main_cpu = dyn_opts["main-cpu"].as<uint8_t>();
  dynset_.can_cpu = dyn_opts["can-cpu"].as<uint8_t>();
}

void ConfigureRealtime(const uint8_t realtime) {
  {
    cpu_set_t cpuset = {};
    CPU_ZERO(&cpuset);
    CPU_SET(realtime, &cpuset);

    const int r = ::sched_setaffinity(0, sizeof(cpu_set_t), &cpuset);
    if (r < 0) {
      throw std::runtime_error("Error setting CPU affinity");
    }

    std::cout << "Affinity set to " << (int)realtime << "\n";
  }

  {
    struct sched_param params = {};
    params.sched_priority = 10;
    const int r = ::sched_setscheduler(0, SCHED_RR, &params);
    if (r < 0) {
      throw std::runtime_error("Error setting realtime scheduler");
    }
  }

  {
    const int r = ::mlockall(MCL_CURRENT | MCL_FUTURE);
    if (r < 0) {
      throw std::runtime_error("Error locking memory");
    }
  }
}

cxxopts::Options dyn_opts() {
  cxxopts::Options options("dynamometer", "Run dual actuator dynamometer for characterization");

  options.add_options()
    ("c,comment", "enter comment string to be included in output csv.", cxxopts::value<std::string>())
    ("gear1", "gear ratio of actuator 1, as a reduction", cxxopts::value<float>()->default_value("1.0"))
    ("gear2", "gear ratio of actuator 2, as a reduction", cxxopts::value<float>()->default_value("1.0"))
    ("actuator-1-id", "actuator 1 CAN ID", cxxopts::value<uint8_t>()->default_value("1"))
    ("actuator-2-id", "actuator 2 CAN ID", cxxopts::value<uint8_t>()->default_value("2"))
    ("actuator-1-bus", "actuator 1 CAN bus", cxxopts::value<uint8_t>()->default_value("1"))
    ("actuator-2-bus", "actuator 2 CAN bus", cxxopts::value<uint8_t>()->default_value("2"))
    ("main-cpu", "main CPU", cxxopts::value<uint8_t>()->default_value("1"))
    ("can-cpu", "CAN CPU", cxxopts::value<uint8_t>()->default_value("2"))
    ("torquesensor", "declare which torque sensor is being used between [trd605-18| trs605-5]", cxxopts::value<std::string>()->default_value("trs605-5"))
    ("duration", "test duration in seconds", cxxopts::value<float>())
    ("frequency", "test sampling and command frequency in Hz", cxxopts::value<float>()->default_value("250"))
    ("test-mode", "choose between [KT|GRP|direct-damping|TV-sweep|manual]", cxxopts::value<std::string>()->default_value("none"))
    ("h,help", "Print usage")
  ;

  // test deps
  return options;
}