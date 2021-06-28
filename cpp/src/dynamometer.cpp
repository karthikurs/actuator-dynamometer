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
#include <cmath>

#include "mjbots/moteus/moteus_protocol.h"
#include "mjbots/moteus/pi3hat_moteus_interface.h"

#include "sensors/Adafruit_ADS1X15.h"
#include "sensors/Adafruit_INA260.h"

#include "dynamometer.h"

#include "libFilter/filters.h"
#include "iir/iir.h"

#include "rapidcsv/rapidcsv.h"

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
  grp_if >> grp_j;
  lpf_order_ = grp_j["butterworth_order"];
  lpf_fc_ = grp_j["cutoff_frequency_Hz"];
  grp_max_ampl = grp_j["torque_amplitude_Nm"];
  dynset_.grp_sampling_period_us = static_cast<int64_t>(
    (1e6)/float(grp_j["random_sampling_frequency_Hz"]));

  std::ifstream safety_if("configs/safety_limits.json");
  safety_if >> safety_j;
  max_motor_temp_C_ = safety_j["max_motor_temp_C"];
  max_housing_temp_C_ = safety_j["max_housing_temp_C"];
  trs605_5_max_torque_Nm_ = safety_j["trs605-5_max_torque_Nm"];
  trd605_18_max_torque_Nm_ = safety_j["trd605-18_max_torque_Nm"];
  actuator_torque_disparity_ratio_ = safety_j["actuator_torque_disparity_ratio"];
  
  dynset_.status_period_us = static_cast<int64_t>(
    (1e6)/10);

  std::uniform_real_distribution<> dist(-grp_max_ampl, grp_max_ampl);
  realdist = dist;

  if (!(dyn_opts["skip-cal"].as<bool>())) {
    std::cout << "restoring calibrations... \n";
    system("python3 -m moteus.moteus_tool --target 1 --pi3hat-cfg '3=1;4=2' --restore-cal /home/pi/embir-modular-leg/moteus-setup/moteus-cal/ri50_cal_1_dyn.log");
    system("python3 -m moteus.moteus_tool --target 2 --pi3hat-cfg '3=1;4=2' --restore-cal /home/pi/embir-modular-leg/moteus-setup/moteus-cal/ri50_cal_2_dyn.log");
  }

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
  res.position = moteus::Resolution::kFloat;
  res.velocity = moteus::Resolution::kFloat;
  res.feedforward_torque = moteus::Resolution::kFloat;
  res.kp_scale = moteus::Resolution::kInt16;
  res.kd_scale = moteus::Resolution::kInt16;
  res.maximum_torque = moteus::Resolution::kIgnore;
  res.stop_position = moteus::Resolution::kIgnore;
  res.watchdog_timeout = moteus::Resolution::kIgnore;
  for (auto& cmd : *commands) {
    cmd.resolution = res;
    cmd.query.velocity = moteus::Resolution::kFloat;
    cmd.query.position = moteus::Resolution::kFloat;
    cmd.query.torque = moteus::Resolution::kFloat;
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
  // std::cout << "entering Run()" << std::endl;
  // sample sensors and store data
  sample_sensors();
  // std::cout << "sampled sensors" << std::endl;
  if (cycle_count_ < 5 || !dynamometer_safe) {
    for (auto& cmd : *output) {
      // We start everything with a stopped command to clear faults.
      cmd.mode = moteus::Mode::kStopped;
    }
    return;
  }

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
  // std::cout << "generated commands" << std::endl;

}

void Dynamometer::set_t0(std::chrono::steady_clock::time_point t0) {
  t0_ = t0;
}

void Dynamometer::swap_actuators() {
  std::swap(actuator_a_idx, actuator_b_idx);
  std::swap(actuator_a_id, actuator_b_id);
}

void Dynamometer::load_replay_data(std::string file) {
  rapidcsv::Document doc(file);
  replay_vel = doc.GetColumn<float>("velocity [rad/s]");
  replay_trq = doc.GetColumn<float>("torque [Nm]");
}

void Dynamometer::run_durability_fsm(mjbots::moteus::PositionCommand &cmda,
  mjbots::moteus::PositionCommand &cmdb) {
  auto fsm_now = std::chrono::steady_clock::now();
  switch (dts) {
    case DurabilityTestState::kIdle: {
      if (fsm_now > fsm_timer_end) {
        start_fsm_timer(2);
        dts = DurabilityTestState::kDurabilityTorqueVelSweep;
      }
      cmda.kp_scale = 0; cmda.kd_scale = 0;
      cmda.feedforward_torque = 0;
      cmdb.kp_scale = 0; cmdb.kd_scale = 0;
      cmdb.feedforward_torque = 0;
      break;
      }
    case DurabilityTestState::kFollow:{
      // step index
      // get new torque and vel values
      // assign them into commands
      float vel = dynset_.replay_vel_scale * replay_vel[replay_idx];
      float trq = dynset_.replay_trq_scale * replay_trq[replay_idx];
      ++replay_idx;
      
      cmda.kp_scale = 0; cmda.kd_scale = 0;
      cmda.feedforward_torque = trq;
      
      cmdb.kp_scale = 1; cmdb.kd_scale = 1;
      cmdb.velocity = -vel;
      // cmdb.feedforward_torque = trq;

      break;
      }
    case DurabilityTestState::kDurabilityGRP:{
      if (fsm_now > fsm_timer_end) {
        start_fsm_timer(15);
        dts = DurabilityTestState::kIdle;
      }
      float rand_cmd = filtered_random();
      
      cmda.kp_scale = 0; cmda.kd_scale = 0;
      cmda.feedforward_torque = rand_cmd;
      
      cmdb.kp_scale = 0; cmdb.kd_scale = 0;
      cmdb.feedforward_torque = 0;
      break;}
    case DurabilityTestState::kDurabilityTorqueVelSweep:
      if (fsm_now > fsm_timer_end) {
        start_fsm_timer(2);
        dts = DurabilityTestState::kIdle;
        swap_actuators();
      }
      cmda.kp_scale = 0; cmda.kd_scale = 0;
      // cmda.position = std::numeric_limits<double>::quiet_NaN();
      // cmda.velocity = std::numeric_limits<double>::quiet_NaN();
      cmda.feedforward_torque = 0.2;
      
      cmdb.kp_scale = 1; cmdb.kd_scale = 1;
      cmdb.position = std::numeric_limits<double>::quiet_NaN();
      cmdb.velocity = -4;
      break;
    case DurabilityTestState::kDurabilityNone: {
      cmda.kp_scale = 0; cmda.kd_scale = 0;
      cmda.feedforward_torque = 0;
      cmdb.kp_scale = 0; cmdb.kd_scale = 0;
      cmdb.feedforward_torque = 0;
      break;
      }
  }
}

void Dynamometer::generate_commands(double time,
  mjbots::moteus::PositionCommand &cmda,
  mjbots::moteus::PositionCommand &cmdb) {
  cmda.position = 0;
  cmda.velocity = 0;
  cmda.feedforward_torque = 0;
  cmdb.position = 0;
  cmdb.velocity = 0;
  cmdb.feedforward_torque = 0;

  // this is superfluous but leaving here for the moment
  if (!dynamometer_safe) {
    cmda.kp_scale = 0; cmda.kd_scale = 0;
    cmdb.kp_scale = 0; cmdb.kd_scale = 0;
    return;
  }

  switch (dynset_.testmode) {
    case TestMode::kTorqueConstant:
      break;
    case TestMode::kDirectDamping:
      break;
    case TestMode::kGRP: {
      float rand_cmd = filtered_random();
      
      cmda.kp_scale = 0; cmda.kd_scale = 0;
      cmda.feedforward_torque = rand_cmd;
      
      cmdb.kp_scale = 0; cmdb.kd_scale = 0;
      cmdb.feedforward_torque = 0;
      break;
      }
    case TestMode::kTorqueVelSweep: {
      // cmda.position = std::sin(t_prog_s_);
      // cmda.velocity = nan("");
      // cmdb.position = std::sin(t_prog_s_);
      // cmdb.velocity = nan("");

      cmda.kp_scale = 0; cmda.kd_scale = 0;
      // cmda.position = std::numeric_limits<double>::quiet_NaN();
      // cmda.velocity = std::numeric_limits<double>::quiet_NaN();
      cmda.feedforward_torque = 0.2;
      
      cmdb.kp_scale = 1; cmdb.kd_scale = 1;
      cmdb.position = std::numeric_limits<double>::quiet_NaN();
      cmdb.velocity = -4;
      // cmdb.feedforward_torque = 0.2;
      // cmdb.feedforward_torque = std::numeric_limits<double>::quiet_NaN();
      break;
      }
    case TestMode::kDurability: {
      run_durability_fsm(cmda, cmdb);
      break;
      }
    case TestMode::kManual:
      break;
    case TestMode::kNone: {
      cmda.kp_scale = 0; cmda.kd_scale = 0;
      cmdb.kp_scale = 0; cmdb.kd_scale = 0;
      break;
      }
  }
}

void Dynamometer::sample_random() {
  std::mt19937 gen(rd_());
  random_sample = realdist(gen);
}

float Dynamometer::filtered_random() {
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

  return rand_cmd;
}

void Dynamometer::sample_sensors() {
  // TODO: Add temp read and calc
  sd_.torque_Nm = 0;
  sd_.temp1_C = 0;
  sd_.temp2_C = 0;

  sd_.ina1_voltage_V = 0;
  sd_.ina1_current_A = 0;
  sd_.ina1_power_W = 0;

  sd_.ina2_voltage_V = 0;
  sd_.ina2_current_A = 0;
  sd_.ina2_power_W = 0;
  
  auto testmode = dynset_.testmode;

  // conditions for reading torque sensor
  if (testmode != TestMode::kDurability) {
    // run faster in GRP mode
    ads_.prime_i2c();
    uint16_t adc = ads_.readADC_SingleEnded(0);
    float torque_volts = ads_.computeVolts(adc);
    float tqsen_gain = (dynset_.tqsen==TorqueSensor::kTRD605_18) ? 18.0/50 : 5.0/5.0;
    sd_.torque_Nm = -(torque_volts - 5.0/3.0) * 3.0;
  }
  // conditions for temp sensors
  if (testmode != TestMode::kGRP) {
    float R0 = 100000;
    float T0 = 25;
    float Rf = 100000;
    float V0 = 3.3;

    ads_.prime_i2c();
    uint16_t adc = ads_.readADC_SingleEnded(2);
    float Vt = ads_.computeVolts(adc);
    // thermistor resistance
    float Rt = Rf*Vt / (V0 - Vt);
    sd_.temp1_C = (1.0/298.15) + (1.0/3950.0)*log10(Rt/R0);
    sd_.temp1_C = 1.0/sd_.temp1_C - 273.15;

    adc = ads_.readADC_SingleEnded(3);
    Vt = ads_.computeVolts(adc);
    Rt = Rf*Vt / (V0 - Vt);
    sd_.temp2_C = (1.0/298.15) + (1.0/3950.0)*log10(Rt/R0);
    sd_.temp2_C = 1.0/sd_.temp2_C - 273.15;
    // std::cout << "sd_.temp1_C - " << sd_.temp1_C << ",\n"
    //   << "sd_.temp2_C = " << sd_.temp2_C << ",\n"
    //   << "Vt = " << Vt << ",\n"
    //   << "Rt = " << Rt << ",\n"
    //   << std::endl;
  }
  // conditions for power meter
  if (testmode != TestMode::kGRP) {
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

void Dynamometer::start_fsm_timer(float seconds) {
  fsm_timer_end = std::chrono::steady_clock::now()
    + std::chrono::milliseconds(static_cast<uint32_t>(1e3 * seconds));
}

void Dynamometer::parse_settings(cxxopts::ParseResult dyn_opts) {
  dynset_.dyn_opts = dyn_opts;
  dynset_.period_s = 1.0/dyn_opts["frequency"].as<float>();
  dynset_.duration_s = dyn_opts["duration"].as<float>();
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
  } else if (test_str == std::string("Durability")) {
    dynset_.testmode = TestMode::kDurability; std::cout << "test mode " << test_str << " selected" << std::endl;
  } else if (test_str == std::string("manual")) {
    dynset_.testmode = TestMode::kManual; std::cout << "test mode " << test_str << " selected" << std::endl;
  } else {
    dynset_.testmode = TestMode::kNone;  std::cout << "no test mode selected. exiting..." << std::endl;
    exit(1);
  }
  
  auto tqsen_str = dyn_opts["torquesensor"].as<std::string>();
  if (tqsen_str == std::string("trd605-18")) dynset_.tqsen = TorqueSensor::kTRD605_18;
  else if (tqsen_str == std::string("trs605-5")) dynset_.tqsen = TorqueSensor::kTRS605_5;
  else dynset_.tqsen = TorqueSensor::kTRS605_5;

  dynset_.main_cpu = dyn_opts["main-cpu"].as<uint8_t>();
  dynset_.can_cpu = dyn_opts["can-cpu"].as<uint8_t>();

  dynset_.replay_vel_scale = dyn_opts["replay-vel-scale"].as<float>();
  dynset_.replay_trq_scale = dyn_opts["replay-trq-scale"].as<float>();
}

bool Dynamometer::safety_check(const std::vector<mjbots::moteus::Pi3HatMoteusInterface::ServoReply>& replies) {
  //
  bool safe = true;
  float trq1;
  float trq2;

  if (replies.size() == 2){
    auto& reply1 = replies[0];
    auto& reply2 = replies[1];

    trq1 = reply1.result.torque;
    trq2 = reply2.result.torque;

    if (fabs(trq1) > 0.5 || fabs(trq2) > 0.5)
      safe &= fabs(trq2-trq1)/trq1 < actuator_torque_disparity_ratio_;
  }
  else std::cerr << "safety check: incorrect number of replies: " << replies.size() << std::endl;

  if (overtemp_latch && sd_.temp1_C < (max_motor_temp_C_ - 15) && sd_.temp2_C < (max_housing_temp_C_ - 15))
    overtemp_latch = false;
  if (sd_.temp1_C > max_motor_temp_C_ || sd_.temp2_C > max_housing_temp_C_)
    overtemp_latch = true;

  safe &= !overtemp_latch;
  if (dynset_.tqsen == Dynamometer::TorqueSensor::kTRS605_5)
    safe &= sd_.torque_Nm < trs605_5_max_torque_Nm_;
  if (dynset_.tqsen == Dynamometer::TorqueSensor::kTRD605_18)
    safe &= sd_.torque_Nm < trd605_18_max_torque_Nm_;
  dynamometer_safe = safe;
  if(!safe) {
    std::cout << "unsafe condition detected!!" << std::endl;
    std::cout << "\tovertemp_latch = " << overtemp_latch
      << ",\n\tsd_.temp1_C = " << sd_.temp1_C
      << ",\n\tsd_.temp2_C = " << sd_.temp2_C
      << ",\n\tmax_motor_temp_C_ = " <<  max_motor_temp_C_
      << ",\n\tmax_housing_temp_C_ = " <<  max_housing_temp_C_
      << ",\n\ttrq1 = " <<  trq1
      << ",\n\ttrq2 = " <<  trq2 << std::endl;

  }
  return safe;
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
    ("p,path", "path to output csv.", cxxopts::value<std::string>()->default_value("/home/pi/embir-modular-leg/dynamometer-data/"))
    ("replay-file", "path to csv of torque, velocity data to replay.", cxxopts::value<std::string>()->default_value(""))
    ("replay-vel-scale", "scale velocity from replay data", cxxopts::value<float>()->default_value("1.0"))
    ("replay-trq-scale", "scale torque from replay data", cxxopts::value<float>()->default_value("1.0"))
    ("gear1", "gear ratio of actuator 1, as a reduction", cxxopts::value<float>()->default_value("1.0"))
    ("gear2", "gear ratio of actuator 2, as a reduction", cxxopts::value<float>()->default_value("1.0"))
    ("actuator-1-id", "actuator 1 CAN ID", cxxopts::value<uint8_t>()->default_value("1"))
    ("actuator-2-id", "actuator 2 CAN ID", cxxopts::value<uint8_t>()->default_value("2"))
    ("actuator-1-bus", "actuator 1 CAN bus", cxxopts::value<uint8_t>()->default_value("3"))
    ("actuator-2-bus", "actuator 2 CAN bus", cxxopts::value<uint8_t>()->default_value("4"))
    ("main-cpu", "main CPU", cxxopts::value<uint8_t>()->default_value("1"))
    ("can-cpu", "CAN CPU", cxxopts::value<uint8_t>()->default_value("2"))
    ("torquesensor", "declare which torque sensor is being used between [trd605-18| trs605-5]", cxxopts::value<std::string>()->default_value("trs605-5"))
    ("duration", "test duration in seconds", cxxopts::value<float>())
    ("frequency", "test sampling and command frequency in Hz", cxxopts::value<float>()->default_value("250"))
    ("test-mode", "choose between [KT|GRP|direct-damping|TV-sweep|manual]", cxxopts::value<std::string>()->default_value("none"))
    ("skip-cal", "skip recalibration")
    ("h,help", "Print usage")
  ;

  // test deps
  return options;
}