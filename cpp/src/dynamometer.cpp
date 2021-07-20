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
#include <cctype>

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


#include <ostream>
namespace Color {
  enum Code {
    FG_BLACK    = 30,
    FG_RED      = 31,
    FG_GREEN    = 32,
    FG_YELLOW   = 33,
    FG_BLUE     = 34,
    FG_DEFAULT  = 39,
    BG_RED      = 41,
    BG_GREEN    = 42,
    BG_YELLOW   = 43,
    BG_BLUE     = 44,
    BG_WHITE    = 47,
    BG_DEFAULT  = 49
  };
  class Modifier {
    Code code;
  public:
    Modifier(Code pCode) : code(pCode) {}
    friend std::ostream&
    operator<<(std::ostream& os, const Modifier& mod) {
      return os << "\033[" << mod.code << "m";
    }
    Modifier& operator=(const Modifier& mod) {
      this->code = mod.code;
      return *this;
    }
  };
}

Color::Modifier fg_blk(Color::FG_BLACK);
Color::Modifier fg_red(Color::FG_RED);
Color::Modifier fg_grn(Color::FG_GREEN);
Color::Modifier fg_blu(Color::FG_BLUE);
Color::Modifier fg_yel(Color::FG_YELLOW);
Color::Modifier fg_def(Color::FG_DEFAULT);

Color::Modifier bg_red(Color::BG_RED);
Color::Modifier bg_grn(Color::BG_GREEN);
Color::Modifier bg_blu(Color::BG_BLUE);
Color::Modifier bg_yel(Color::BG_YELLOW);
Color::Modifier bg_wht(Color::BG_WHITE);
Color::Modifier bg_def(Color::BG_DEFAULT);

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
  motor_temp_buffer.resize(8);
  housing_temp_buffer.resize(8);
  trs605_5_max_torque_Nm_ = safety_j["trs605-5_max_torque_Nm"];
  trd605_18_max_torque_Nm_ = safety_j["trd605-18_max_torque_Nm"];
  actuator_torque_disparity_ratio_ = safety_j["actuator_torque_disparity_ratio"];
  disparity_buffer.resize(8);

  std::ifstream durability_if("configs/durability.json");
  durability_if >> durability_j;
  kFollow_duration_S = durability_j["kFollow_duration_S"];
  kDurabilityGRP_duration_S = durability_j["kDurabilityGRP_duration_S"];
  kTorqueVelSweep_condition_duration_S = durability_j["kTorqueVelSweep_condition_duration_S"];
  kTorqueVelSweep_velocities_rad_s = durability_j["kTorqueVelSweep_velocities_rad_s"].get<std::vector<float>>();
  kTorqueVelSweep_torques_Nm = durability_j["kTorqueVelSweep_torques_Nm"].get<std::vector<float>>();

  std::ifstream step_if("configs/step.json");
  step_if >> step_j;
  step_mag_Nm_ = step_j["torque_step_mag_Nm"];
  step_temp_ceiling_C_ = step_j["temp_ceiling_C"];
  step_temp_floor_C_ = step_j["temp_floor_C"];


  for (float trq : kTorqueVelSweep_torques_Nm) {
    for (float vel : kTorqueVelSweep_velocities_rad_s) {
      // std::cout << trq << ", " << vel << std::endl;
      sweep_trq.push_back(trq);
      sweep_trq.push_back(trq);
      sweep_trq.push_back(0);
      sweep_trq.push_back(-trq);
      sweep_trq.push_back(-trq);
      sweep_trq.push_back(0);

      sweep_vel.push_back(vel);
      sweep_vel.push_back(-vel);
      sweep_vel.push_back(0);
      sweep_vel.push_back(vel);
      sweep_vel.push_back(-vel);
      sweep_vel.push_back(0);
    }
  }

  dynset_.status_period_us = static_cast<int64_t>(
    (1e6)/10);

  std::uniform_real_distribution<> dist(-grp_max_ampl, grp_max_ampl);
  realdist = dist;

  if (!(dyn_opts["skip-cal"].as<bool>())) {
    std::cout << "restoring calibrations... \n";
    system("python3 -m moteus.moteus_tool --target 1 --pi3hat-cfg '3=1;4=2' --restore-cal /home/pi/embir-modular-leg/moteus-setup/moteus-cal/ri50_cal_1_dyn.log");
    system("python3 -m moteus.moteus_tool --target 2 --pi3hat-cfg '3=1;4=2' --restore-cal /home/pi/embir-modular-leg/moteus-setup/moteus-cal/ri50_cal_2_dyn.log");
  }
  if (dyn_opts["swap-actuators"].as<bool>()) swap_actuators();
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
  
  if(dynamometer_safe) t_func_s_ += t_prog_s_ - t_prog_old_s_;
  t_prog_old_s_ = t_prog_s_;
  // std::cout << "entering Run()" << std::endl;
  // sample sensors and store data
  sample_sensors();
  // std::cout << "sampled sensors" << std::endl;
  if (!encoder_offset_set) {
    if (status.size() >= 2 && cycle_count_ >= 5) {
      encoder_offset = status[0].result.position + status[1].result.position;
      std::cout << "initial encoder offset = " << encoder_offset << std::endl;
      encoder_offset_set = true;
    }
    else {
      for (auto& cmd : *output) {
        // We start everything with a stopped command to clear faults.
        cmd.mode = moteus::Mode::kStopped;
      }
      return;
    };
  }
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
  if (dts == DurabilityTestState::kDurabilityGRP)
    actuator_b_out.mode = moteus::Mode::kStopped; // set load actuator to stopped for this one
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
  double fsm_prog_now = t_prog_s_;
  double fsm_func_now = t_func_s_;
  switch (dts) {
    case DurabilityTestState::kIdle: {
      if (fsm_func_now > fsm_function_timer_end && dts_after_idle == DurabilityTestState::kFollow) {
        start_fsm_function_timer(kFollow_duration_S);
        dts = DurabilityTestState::kFollow;
        std::cout << std::endl << "transitioning from Idle to Follow" << std::endl;
      }
      else if (fsm_func_now > fsm_function_timer_end && dts_after_idle == DurabilityTestState::kDurabilityGRP) {
        start_fsm_function_timer(kDurabilityGRP_duration_S);
        dts = DurabilityTestState::kDurabilityGRP;
        std::cout << std::endl << "transitioning from Idle to DurabilityGRP" << std::endl;
      }
      else if (fsm_func_now > fsm_function_timer_end && dts_after_idle == DurabilityTestState::kDurabilityTorqueVelSweep) {
        start_fsm_function_timer(kTorqueVelSweep_condition_duration_S);
        dts = DurabilityTestState::kDurabilityTorqueVelSweep;
        std::cout << std::endl << "transitioning from Idle to DurabilityTorqueVelSweep" << std::endl;
      }
      if (num_swaps >= 2) {
        std::cout << "ran thru one cycle for each actuator; exiting..." << std::endl;
        end_program = true;
      }
      cmda.kp_scale = 0; cmda.kd_scale = 0;
      cmda.position = std::numeric_limits<double>::quiet_NaN();
      cmda.feedforward_torque = 0;

      cmdb.kp_scale = 0; cmdb.kd_scale = 0;
      cmdb.position = std::numeric_limits<double>::quiet_NaN();
      cmdb.feedforward_torque = 0;
      break;
      }
    case DurabilityTestState::kFollow:{
      if (fsm_func_now > fsm_function_timer_end) {
        start_fsm_function_timer(10);
        dts = DurabilityTestState::kIdle;
        dts_after_idle = DurabilityTestState::kDurabilityGRP;
        std::cout << std::endl << "transitioning from Follow to Idle" << std::endl;
      }
      // step index
      // get new torque and vel values
      // assign them into commands
      float vel = dynset_.replay_vel_scale * replay_vel[replay_idx] * dynset_.gear2 / (2*PI);
      float trq = dynset_.replay_trq_scale * replay_trq[replay_idx] / dynset_.gear1;
      ++replay_idx;
      if (replay_idx == replay_trq.size()) {
        std::cout << "looping replay file and swapping actuators (30s pause)..." << std::endl;
        replay_idx = 0;
        swap_actuators();
        num_swaps++;
        start_fsm_function_timer(30);
        dts = DurabilityTestState::kIdle;
        dts_after_idle = DurabilityTestState::kFollow;
        std::cout << std::endl << "transitioning from Follow to Idle" << std::endl;
      }
      
      cmda.kp_scale = 0; cmda.kd_scale = 0;
      cmda.feedforward_torque = trq;
      
      cmdb.kp_scale = 0.4; cmdb.kd_scale = 0.2;
      cmdb.position = std::numeric_limits<double>::quiet_NaN();
      cmdb.velocity = -vel;
      cmdb.feedforward_torque = 0.95*trq;
      // cmdb.feedforward_torque = trq;

      break;
      }
    case DurabilityTestState::kDurabilityGRP:{
      if (fsm_func_now > fsm_function_timer_end) {
        start_fsm_function_timer(10);
        dts = DurabilityTestState::kIdle;
        dts_after_idle = DurabilityTestState::kDurabilityTorqueVelSweep;
        std::cout << std::endl << "transitioning from DurabilityGRP to Idle" << std::endl;
      }
      float rand_cmd = filtered_random();
      
      cmda.kp_scale = 0; cmda.kd_scale = 0;
      cmda.feedforward_torque = rand_cmd;
      
      cmdb.kp_scale = 0; cmdb.kd_scale = 0;
      cmdb.feedforward_torque = 0;
      break;}
    case DurabilityTestState::kDurabilityTorqueVelSweep: {
      // transition when we're on the last index and timer has run out
      if (sweep_idx >= (sweep_vel.size()-1) && fsm_func_now > fsm_function_timer_end) {
        start_fsm_function_timer(10);
        dts = DurabilityTestState::kIdle;
        dts_after_idle = DurabilityTestState::kFollow;
        std::cout << std::endl << "transitioning from DurabilityTorqueVelSweep to Idle" << std::endl;
        sweep_idx = 0; // reset index for next round
      }
      // incremember condition when there is room to increment and
      // we've run for condition duration
      if (sweep_idx < (sweep_vel.size()-1) && fsm_func_now > fsm_function_timer_end) {
        start_fsm_function_timer(kTorqueVelSweep_condition_duration_S);
        sweep_idx++;
      }
      float trq_condition = sweep_trq[sweep_idx] / dynset_.gear1;
      float vel_condition = sweep_vel[sweep_idx] * dynset_.gear1;
      cmda.kp_scale = 0; cmda.kd_scale = 0;
      // cmda.position = std::numeric_limits<double>::quiet_NaN();
      // cmda.velocity = std::numeric_limits<double>::quiet_NaN();
      cmda.feedforward_torque = trq_condition;
      
      cmdb.kp_scale = 2; cmdb.kd_scale = 2;
      cmdb.position = std::numeric_limits<double>::quiet_NaN();
      cmdb.velocity = -vel_condition;
      cmdb.feedforward_torque = 0;
      break;
      }
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
    case TestMode::kStep: {
      cmda.kp_scale = 0; cmda.kd_scale = 0;
      if (sd_.temp1_C > step_temp_ceiling_C_) step_temp_latch = true;
      else if (sd_.temp1_C < step_temp_floor_C_ && step_temp_latch) step_temp_latch = false;

      cmda.position = 0;
      cmda.velocity = 0;
      if (!step_temp_latch) cmda.feedforward_torque = step_mag_Nm_;
      else cmda.feedforward_torque = 0;
      
      cmdb.kp_scale = 10; cmdb.kd_scale = 1;
      cmdb.position = 0;
      cmdb.velocity = 0;

      // these commands for slow turn (maybe even heat distribution?)
      cmdb.kp_scale = 1; cmdb.kd_scale = 4;
      cmdb.position = std::numeric_limits<double>::quiet_NaN();
      cmdb.velocity = -1;
      // cmdb.velocity = std::numeric_limits<double>::quiet_NaN();
      cmdb.feedforward_torque = 0;
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
  float t1 = 0;
  float t2 = 0;
  float alpha = 0.1;
  // sd_.temp1_C = 0;
  // sd_.temp2_C = 0;

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
    // TODO: put these into a config json
    float R0 = 100000;
    float T0 = 25;
    float Rf = 100000;
    float V0 = 3.3;

    ads_.prime_i2c();
    uint16_t adc = ads_.readADC_SingleEnded(2);
    float Vt = ads_.computeVolts(adc);
    if (std::isnan(Vt)) std::cout << "\n nan temp 0" << std::endl;
    // thermistor resistance
    float Rt = Rf*Vt / (V0 - Vt);
    if (std::isnan(Rt)) std::cout << "\n nan temp 1" << std::endl;
    
    t1 = (1.0/298.15) + (1.0/3950.0)*log(Rt/R0);
    if (std::isnan(t1)) t1 = sd_.temp1_C;
    else {
      t1 = 1.0/t1 - 273.15;
      if (std::isnan(t1)) std::cout << "\n nan temp 3" << std::endl;
    }
    // exp decay lpf
    t1 = alpha*t1 + (1-alpha)*sd_.temp1_C;

    adc = ads_.readADC_SingleEnded(3);
    Vt = ads_.computeVolts(adc);
    Rt = Rf*Vt / (V0 - Vt);
    t2 = (1.0/298.15) + (1.0/3950.0)*log(Rt/R0);
    if (std::isnan(t2)) t2 = sd_.temp2_C;
    else t2 = 1.0/t2 - 273.15;

    t2 = alpha*t2 + (1-alpha)*sd_.temp2_C;
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
  sd_.temp1_C = t1;
  sd_.temp2_C = t2;
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

void Dynamometer::start_fsm_program_timer(float seconds) {
  fsm_program_timer_end = t_prog_s_ + seconds;
}

void Dynamometer::start_fsm_function_timer(float seconds) {
  fsm_function_timer_end = t_func_s_ + seconds;
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
  // case invariance -- convert to user input to lowercase
  std::transform(test_str.begin(), test_str.end(), test_str.begin(),
    [](unsigned char c){ return std::tolower(c); });

  // std::cout << test_str << (test_str == std::string("TV-sweep")) << std::endl;
  if (test_str == std::string("kt")) dynset_.testmode = TestMode::kTorqueConstant;
  else if (test_str == std::string("grp")) dynset_.testmode = TestMode::kGRP;
  else if (test_str == std::string("direct-damping")) dynset_.testmode = TestMode::kDirectDamping;
  else if (test_str == std::string("tv-sweep")) dynset_.testmode = TestMode::kTorqueVelSweep;
  else if (test_str == std::string("durability")) dynset_.testmode = TestMode::kDurability;
  else if (test_str == std::string("step")) dynset_.testmode = TestMode::kStep;
  else if (test_str == std::string("manual")) dynset_.testmode = TestMode::kManual;
  else {
    dynset_.testmode = TestMode::kNone;
    std::cout << "no test mode selected (input was \"" << test_str << "\"). exiting..." << std::endl;
    exit(1);
  }
  std::cout << "test mode \"" << test_str << "\" selected" << std::endl;
  
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
  float motor_temp = sd_.temp1_C;
  float housing_temp = sd_.temp2_C;

  if (replies.size() == 2){
    auto& reply1 = replies[0];
    auto& reply2 = replies[1];

    trq1 = reply1.result.torque;
    trq2 = reply2.result.torque;

    float disparity = fabs(trq2-trq1)/trq1;

    std::rotate(disparity_buffer.rbegin(),
      disparity_buffer.rbegin()+1, disparity_buffer.rend());
    disparity_buffer[0] = disparity;
    disparity = 0;
    for (float disp : disparity_buffer) disparity += disp;
    disparity /= disparity_buffer.size();

    if ((fabs(trq1) > 0.5 || fabs(trq2) > 0.5) 
      && (fabs(trq2-trq1)/trq1 > actuator_torque_disparity_ratio_)) {
      
      // safe &= false;
      // std::cout << "torque disparity!! trq1 = " << trq1 << ", trq2 = " << trq2 << std::endl;
    }    
    // check for moteus faults:
    // in a fault condition, we latch unsafe and 
    // commands are set to stop mode until the fault clears
    safe &= (reply1.result.fault == 0);
    safe &= (reply2.result.fault == 0);

    float current_encoder_offset = reply1.result.position + reply2.result.position;

    // latch unsafe if encoder offset changes by more than 0.05 rotations,
    // which represents a rotor slip
    safe &= (fabs(current_encoder_offset - encoder_offset) < 0.05);
  }
  else std::cout << "safety check: incorrect number of replies: " << replies.size() << std::endl;

  std::rotate(motor_temp_buffer.rbegin(),
    motor_temp_buffer.rbegin()+1, motor_temp_buffer.rend());
  motor_temp_buffer[0] = motor_temp;
  motor_temp = 0;
  for (float temp : motor_temp_buffer) motor_temp += temp;
  motor_temp /= motor_temp_buffer.size();

  std::rotate(housing_temp_buffer.rbegin(),
    housing_temp_buffer.rbegin()+1, housing_temp_buffer.rend());
  housing_temp_buffer[0] = housing_temp;
  housing_temp = 0;
  for (float temp : housing_temp_buffer) housing_temp += temp;
  housing_temp /= housing_temp_buffer.size();

  if (overtemp_latch && motor_temp < (max_motor_temp_C_ - 15) && housing_temp < (max_housing_temp_C_ - 15))
    overtemp_latch = false;
  if (motor_temp > max_motor_temp_C_ || housing_temp > max_housing_temp_C_)
    overtemp_latch = true;

  safe &= !overtemp_latch;

  if (dynset_.tqsen == Dynamometer::TorqueSensor::kTRS605_5)
    safe &= sd_.torque_Nm < trs605_5_max_torque_Nm_;
  if (dynset_.tqsen == Dynamometer::TorqueSensor::kTRD605_18)
    safe &= sd_.torque_Nm < trd605_18_max_torque_Nm_;
  // safe if voltages are above 32.0V and not 0 (initialized value/i2c error)
  safe &= (sd_.ina1_voltage_V > 32.0 || sd_.ina1_voltage_V < 1.0);
  safe &= (sd_.ina2_voltage_V > 32.0 || sd_.ina2_voltage_V < 1.0);

  dynamometer_safe = safe;
  if(!safe) {
    // std::cout << "unsafe condition detected!!" << std::endl;
    // std::cout << "\tovertemp_latch = " << overtemp_latch
    //   << ",\n\tmotor_temp = " << motor_temp
    //   << ",\n\thousing_temp = " << housing_temp
    //   << ",\n\ttrq1 = " <<  trq1
    //   << ",\n\ttrq2 = " <<  trq2 << std::endl;
  }
  return safe;
}

void Dynamometer::print_status_update() {
  Color::Modifier bg_temp_m(Color::BG_DEFAULT);
  Color::Modifier bg_temp_h(Color::BG_DEFAULT);
  Color::Modifier bg_safe(Color::BG_DEFAULT);
  
  Color::Modifier bg_temp_latch(Color::BG_DEFAULT);

  std::cout << fg_blk << bg_wht << "  t_p:"
    << std::setw(7) << std::setprecision(1) << std::fixed << t_prog_s_
    << "|t_f:"
    << std::setw(7) << std::setprecision(1) << std::fixed << t_func_s_
    << fg_def << bg_def << "|" ;
  if (dynset_.testmode == Dynamometer::TestMode::kDurability) {
    std::cout << "dts:"
      << std::setw(2) << std::setprecision(2) << std::fixed << (int)dts << "|";
  } else if (dynset_.testmode == Dynamometer::TestMode::kStep) {
    if (step_temp_latch) bg_temp_latch = bg_red;
    else bg_temp_latch = bg_grn;
    std::cout << "temp latch:"
      << std::setw(2) << std::setprecision(2) << std::fixed 
        << bg_temp_latch << (int)step_temp_latch << bg_def << "|";
  }

  float temp_m = sd_.temp1_C;
  float temp_h = sd_.temp2_C;
  if (temp_m < 40) bg_temp_m = bg_grn;
  else if (temp_m < 60) bg_temp_m = bg_yel;
  else bg_temp_m = bg_red;

  if (temp_h < 40) bg_temp_h = bg_grn;
  else if (temp_h < 60) bg_temp_h = bg_yel;
  else bg_temp_h = bg_red;
  std::cout << "temp_m:" << bg_temp_m << fg_blk 
    << std::setw(5) << std::setprecision(1) << std::fixed << temp_m << fg_def << bg_def
    << "|temp_h:" << bg_temp_h << fg_blk 
    << std::setw(5) << std::setprecision(1) << std::fixed << temp_h << fg_def << bg_def
    << "|driving id: "
    << bg_blu << fg_def << (int)actuator_a_id << bg_def;
  
  if (dynamometer_safe) std::cout << "|" << bg_grn << fg_blk << "**SAFE**";
  else std::cout << "|" << bg_red << fg_blk << "*UNSAFE*";
  std::cout << fg_def << bg_def << "\r";
  std::cout.flush();
  return;
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
    ("swap-actuators", "start with actuator roles swapped (driving vs. load)")
    ("h,help", "Print usage")
  ;

  // test deps
  return options;
}