#pragma once

#include <sys/mman.h>

#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <random>
#include <cstdio>

#include "mjbots/moteus/moteus_protocol.h"
#include "mjbots/moteus/pi3hat_moteus_interface.h"

#include "sensors/Adafruit_ADS1X15.h"
#include "sensors/Adafruit_INA260.h"

#include "cxxopts/cxxopts.hpp"

void LockMemory();

std::pair<double, double> MinMaxVoltage(
    const std::vector<mjbots::moteus::Pi3HatMoteusInterface::ServoReply>& r);

void ConfigureRealtime(const uint8_t realtime);

cxxopts::Options dyn_opts();

class Dynamometer {
 public:
  struct SensorData {
    float torque_Nm;
    float ina1_current_A;
    float ina1_voltage_V;
    float ina1_power_W;
    float ina2_current_A;
    float ina2_voltage_V;
    float ina2_power_W;
    float temp1_C;
    float temp2_C;
  };

  // interpret command line options to settings class
  enum TorqueSensor : uint8_t {
    kTRD605_18,
    kTRS605_5
  };

  enum TestMode : uint8_t {
    kTorqueConstant,
    kGRP,
    kDirectDamping,
    kTorqueVelSweep,
    kManual,
    kNone
  };

  struct DynamometerSettings {
    float period_s;
    float gear1;
    float gear2;

    uint8_t actuator_1_id;
    uint8_t actuator_1_bus;
    uint8_t actuator_2_id;
    uint8_t actuator_2_bus;

    uint8_t main_cpu;
    uint8_t can_cpu;

    TestMode testmode;
    TorqueSensor tqsen;

    cxxopts::ParseResult dyn_opts;
  };


  Dynamometer(cxxopts::ParseResult dyn_opts, Adafruit_ADS1015 &ads,
  Adafruit_INA260 &ina1, Adafruit_INA260 &ina2);

  /// This is called before any control begins, and must return the
  /// set of servos that are used, along with which bus each is
  /// attached to.
  std::map<int, int> servo_bus_map() const;

  /// This is also called before any control begins.  @p commands will
  /// be pre-populated with an entry for each servo returned by
  /// 'servo_bus_map'.  It can be used to perform one-time
  /// initialization like setting the resolution of commands and
  /// queries.
  void Initialize(std::vector<mjbots::moteus::Pi3HatMoteusInterface::ServoCommand>* commands);

  mjbots::moteus::QueryResult Get(const std::vector<mjbots::moteus::Pi3HatMoteusInterface::ServoReply>& replies, int id);

  /// This is run at each control cycle.  @p status is the most recent
  /// status of all servos (note that it is possible for a given
  /// servo's result to be omitted on some frames).
  ///
  /// @p output should hold the desired output.  It will be
  /// pre-populated with the result of the last command cycle, (or
  /// Initialize to begin with).
  void Run(const std::vector<mjbots::moteus::Pi3HatMoteusInterface::ServoReply>& status,
           std::vector<mjbots::moteus::Pi3HatMoteusInterface::ServoCommand>* output);

  void set_t0(std::chrono::steady_clock::time_point t0);

  void swap_actuators();

  void generate_commands(double time, mjbots::moteus::PositionCommand &cmda, mjbots::moteus::PositionCommand &cmdb);
  
  void sample_random();

  void sample_sensors();

  std::string stringify_sensor_data();

  std::string stringify_sensor_data_headers();

  double get_program_time();

  void parse_settings(cxxopts::ParseResult dyn_opts);

  DynamometerSettings dynset_;
 private:
  char cstr_buffer[128];
  Adafruit_ADS1015 ads_;
  Adafruit_INA260 ina1_;
  Adafruit_INA260 ina2_;

  SensorData sd_;

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

  double *lpf_dcof_;
  int *lpf_ccof_;
  double lpf_sf_;
  uint8_t lpf_order_ = 3;
  float lpf_fc_ = 40;
  std::vector<float> fib_;
  std::vector<float> fob_;

  std::random_device rd_;
  std::uniform_real_distribution<> realdist;
  float random_sample = 0;
};
