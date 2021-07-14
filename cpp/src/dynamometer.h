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
#include "nlohmann/json.hpp"


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
    kDurability,
    kStep,
    kManual,
    kNone
  };

  enum DurabilityTestState : uint8_t {
    kIdle,
    kFollow,
    kDurabilityGRP,
    kDurabilityTorqueVelSweep,
    kDurabilityNone
  };

  struct DynamometerSettings {
    float period_s;
    float duration_s;
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

    uint32_t grp_sampling_period_us;
    uint32_t status_period_us;

    float replay_vel_scale;
    float replay_trq_scale;
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

  void load_replay_data(std::string file);

  void run_durability_fsm(mjbots::moteus::PositionCommand &cmda,
    mjbots::moteus::PositionCommand &cmdb);

  void generate_commands(double time, mjbots::moteus::PositionCommand &cmda, mjbots::moteus::PositionCommand &cmdb);
  
  void sample_random();

  float filtered_random();

  void sample_sensors();

  std::string stringify_sensor_data();

  std::string stringify_sensor_data_headers();

  inline double get_program_time() {return t_prog_s_;}

  inline double get_function_time() {return t_func_s_;}

  void start_fsm_program_timer(float seconds);

  void start_fsm_function_timer(float seconds);

  void parse_settings(cxxopts::ParseResult dyn_opts);

  inline float get_torque() {return sd_.torque_Nm;}

  inline uint8_t get_actuator_a_id() {return actuator_a_id;}

  inline DurabilityTestState get_dts() {return dts;}
  
  inline bool get_end_program() {return end_program;}

  void print_status_update();

  bool safety_check(const std::vector<mjbots::moteus::Pi3HatMoteusInterface::ServoReply>& replies);

  DynamometerSettings dynset_;
  nlohmann::json grp_j;
  nlohmann::json safety_j;
  nlohmann::json durability_j;
  nlohmann::json step_j;

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

  float grp_max_ampl = 1.0; //Nm

  std::chrono::steady_clock::time_point t0_;
  double t_prog_s_ = 0;
  double t_prog_old_s_ = 0;
  double t_func_s_ = 0;

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

  DurabilityTestState dts = DurabilityTestState::kIdle;
  DurabilityTestState dts_after_idle = DurabilityTestState::kFollow;
  double fsm_program_timer_end = 0;
  double fsm_function_timer_end = 0;

  float encoder_offset = 0;

  float kFollow_duration_S = 0;
  float kDurabilityGRP_duration_S = 0;
  float kTorqueVelSweep_condition_duration_S = 0;
  std::vector<float> kTorqueVelSweep_velocities_rad_s;
  std::vector<float> kTorqueVelSweep_torques_Nm;
  std::vector<float> sweep_vel;
  std::vector<float> sweep_trq;
  uint32_t sweep_idx = 0;

  std::vector<float> replay_vel;
  std::vector<float> replay_trq;

  uint32_t replay_idx = 0;

  bool overtemp_latch = false;
  bool dynamometer_safe = true;
  bool encoder_offset_set = false;

  float max_motor_temp_C_;
  std::vector<float> motor_temp_buffer; 
  float max_housing_temp_C_;
  std::vector<float> housing_temp_buffer; 
  float trs605_5_max_torque_Nm_;
  float trd605_18_max_torque_Nm_;
  float actuator_torque_disparity_ratio_;
  std::vector<float> disparity_buffer;

  float step_mag_Nm_ = 1;
  float step_temp_ceiling_C_ = 60;
  float step_temp_floor_C_ = 30;
  bool step_temp_latch = false;

  uint32_t num_swaps = 0;
  bool end_program = false;
};
