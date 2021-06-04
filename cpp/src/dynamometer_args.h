#include <string>
#include <thread>
#include <vector>

#include "cxxopts/cxxopts.hpp"

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
  kManual
};

class DynamometerSettings {
  public:
  DynamometerSettings(cxxopts::Options dyn_opts) : dyn_opts_(dyn_opts) {}

  uint8_t actuator_1_id;
  uint8_t actuator_1_bus;
  uint8_t actuator_2_id;
  uint8_t actuator_2_bus;
  float frequency;

  float gear1;
  float gear2;

  TestMode testmode;
  TorqueSensor tqsen;

  private:
  cxxopts::Options dyn_opts_;
};

cxxopts::Options dyn_opts() {
  cxxopts::Options options("dynamometer", "Run dual actuator dynamometer for characterization");

  options.add_options()
    ("c,comment", "enter comment string to be included in output csv.", cxxopts::value<std::string>())
    ("gear1", "gear ratio of actuator 1, as a reduction", cxxopts::value<float>()->default_value("1.0"))
    ("gear2", "gear ratio of actuator 2, as a reduction", cxxopts::value<float>()->default_value("1.0"))
    ("torquesensor", "declare which torque sensor is being used between {trd605-18, trs605-5}", cxxopts::value<std::string>()->default_value("trs605-5"))
    ("duration", "test duration in seconds", cxxopts::value<float>())
    ("standard-test", "run a standardized test", cxxopts::value<std::string>())
    ("manual-test", "manually configure test", cxxopts::value<bool>()->default_value("true"))
    ("h,help", "Print usage")
  ;

  return options;
}