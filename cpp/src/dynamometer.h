#include <string>
#include <thread>
#include <vector>

#include "cxxopts/cxxopts.hpp"

void ConfigureRealtime(const uint8_t realtime) {
  {
    cpu_set_t cpuset = {};
    CPU_ZERO(&cpuset);
    CPU_SET(realtime, &cpuset);

    const int r = ::sched_setaffinity(0, sizeof(cpu_set_t), &cpuset);
    if (r < 0) {
      throw std::runtime_error("Error setting CPU affinity");
    }

    std::cout << "Affinity set to " << realtime << "\n";
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

class DynamometerSettings {
  public:
  DynamometerSettings(cxxopts::ParseResult dyn_opts) : dyn_opts_(dyn_opts) {
    period_s = 1.0/dyn_opts_["frequency"].as<float>();
    gear1 = dyn_opts_["gear1"].as<float>();
    gear2 = dyn_opts_["gear2"].as<float>();
    actuator_1_id = dyn_opts_["actuator-1-id"].as<uint8_t>();
    actuator_2_id = dyn_opts_["actuator-2-id"].as<uint8_t>();
    actuator_1_bus = dyn_opts_["actuator-1-bus"].as<uint8_t>();
    actuator_2_bus = dyn_opts_["actuator-2-bus"].as<uint8_t>();

    auto test_str = dyn_opts_["test-mode"].as<std::string>();

    // std::cout << test_str << (test_str == std::string("TV-sweep")) << std::endl;
    if (test_str == std::string("KT")) {
      testmode = TestMode::kTorqueConstant; std::cout << "test mode " << test_str << " selected" << std::endl;
    } else if (test_str == std::string("GRP")) {
      testmode = TestMode::kGRP; std::cout << "test mode " << test_str << " selected" << std::endl;
    } else if (test_str == std::string("direct-damping")) {
      testmode = TestMode::kDirectDamping; std::cout << "test mode " << test_str << " selected" << std::endl;
    } else if (test_str == std::string("TV-sweep")) {
      testmode = TestMode::kTorqueVelSweep; std::cout << "test mode " << test_str << " selected" << std::endl;
    } else if (test_str == std::string("manual")) {
      testmode = TestMode::kManual; std::cout << "test mode " << test_str << " selected" << std::endl;
    } else {
      testmode = TestMode::kNone;  std::cout << "no test mode selected" << std::endl;
    }
    
    auto tqsen_str = dyn_opts_["torquesensor"].as<std::string>();
    if (tqsen_str == std::string("trd605-18")) tqsen = TorqueSensor::kTRD605_18;
    else if (tqsen_str == std::string("trs605-5")) tqsen = TorqueSensor::kTRS605_5;
    else tqsen = TorqueSensor::kTRS605_5;

    main_cpu = dyn_opts_["main-cpu"].as<uint8_t>();
    can_cpu = dyn_opts_["can-cpu"].as<uint8_t>();
  }

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

  private:
  cxxopts::ParseResult dyn_opts_;
};

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