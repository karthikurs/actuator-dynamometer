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
#include <ctime>
#include <csignal>

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

volatile sig_atomic_t interrupted=false; 

void sig_handle(int s) {
  interrupted = true;
}

std::string stringify_moteus_reply(MoteusInterface::ServoReply& reply) {
  uint8_t id = reply.id;
  auto& data = reply.result;
  std::ostringstream result;
  sprintf(cstr_buffer, "%2d,% -f,% -f,",
    data.mode, data.position, data.velocity);
  result << cstr_buffer;
  sprintf(cstr_buffer, "% -f,% -f,%2d",
    data.torque, data.temperature, data.fault);
  result << cstr_buffer;
  return result.str();
}

std::string stringify_moteus_reply_header(uint8_t id) {
  std::ostringstream result;
  std::string cN = "c"+std::to_string(id)+" ";
  result << cN << "mode," << cN << "position [rev]," << cN << "velocity [Hz],"
    << cN << "torque [Nm]," << cN << "temp [C]," << cN << "fault";
  return result.str();
}

std::string stringify_actuator(MoteusInterface::ServoCommand command,
  MoteusInterface::ServoReply& reply, float gear_reduction) {
  uint8_t id = reply.id;
  std::ostringstream result;
  auto& cmd_data = command.position;
  sprintf(cstr_buffer, "% -f,% -f,% -f,",
    cmd_data.position*2*PI/gear_reduction,
    cmd_data.velocity*2*PI/gear_reduction,
    cmd_data.feedforward_torque*gear_reduction);
  result << cstr_buffer;

  auto& reply_data = reply.result;
  sprintf(cstr_buffer, "% -f,% -f,% -f",
    reply_data.position*2*PI/gear_reduction,
    reply_data.velocity*2*PI/gear_reduction,
    reply_data.torque*gear_reduction);
  result << cstr_buffer;

  if(command.id != reply.id) {std::cout << "\n\nAAAAAAHHHHHH ID MISMATCH -- cmd_id = " << command.id << ", reply_id = " << reply.id << "\n\n";}
  return result.str();
}

std::string stringify_actuator_headers(uint8_t id) {
  std::ostringstream result;
  std::string aN = "a"+std::to_string(id)+" ";
  result << aN << "position cmd [rad]," << aN << "velocity cmd [rad/s]," << aN << "ff torque cmd [Nm],"
    << aN << "position [rad]," << aN << "velocity [rad/s]," << aN << "torque [Nm]";
  return result.str();
}

void Run(Dynamometer* dynamometer, std::ofstream& data_file) {

  // for convenience
  Dynamometer::DynamometerSettings& dynset = dynamometer->dynset_;

  // * SETUP *

  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = sig_handle;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

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
  const auto status_period = std::chrono::microseconds(dynset.status_period_us);
  const auto grp_sampling_period = std::chrono::microseconds(dynset.grp_sampling_period_us);
  auto next_status = next_cycle + status_period;
  auto next_grp = next_cycle + grp_sampling_period;
  uint64_t cycle_count = 0;
  uint64_t total_skip_count = 0;
  double total_margin = 0.0;
  uint64_t margin_cycles = 0;
  uint64_t reply_miss_count = 0;

  std::string c1_str = "";
  std::string c2_str = "";
  std::string a1_str = "";
  std::string a2_str = "";
  std::string sensor_str = "";

  dynamometer->set_t0(std::chrono::steady_clock::now());

  data_file << "time [s],";
  if(dynset.testmode == Dynamometer::TestMode::kGRP) {
    data_file << "a1 velocity [rad/s],trs605-5 torque [Nm]\n";
  }
  else {
    data_file << stringify_actuator_headers(1) << ","
      << stringify_moteus_reply_header(1) << ","
      << stringify_actuator_headers(2) << ","
      << stringify_moteus_reply_header(2) << ","
      << dynamometer->stringify_sensor_data_headers() << ",actuator_a_id";
    if(dynset.testmode == Dynamometer::TestMode::kDurability)
      data_file << ",durability test mode";
    data_file << "\n";
  }

  // * MAIN LOOP *
  while (!interrupted && dynamometer->get_program_time() < dynset.duration_s && !dynamometer->get_end_program()) {
    cycle_count++;
    margin_cycles++;
    // std::cout << "loop" << std::endl;
    // Terminal status update
    {
      const auto now = std::chrono::steady_clock::now();
      if (now > next_status) {
        // Insert code here that you want to run at a lower frequency
        next_status += status_period;
        total_margin = 0;
        margin_cycles = 0;
        // data_file.flush();

        dynamometer->print_status_update();
      }

      int skip_count = 0;
      while (now > next_cycle) {
        skip_count++;
        next_cycle += period;
      }
      total_skip_count += skip_count;
      if (skip_count) {
        std::cout << "Skipped " << total_skip_count << "/" << cycle_count <<" cycles in total" << std::endl;
        // data_file << "\n# Skipped " << skip_count << " cycles\n";
      }
      if (skip_count > 50) {
        std::cout << std::endl;
        data_file.close();
        std::exit(EXIT_FAILURE);
      }
    }

    {
      const auto now = std::chrono::steady_clock::now();
      if (now > next_grp) {
        dynamometer->sample_random();
        next_grp += grp_sampling_period;
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
      reply_miss_count = 0;
      uint8_t cmd1_idx = (saved_commands.at(0).id == 1) ? 0 : 1;
      uint8_t cmd2_idx = (cmd1_idx == 0) ? 1 : 0;
      uint8_t rpl1_idx = (saved_replies.at(0).id == 1) ? 0 : 1;
      uint8_t rpl2_idx = (rpl1_idx == 0) ? 1 : 0;


      data_file << std::setw(10) << std::setprecision(4) << std::fixed
        << (dynamometer->get_program_time()) << ",";
      if (dynset.testmode == Dynamometer::TestMode::kGRP) {
        data_file << std::setw(10) << std::setprecision(2) << std::fixed
          << saved_replies.at(rpl1_idx).result.velocity*2*PI/dynset.gear1 << ",";
        data_file << std::setw(10) << std::setprecision(2) << std::fixed
          << (dynamometer->get_torque()) << "\n";
      }
      else {
        c1_str = stringify_moteus_reply(saved_replies.at(rpl1_idx));
        c2_str = stringify_moteus_reply(saved_replies.at(rpl2_idx));
        a1_str = stringify_actuator(saved_commands.at(cmd1_idx), saved_replies.at(rpl1_idx), dynset.gear1);
        a2_str = stringify_actuator(saved_commands.at(cmd2_idx), saved_replies.at(rpl2_idx), dynset.gear2);
        sensor_str = dynamometer->stringify_sensor_data();
        data_file << a1_str << "," << c1_str << ","
          << a2_str << "," << c2_str << "," << sensor_str << "," << dynamometer->get_actuator_a_id();
        if(dynset.testmode == Dynamometer::TestMode::kDurability)
          data_file << "," << (int)(dynamometer->get_dts());
        data_file << "\n";
      }
      // data_file.flush();
    }
    else if (cycle_count > 5 && saved_replies.size() < 2) {
      // data_file << "# missing moteus reply" << std::endl;
      // std::cout << "# missing moteus reply" << std::endl;
      reply_miss_count++;
    }

    // kill loop if we miss all these replies
    if (reply_miss_count > 20) {
      break;
      std::cout << "missed too many replies! ending..." << std::endl;
    }

    if (cycle_count > 1) std::copy(commands.begin(), commands.end(), saved_commands.begin());
    dynamometer->safety_check(saved_replies);
  }
  // IF INTERRUPTED
  std::cout << std::endl << "exiting..." << std::endl;

  // while (!can_result.valid()) {
  //   // do nothing
  // }
  // for (auto& cmd : moteus_data.commands) cmd.mode = moteus::Mode::kStopped;
  // moteus_interface.Cycle(moteus_data, nullptr);
  data_file.close();
  std::exit(EXIT_SUCCESS);
}


int main(int argc, char** argv) {
  auto options = dyn_opts();
  auto opts = options.parse(argc, argv);
  std::cout << "user comment: " << opts["comment"].as<std::string>() << std::endl;

  std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
  std::time_t nowc = std::chrono::system_clock::to_time_t(now);
  std::ostringstream filename_stream;
  filename_stream << std::put_time(std::localtime(&nowc), "dynamometer_test_%d_%m_%Y_%H-%M-%S.csv");
  std::string filename = filename_stream.str();
  std::cout << "outputting to file \"" << filename << "\"" << std::endl;

  // std::ofstream data_file("/home/pi/embir-modular-leg/dynamometer-data/"+filename);
  std::ofstream data_file(opts["path"].as<std::string>()+filename);

  std::vector<std::string> arg_list(argv, argv+argc);
  data_file << "# ";
  for (auto arg_str : arg_list) data_file << arg_str << " ";
  data_file << std::endl;
  // return 0;

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
  if (dynset.testmode == Dynamometer::TestMode::kDurability) {
    // Load mini cheetah test file.
    auto replay_file = opts["replay-file"].as<std::string>();
    if (replay_file == "") {
      std::cerr << "No replay file specified! Exiting..." << std::endl;
      return 1;
    }
    dynamometer.load_replay_data(replay_file);
  }
  data_file << "# \n# user comment: " << opts["comment"].as<std::string>() << "\n# \n";
  data_file << "# test mode: " << opts["test-mode"].as<std::string>() << "\n";
  data_file << "# period s: " << 1.0/opts["frequency"].as<float>() << "\n";
  data_file << "# duration s: " << opts["duration"].as<float>() << "\n";
  data_file << "# gear 1: " << opts["gear1"].as<float>() << "\n";
  data_file << "# gear 2: " << opts["gear2"].as<float>() << "\n";
  data_file << "# actuator 1 id: " << (int)(opts["actuator-1-id"].as<uint8_t>()) << "\n";
  data_file << "# actuator 2 id: " << (int)(opts["actuator-2-id"].as<uint8_t>()) << "\n";
  data_file << "# actuator 1 bus: " << (int)(opts["actuator-1-bus"].as<uint8_t>()) << "\n";
  data_file << "# actuator 2 bus: " << (int)(opts["actuator-2-bus"].as<uint8_t>()) << "\n";
  data_file << "# torque sensor: " << opts["torquesensor"].as<std::string>() << "\n";
  data_file << "# main cpu: " << (int)(opts["main-cpu"].as<uint8_t>()) << "\n";
  data_file << "# can cpu: " << (int)(opts["can-cpu"].as<uint8_t>()) << "\n# \n";
  data_file << "# grp.json:\n#     " << dynamometer.grp_j << std::endl;
  data_file << "# safety_limits.json:\n#     " << dynamometer.safety_j << std::endl;
  data_file << "# durability.json:\n#     " << dynamometer.durability_j << std::endl;
  if (dynset.testmode == Dynamometer::TestMode::kDurability)
    data_file << "# replay file:\n#     " << opts["replay-file"].as<std::string>() << std::endl;
  if (dynset.testmode == Dynamometer::TestMode::kStep)
    data_file << "# step.json:\n#     " << dynamometer.step_j << std::endl;


  ConfigureRealtime(dynset.main_cpu);
  ConfigureRealtime(dynset.can_cpu);

  // return 0;
  Run(&dynamometer, data_file);

  data_file.close();
  return 0;
}