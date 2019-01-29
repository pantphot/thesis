#include <algorithm>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "nettools/options.hpp"

bool find_command_option(const std::vector<std::string> & args, const std::string & option)
{
  return std::find(args.begin(), args.end(), option) != args.end();
}

std::string get_command_option(const std::vector<std::string> & args, const std::string & option)
{
  auto it = std::find(args.begin(), args.end(), option);
  if (it != args.end() && ++it != args.end()) {
    return *it;
  }
  return std::string();
}

bool get_flag_option(const std::vector<std::string> & args, const std::string & option)
{
  auto it = std::find(args.begin(), args.end(), option);
  if (it != args.end()) {
    return true;
  }
  return false;
}

bool parse_command_options(
  int argc, char ** argv, size_t * depth,
  rmw_qos_reliability_policy_t * reliability_policy,
  rmw_qos_history_policy_t * history_policy,
  int * freq,
  int * step)
{
  std::vector<std::string> args(argv, argv + argc);

  if (find_command_option(args, "-h")) {
    std::stringstream ss;
    ss << "Usage:" << std::endl;
    ss << " -h: This message." << std::endl;
    ss << " -r: Reliability QoS setting:" << std::endl;
    ss << "    0 - best effort" << std::endl;
    ss << "    1 - reliable (default)" << std::endl;
    ss << " -d: Depth of the queue: only honored if used together with 'keep last'. " <<
      "10 (default)" << std::endl;
    ss << " -k: History QoS setting:" << std::endl;
    ss << "    0 - only store up to N samples, configurable via the queue depth (default)" <<
      std::endl;
    ss << "    1 - keep all the samples" << std::endl;
    ss << " -f: Publish frequency in Hz. 30 (default)" << std::endl;
    ss << " -s: Message length increment step in kB. (default = 1kB)" << std::endl;
    std::cout << ss.str();
    return false;
  }

  auto depth_str = get_command_option(args, "-d");
  if (!depth_str.empty()) {
    *depth = std::stoul(depth_str.c_str());
  }

  if (freq != nullptr) {
    auto freq_str = get_command_option(args, "-f");
    if (!freq_str.empty()) {
      *freq = std::stoi(freq_str.c_str());
    }
  }

  if (step != nullptr) {
    auto step_str = get_command_option(args, "-s");
    if (!step_str.empty()) {
      *step = std::stoi(step_str.c_str());
    }
  }

  auto reliability_str = get_command_option(args, "-r");
  if (!reliability_str.empty()) {
    unsigned int r = std::stoul(reliability_str.c_str());
    *reliability_policy =
      r ? RMW_QOS_POLICY_RELIABILITY_RELIABLE : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  }

  auto history_str = get_command_option(args, "-k");
  if (!history_str.empty()) {
    unsigned int r = std::stoul(history_str.c_str());
    *history_policy = r ? RMW_QOS_POLICY_HISTORY_KEEP_ALL : RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  }

  return true;
}
