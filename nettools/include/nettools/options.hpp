#ifndef NETTOOLS__OPTIONSMONITOR_HPP_
#define NETTOOLS__OPTIONSMONITOR_HPP_

#include <string>
#include <vector>

#include "rmw/types.h"

/// Find "option" in the argument vector.
/**
 * \param[in] args The argument vector
 * \param[in] option The option to search for
 * \return True if option was found in args, false otherwise.
 */
bool find_command_option(
  const std::vector<std::string> & args, const std::string & option);

/// Get the value corresponding to option.
/**
 * \param[in] args The argument vector to search in
 * \param[in] option The option to search for
 * \return The value that comes after "option"
 */
std::string get_command_option(
  const std::vector<std::string> & args, const std::string & option);

/// Parse the C-style argument vector and return demo-specific parameters.
/**
 * \param[in] argc Size of the argument vector.
 * \param[in] argv Argument vector, an array of C-style strings.
 * \param[in] depth The queue size for the KEEP_LAST QoS policy.
 * \param[in] reliability_policy The reliability policy (RELIABLE or BEST_EFFORT).
 * \param[in] name of topic to subscribe to.
 * \param[in] message type of the topic.
 */
bool parse_command_options(
  int argc, char ** argv, size_t * depth,
  rmw_qos_reliability_policy_t * reliability_policy,
  rmw_qos_history_policy_t * history_policy,
  std::string * msg_type = nullptr,
  std::string * topic = nullptr);

#endif  //
