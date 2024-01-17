/***********************************************************
 *                                                         *
 * Copyright (c)                                           *
 *                                                         *
 * The Verifiable & Control-Theoretic Robotics (VECTR) Lab *
 * University of California, Los Angeles                   *
 *                                                         *
 * Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez   *
 * Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu         *
 *                                                         *
 ***********************************************************/

#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"

namespace dlio {

template <typename T>
struct identity {
  typedef T type;
};

template <typename T>
void declare_param(rclcpp::Node* node, const std::string param_name, T& param,
                   const typename identity<T>::type& default_value) {
  node->declare_parameter(param_name, default_value);
  node->get_parameter(param_name, param);
  RCLCPP_INFO_STREAM(node->get_logger(), param_name << " = " << param);
}

template <typename T>
void declare_param_vector(rclcpp::Node* node, const std::string param_name,
                          T& param,
                          const typename identity<T>::type& default_value) {
  node->declare_parameter(param_name, default_value);
  node->get_parameter(param_name, param);
  for (int i = 0; i < param.size(); i++) {
    RCLCPP_INFO_STREAM(node->get_logger(), param_name << "[" << i << "]"
                                                      << " = " << param[i]);
  }
}

/// Get the given subnode value.
/// The only reason this function exists is to wrap the exceptions in slightly
/// nicer error messages, including the name of the failed key
/// @throw YAML::Exception
template <typename T>
T yaml_get_value(const YAML::Node& node, const std::string& key) {
  try {
    return node[key].as<T>();
  } catch (YAML::Exception& e) {
    std::stringstream ss;
    ss << "Failed to parse YAML tag '" << key << "' for reason: " << e.msg;
    throw YAML::Exception(e.mark, ss.str());
  }
}

}  // namespace dlio
