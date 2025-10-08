#include "allan_variance_ros2/yaml_parsers.hpp"
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/node/parse.h>

YAML::Node loadYamlFile(const std::string &filename) {
  if (filename.empty()) { throw std::invalid_argument("Filename is empty!"); }

  YAML::Node node;

  try {
    node = YAML::LoadFile(filename);
  } catch (...) {
    throw std::invalid_argument("Error reading config file: " + filename);
  }

  if (node.IsNull()) { throw std::invalid_argument("Error reading config file: " + filename); }

  RCLCPP_INFO(rclcpp::get_logger("yaml_parser"), "Successfully read config file: %s", filename.c_str());

  return node;
}