/**
 * @file   allan_variance.cpp
 * @brief  Tool to compute Allan Variance and Deviation from ROS2 rosbag.
 * @author Russell Buchanan
 */

#include <boost/filesystem.hpp>
#include <ctime>
#include <set>
#include <memory>

// ROS2
#include "rclcpp/rclcpp.hpp"

// allan_variance_ros2
#include "allan_variance_ros2/AllanVarianceComputor.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("allan_variance_node");
  
  std::string bags_folder = ".";
  std::string config_file;

  if (argc >= 3) {
    bags_folder = argv[1];
    config_file = argv[2];
    RCLCPP_INFO_STREAM(node->get_logger(), "Bag Folder = " << bags_folder);
    RCLCPP_INFO_STREAM(node->get_logger(), "Config File = " << config_file);
  } else {
    RCLCPP_WARN(node->get_logger(), "Usage: ./allan_variance /path/to/bag_folder /path/to/config.yaml");
    return 1;
  }

  namespace fs = boost::filesystem;
  fs::path path = fs::absolute(fs::path(bags_folder));

  if (!fs::is_directory(path)) {
    RCLCPP_ERROR_STREAM(node->get_logger(), path.string() << " is not a directory!");
    return 1;
  }
  
  std::clock_t start = std::clock();

  // Note: ROS2 bags are directories. This code assumes we process one bag directory.
  // The original code iterated through .bag files. We will process the provided folder as a single bag.
  allan_variance_ros2::AllanVarianceComputor computor(node, config_file, bags_folder);
  RCLCPP_INFO(node->get_logger(), "Computor constructed");
  
  if (rclcpp::ok()) {
      RCLCPP_INFO_STREAM(node->get_logger(), "Processing rosbag folder " << bags_folder);
      computor.run(bags_folder);
  }

  double durationTime = (std::clock() - start) / (double)CLOCKS_PER_SEC;
  RCLCPP_INFO(node->get_logger(), "Total computation time: %f s", durationTime);
  RCLCPP_INFO(node->get_logger(), "Data written to allan_variance.csv");

  rclcpp::shutdown();
  return 0;
}
