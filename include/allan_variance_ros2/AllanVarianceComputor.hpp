#pragma once

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <mutex>
#include <fstream>
#include <iomanip>

// allan_variance_ros2
#include "allan_variance_ros2/ImuMeasurement.hpp"
#include "allan_variance_ros2/yaml_parsers.hpp"

namespace allan_variance_ros2 {

template <class T>
using EigenVector = std::vector<T, Eigen::aligned_allocator<T>>;

// Helper structs for output formats
struct AllanVarianceFormat {
  double period;
  double accX;
  double accY;
  double accZ;
  double gyroX;
  double gyroY;
  double gyroZ;

  void writeOnFile(std::ofstream& file) {
    file << std::fixed << std::setprecision(10) << period 
         << " " << accX << " " << accY << " " << accZ << " "
         << gyroX << " " << gyroY << " " << gyroZ << std::endl;
  }
};

class AllanVarianceComputor {
 public:
  AllanVarianceComputor(rclcpp::Node::SharedPtr node, std::string config_file, std::string output_path);

  virtual ~AllanVarianceComputor() {closeOutputs();}

  void run(std::string bag_path);
  void closeOutputs();
  void allanVariance();
  void writeAllanDeviation(std::vector<double> variance, double period);

 private:
  // ROS
  rclcpp::Node::SharedPtr node_;

  // Data
  AllanVarianceFormat aVRecorder_{};
  std::ofstream av_output_;
  std::string imu_output_file_;

  // Config
  int sequence_time_{};
  int measure_rate_{};
  std::string input_topic_;
  double imu_rate_ = 100.0;

  int skipped_imu_{};
  int imu_skip_;
  uint64_t tCurrNanoSeconds_{};
  uint64_t lastImuTime_{};
  uint64_t firstTime_{};
  EigenVector<ImuMeasurement> imuBuffer_;
  bool firstMsg_;
  float overlap_; // Percent to overlap bins
};
}  // namespace allan_variance_ros2
