/**
 * @file   AllanVarianceComputor.cpp
 * @brief  Implementation of the AllanVarianceComputor class for ROS2.
 * @author Russell Buchanan
 */

#include "allan_variance_ros2/AllanVarianceComputor.hpp"
#include <omp.h>
#include "rosbag2_cpp/reader.hpp"
#include "rclcpp/serialization.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace allan_variance_ros2 {

AllanVarianceComputor::AllanVarianceComputor(rclcpp::Node::SharedPtr node, std::string config_file, std::string output_path)
    : node_(node), firstMsg_(true), overlap_(0.0) {
  YAML::Node yaml_node = loadYamlFile(config_file);

  get(yaml_node, "imu_topic", input_topic_);
  RCLCPP_INFO_STREAM(node_->get_logger(), "imu_topic: " << input_topic_);
  get(yaml_node, "imu_rate", imu_rate_);
  RCLCPP_INFO_STREAM(node_->get_logger(), "imu_rate: " << imu_rate_);
  get(yaml_node, "measure_rate", measure_rate_);
  RCLCPP_INFO_STREAM(node_->get_logger(), "measure_rate: " << measure_rate_);
  get(yaml_node, "sequence_time", sequence_time_);
  RCLCPP_INFO_STREAM(node_->get_logger(), "sequence_time: " << sequence_time_);

  imu_skip_ = int(imu_rate_ / measure_rate_);
  imu_output_file_ = output_path + "/" + "allan_variance" + ".csv";
}

void AllanVarianceComputor::run(std::string bag_path) {
  RCLCPP_INFO_STREAM(node_->get_logger(), "Processing " << bag_path << " ...");
  av_output_ = std::ofstream(imu_output_file_.c_str(), std::ofstream::out);

  rosbag2_cpp::Reader reader;
  reader.open(bag_path);

  rclcpp::Serialization<sensor_msgs::msg::Imu> serialization;
  
  int imu_counter = 0;
  
  auto start = std::chrono::steady_clock::now();

  while (reader.has_next()) {
    rosbag2_storage::SerializedBagMessageSharedPtr msg = reader.read_next();

    if (msg->topic_name != input_topic_) {
      continue;
    }

    rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
    auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
    serialization.deserialize_message(&serialized_msg, imu_msg.get());
    
    tCurrNanoSeconds_ = rclcpp::Time(imu_msg->header.stamp).nanoseconds();

    imu_counter++;

    // Subsample
    if (imu_counter % imu_skip_ != 0 || (imu_counter / imu_rate_) > sequence_time_) {
      continue;
    }
    
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(now - start).count() >= 2.0) {
        RCLCPP_INFO_STREAM(node_->get_logger(), (imu_counter / imu_rate_) << " / " << sequence_time_ << " seconds loaded");
        start = now;
    }

    if (firstMsg_) {
      firstMsg_ = false;
      firstTime_ = tCurrNanoSeconds_;
      lastImuTime_ = tCurrNanoSeconds_;
    }

    if (tCurrNanoSeconds_ < lastImuTime_) {
      skipped_imu_++;
      RCLCPP_ERROR_STREAM(node_->get_logger(), "IMU out of order. Current(ns): "
                       << tCurrNanoSeconds_ - firstTime_ << " Last(ns): "
                       << lastImuTime_ - firstTime_ << " (" << skipped_imu_ << " dropped)");
      continue;
    }
    lastImuTime_ = tCurrNanoSeconds_;

    ImuMeasurement input;
    input.t = tCurrNanoSeconds_;
    input.I_a_WI = Eigen::Vector3d(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y,
                                   imu_msg->linear_acceleration.z);
    input.I_w_WI =
        Eigen::Vector3d(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);

    imuBuffer_.push_back(input);
    
    if (!rclcpp::ok()) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Stop requested, closing the bag!");
        break;
    }
  }

  RCLCPP_INFO_STREAM(node_->get_logger(), "Finished collecting data. " << imuBuffer_.size() << " measurements");

  if(!imuBuffer_.empty()) {
    allanVariance();
  } else {
    RCLCPP_ERROR(node_->get_logger(), "No IMU messages to process, is your topic right?");
  }
}

void AllanVarianceComputor::closeOutputs() { av_output_.close(); }

void AllanVarianceComputor::allanVariance() {
  std::mutex mtx;
  bool stop_early = false;
  std::map<int,std::vector<std::vector<double>>> averages_map;

  int period_min = 1;
  int period_max = 10000;

  #pragma omp parallel for
  for (int period = period_min; period < period_max; period++) {
    if (!rclcpp::ok() || stop_early) {
      stop_early = true;
      continue;
    }

    std::vector<std::vector<double>> averages;
    double period_time = period * 0.1;

    int max_bin_size = period_time * measure_rate_;
    if (max_bin_size == 0) continue;
    int overlap = floor(max_bin_size * overlap_);

    std::vector<double> current_average = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    for (size_t j = 0; j <= imuBuffer_.size() - max_bin_size; j += (max_bin_size - overlap)) {
      for (int m = 0; m < max_bin_size; m++) {
        current_average[0] += imuBuffer_[j + m].I_a_WI[0];
        current_average[1] += imuBuffer_[j + m].I_a_WI[1];
        current_average[2] += imuBuffer_[j + m].I_a_WI[2];
        current_average[3] += imuBuffer_[j + m].I_w_WI[0] * 180 / M_PI;
        current_average[4] += imuBuffer_[j + m].I_w_WI[1] * 180 / M_PI;
        current_average[5] += imuBuffer_[j + m].I_w_WI[2] * 180 / M_PI;
      }

      for(int i=0; i<6; ++i) current_average[i] /= max_bin_size;
      averages.push_back(current_average);
      current_average = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    }
    
    if (averages.empty()) continue;

    {
      std::lock_guard<std::mutex> lck(mtx);
      int num_averages = averages.size();
      RCLCPP_INFO_STREAM(node_->get_logger(), "Computed " << num_averages << " averages for period " << period_time
                      << " (" << (period_max - averages_map.size()) << " left)");
      averages_map.insert({period, averages});
    }
  }

  if(!rclcpp::ok() || stop_early) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Stop requested, stopping calculation!");
    return;
  }

  // Write header to CSV
  av_output_ << "period,accX,accY,accZ,gyroX,gyroY,gyroZ" << std::endl;

  for (int period = period_min; period < period_max; period++) {
    if (averages_map.find(period) == averages_map.end()) continue;

    std::vector<std::vector<double>> averages = averages_map.at(period);
    double period_time = period * 0.1;
    int num_averages = averages.size();
    
    if (num_averages < 2) continue;

    std::vector<double> allan_variance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    for (int k = 0; k < num_averages - 1; k++) {
      for(int i=0; i<6; ++i) allan_variance[i] += std::pow(averages[k + 1][i] - averages[k][i], 2);
    }

    std::vector<double> avar;
    for(int i=0; i<6; ++i) avar.push_back(allan_variance[i] / (2.0 * (num_averages - 1)));

    std::vector<double> allan_deviation;
    for(int i=0; i<6; ++i) allan_deviation.push_back(std::sqrt(avar[i]));

    writeAllanDeviation(allan_deviation, period_time);
  }
}

void AllanVarianceComputor::writeAllanDeviation(std::vector<double> deviation, double period) {
  aVRecorder_.period = period;
  aVRecorder_.accX = deviation[0];
  aVRecorder_.accY = deviation[1];
  aVRecorder_.accZ = deviation[2];
  aVRecorder_.gyroX = deviation[3];
  aVRecorder_.gyroY = deviation[4];
  aVRecorder_.gyroZ = deviation[5];
  aVRecorder_.writeOnFile(av_output_);
}

}  // namespace allan_variance_ros2
