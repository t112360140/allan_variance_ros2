/**
 * @file   ImuSimulator.cpp
 * @brief  Tool to simulate imu data for ROS2.
 * @author Rick Liu
 */

// std, eigen and boost
#include <boost/filesystem.hpp>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <ctime>
#include <fstream>
#include <memory>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "rclcpp/serialization.hpp"

#include "allan_variance_ros2/yaml_parsers.hpp"

using Vec3d = Eigen::Vector3d;

Vec3d RandomNormalDistributionVector(double sigma) {
  static boost::mt19937 rng(time(0)); // Seed with time
  static boost::normal_distribution<> nd(0, 1);
  return {sigma * nd(rng), sigma * nd(rng), sigma * nd(rng)};
}

template <typename S, typename T> void FillROSVector3d(const S &from, T &to) {
  to.x = from.x();
  to.y = from.y();
  to.z = from.z();
}

class ImuSimulator : public rclcpp::Node {
public:
  ImuSimulator(std::string config_file, std::string output_path)
      : Node("imu_simulator") {
    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    writer_->open(output_path);

    auto yaml_config = loadYamlFile(config_file);

    get(yaml_config, "accelerometer_noise_density", accelerometer_noise_density_);
    get(yaml_config, "accelerometer_random_walk", accelerometer_random_walk_);
    get(yaml_config, "accelerometer_bias_init", accelerometer_bias_init_);

    get(yaml_config, "gyroscope_noise_density", gyroscope_noise_density_);
    get(yaml_config, "gyroscope_random_walk", gyroscope_random_walk_);
    get(yaml_config, "gyroscope_bias_init", gyroscope_bias_init_);

    get(yaml_config, "rostopic", rostopic_);
    RCLCPP_INFO_STREAM(this->get_logger(), "rostopic: " << rostopic_);
    get(yaml_config, "update_rate", update_rate_);
    RCLCPP_INFO_STREAM(this->get_logger(), "update_rate: " << update_rate_);
    get(yaml_config, "sequence_time", sequence_time_);
    RCLCPP_INFO_STREAM(this->get_logger(), "sequence_time: " << sequence_time_);
  }

  virtual ~ImuSimulator() = default;

  void run() {
    RCLCPP_INFO(this->get_logger(), "Generating IMU data ...");

    double dt = 1.0 / update_rate_;
    rclcpp::Time start_time(1, 0, RCL_ROS_TIME); // Start at 1 second
    Vec3d accelerometer_bias = Vec3d::Constant(accelerometer_bias_init_);
    Vec3d gyroscope_bias = Vec3d::Constant(gyroscope_bias_init_);
    Vec3d accelerometer_real = Vec3d::Zero();
    Vec3d gyroscope_real = Vec3d::Zero();

    // Create a topic in the bag
    writer_->create_topic({rostopic_, "sensor_msgs/msg/Imu", rmw_get_serialization_format(), ""});

    for (int64_t i = 0; i < sequence_time_ * update_rate_; ++i) {
      if (!rclcpp::ok()) {
        break;
      }

      // Bias walk
      accelerometer_bias += RandomNormalDistributionVector(accelerometer_random_walk_) * sqrt(dt);
      gyroscope_bias += RandomNormalDistributionVector(gyroscope_random_walk_) * sqrt(dt);

      // Measurement
      Vec3d acc_measure = accelerometer_real + accelerometer_bias + RandomNormalDistributionVector(accelerometer_noise_density_) / sqrt(dt);
      Vec3d gyro_measure = gyroscope_real + gyroscope_bias + RandomNormalDistributionVector(gyroscope_noise_density_) / sqrt(dt);

      auto msg = std::make_shared<sensor_msgs::msg::Imu>();
      rclcpp::Time current_time = start_time + rclcpp::Duration::from_seconds(i / update_rate_);
      msg->header.stamp = current_time;
      msg->header.frame_id = "imu_link"; // Good practice to set frame_id
      FillROSVector3d(acc_measure, msg->linear_acceleration);
      FillROSVector3d(gyro_measure, msg->angular_velocity);
      
      writer_->write(msg, rostopic_, current_time);
    }

    RCLCPP_INFO(this->get_logger(), "Finished generating data.");
  }

private:
  std::unique_ptr<rosbag2_cpp::Writer> writer_;

  double accelerometer_noise_density_;
  double accelerometer_random_walk_;
  double accelerometer_bias_init_;

  double gyroscope_noise_density_;
  double gyroscope_random_walk_;
  double gyroscope_bias_init_;

  std::string rostopic_;
  double update_rate_;
  double sequence_time_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  if (argc < 3) {
    RCLCPP_WARN(rclcpp::get_logger("main"), "Usage: ./imu_simulator /path/to/output/bag_folder /path/to/simulation/config_filename");
    return 1;
  }
  
  std::string rosbag_foldername = argv[1];
  std::string config_file = argv[2];
  RCLCPP_INFO(rclcpp::get_logger("main"), "Bag folder = %s", rosbag_foldername.c_str());
  RCLCPP_INFO(rclcpp::get_logger("main"), "Config File = %s", config_file.c_str());

  auto start = std::clock();
  auto simulator = std::make_shared<ImuSimulator>(config_file, rosbag_foldername);
  RCLCPP_INFO(simulator->get_logger(), "IMU simulator constructed");
  simulator->run();

  double durationTime = (std::clock() - start) / (double)CLOCKS_PER_SEC;
  RCLCPP_INFO(rclcpp::get_logger("main"), "Total computation time: %f s", durationTime);
  
  rclcpp::shutdown();
  return 0;
}
