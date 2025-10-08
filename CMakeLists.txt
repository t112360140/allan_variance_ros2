cmake_minimum_required(VERSION 3.8)
project(allan_variance_ros2)

if(CMAKE_COMP-ILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP)


# AllanVarianceComputor Library
add_library(allan_variance_computor
  src/AllanVarianceComputor.cpp
  src/yaml_parsers.cpp
  src/ImuMeasurement.cpp)
target_include_directories(allan_variance_computor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(allan_variance_computor
  "rclcpp"
  "rosbag2_cpp"
  "sensor_msgs"
  "yaml_cpp_vendor")
target_link_libraries(allan_variance_computor
  Eigen3::Eigen)
if(OpenMP_FOUND)
  target_link_libraries(allan_variance_computor OpenMP::OpenMP_CXX)
endif()


# allan_variance executable
add_executable(allan_variance src/allan_variance.cpp)
ament_target_dependencies(allan_variance "rclcpp")
target_link_libraries(allan_variance
  allan_variance_computor
  Boost::filesystem)


# --- Temporarily disabling the imu_simulator ---
# add_executable(imu_simulator src/ImuSimulator.cpp)
# ament_target_dependencies(imu_simulator
#   "rclcpp"
#   "rosbag2_cpp"
#   "sensor_msgs"
#   "rosidl_default_runtime")
# target_link_libraries(imu_simulator
#   allan_variance_computor
#   sensor_msgs::sensor_msgs__cpp
# )


# Install rules
install(TARGETS
  allan_variance_computor
  allan_variance
  # --- Temporarily disabling the imu_simulator ---
  # imu_simulator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()