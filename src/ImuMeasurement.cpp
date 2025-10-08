#include "allan_variance_ros2/ImuMeasurement.hpp"


ImuMeasurement::ImuMeasurement():
  I_a_WI(0,0,0),
  I_w_WI(0,0,0)
{
}

ImuMeasurement::ImuMeasurement(const uint64_t _t,
                               const Eigen::Vector3d& _I_a_WI,
                               const Eigen::Vector3d& _I_w_WI){
  t = _t;
  I_a_WI = _I_a_WI;
  I_w_WI = _I_w_WI;
}

std::ostream& operator<< (std::ostream& stream, const ImuMeasurement& meas) {
  stream << "IMU Measurement at time = " << meas.t << " : \n"
         << "I_a_WI: " << meas.I_a_WI.transpose() << "\n"
         << "I_w_WI: " << meas.I_w_WI.transpose() << "\n";
  return stream;
}