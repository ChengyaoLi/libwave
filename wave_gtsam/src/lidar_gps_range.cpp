#include "wave/gtsam/lidar_gps_range.hpp"

namespace wave {

RangeFactor::RangeFactor(gtsam::Key LIDAR_GPS,
                             const double lidar_gps_range,
                             const gtsam::SharedNoiseModel &model)
    : NoiseModelFactor1(model, LIDAR_GPS),
      LIDAR_GPS_RANGE(lidar_gps_range) {}

gtsam::Vector RangeFactor::evaluateError(
  const gtsam::Pose3 &T_LIDAR_GPS,
  boost::optional<gtsam::Matrix &> J_T_LIDAR_GPS) const {
    
    auto retval = gtsam::Pose3::range(T_LIDAR_GPS)-LIDAR_GPS_RANGE;

    if (J_T_LIDAR_GPS) {
        J_T_LIDAR_GPS->resize(1, 6);
	J_T_LIDAR_GPS(0,0) = 0;
	J_T_LIDAR_GPS(0,1) = 0;
	J_T_LIDAR_GPS(0,2) = 0;
        *J_T_LIDAR_GPS = J_T_LIDAR_GPS;
    }

    return retval;
}
}
