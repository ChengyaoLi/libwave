#ifndef WAVE_LIDAR_GPS_RANGE_HPP
#define WAVE_LIDAR_GPS_RANGE_HPP

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>

namespace wave {

/**
 * This class is written to use measured transform from imu to gps as a prior in the map builders
 */

class RangeFactor
  : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
    const double LIDAR_GPS_RANGE;  // range measurement from CAD
 public:
    RangeFactor(gtsam::Key LIDAR_GPS,
		  const double LIDAR_GPS_RANGE,
                  const gtsam::SharedNoiseModel &model);

    gtsam::Vector evaluateError(
      const gtsam::Pose3 &T_LIDAR_GPS,
      boost::optional<gtsam::Matrix &> J_T_LIDAR_GPS = boost::none) const;
};
}

#endif  // WAVE_LIDAR_GPS_RANGE_HPP
