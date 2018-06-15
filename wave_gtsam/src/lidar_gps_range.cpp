#include "wave/gtsam/lidar_gps_range.hpp"

namespace wave {

RangeFactor::RangeFactor(gtsam::Key LIDAR_GPS,
                              double lidar_gps_range,
                             const gtsam::SharedNoiseModel &model)
    : NoiseModelFactor1(model, LIDAR_GPS),
      LIDAR_GPS_RANGE(lidar_gps_range) {}

gtsam::Vector RangeFactor::evaluateError(
        const gtsam::Pose3 &T_LIDAR_GPS,
        boost::optional<gtsam::Matrix &> J_T_LIDAR_GPS) const {

    Eigen::Affine3d origin;
    origin.matrix() <<   1, 0,0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1.000000000000000;
    gtsam::Pose3 T_origin(origin.matrix());
    auto retval = (gtsam::Matrix11() <<T_LIDAR_GPS.range(T_origin,J_T_LIDAR_GPS,boost::none)-LIDAR_GPS_RANGE).finished();

    return retval;
}
}
