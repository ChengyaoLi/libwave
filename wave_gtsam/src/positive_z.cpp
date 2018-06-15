#include "wave/gtsam/positive_z.hpp"

namespace wave {

PositiveZFactor::PositiveZFactor(gtsam::Key LIDAR_GPS,
                             const gtsam::SharedNoiseModel &model)
    : NoiseModelFactor1(model, LIDAR_GPS) {}

gtsam::Vector PositiveZFactor::evaluateError(
        const gtsam::Pose3 &T_LIDAR_GPS,
        boost::optional<gtsam::Matrix &> J_Z) const {
    double Z = T_LIDAR_GPS.z();
    auto retval = (gtsam::Matrix11() <<fabs(Z)-Z).finished();

    if(J_Z) {
        if (Z<0) {
            double j_x, j_y,j_z;
            gtsam::Matrix4 posematrix = T_LIDAR_GPS.matrix();
            j_x = -2 *posematrix(2,0);
            j_y = -2 *posematrix(2,1);
            j_z = -2 *posematrix(2,2);
            (*J_Z) = (gtsam::Matrix16() << 0, 0, 0, j_x, j_y, j_z).finished();
        }
        else
            (*J_Z) = (gtsam::Matrix16() << 0, 0, 0, 0, 0, 0).finished();
    }
    return retval;
}
}
