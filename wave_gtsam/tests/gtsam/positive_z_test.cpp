#include <random>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include "wave/gtsam/positive_z.hpp"
#include "wave/wave_test.hpp"

namespace wave {

    TEST(positive_z, init) {
        gtsam::Key Lidar_GPS = 1;
        Mat6 info;
        info.setIdentity();
        auto model = gtsam::noiseModel::Gaussian::Information(info);
        PositiveZFactor factor(Lidar_GPS, model);
    }
// Test that the factor produces zero error
    TEST(positive_z, zero_error) {

        gtsam::Key GPS_LIDAR = 1;

        Eigen::Affine3d gps_lidar;

        gps_lidar.matrix() << 0.014639292388417, 0.999713733025263, 0.018924669589204, -1.124956918212964,
                -0.999886807885517, 0.014570840151735, 0.003749937837334, 0.072660555301776,
                0.003473116018466, -0.018977423902277, 0.999813880103270, 1.25,
                0, 0, 0, 1;

        gtsam::Pose3 T_gps_lidar(gps_lidar.matrix());
        Mat6 info;
        info.setIdentity();
        auto model = gtsam::noiseModel::Gaussian::Information(info);
        PositiveZFactor factor(GPS_LIDAR, model);

        auto err = factor.evaluateError(T_gps_lidar);

        EXPECT_NEAR(err.norm(),0, 1e-6);
    }

// Test that the factor jacobians are good when error is small - negative case
    TEST(positive_z, jacobians) {

        gtsam::Key GPS_LIDAR = 1;

        Eigen::Affine3d gps_lidar;

        gps_lidar.matrix() << 0.014639292388417, 0.999713733025263, 0.018924669589204, -1.124956918212964,
                -0.999886807885517, 0.014570840151735, 0.003749937837334, 0.072660555301776,
                0.003473116018466, -0.018977423902277, 0.999813880103270, 1.25,
                0, 0, 0, 1;

        gtsam::Pose3 T_gps_lidar(gps_lidar.matrix());
        Mat6 info;
        info.setIdentity();
        auto model = gtsam::noiseModel::Gaussian::Information(info);
        PositiveZFactor factor(GPS_LIDAR, model);
        gtsam::Matrix J_Z;

        auto err = factor.evaluateError(T_gps_lidar,J_Z);

        auto fun = boost::bind(&PositiveZFactor::evaluateError,
                               boost::ref(factor),
                               _1,
                               boost::none);

        gtsam::Matrix J_Z_num = gtsam::numericalDerivative11<gtsam::Vector,
                gtsam::Pose3>(fun, T_gps_lidar,1e-6);
        EXPECT_NEAR(err.norm(), 0, 1e-6);
        EXPECT_NEAR((J_Z - J_Z_num).norm(), 0, 1e-8);
    }

// Test that the factor jacobians are good when error is small - positive case
    TEST(positive_z, jacobians2) {


        gtsam::Key GPS_LIDAR = 1;

        Eigen::Affine3d gps_lidar;

        gps_lidar.matrix() <<  0.1349981, -0.6749198,  0.7254369,0,
                              -0.8918760,  0.2361870,  0.3857109,0,
                              -0.4316627, -0.6990700, -0.5700600, -1.25,
                                  0, 0, 0, 1;

        gtsam::Pose3 T_gps_lidar(gps_lidar.matrix());
        Mat6 info;
        info.setIdentity();
        auto model = gtsam::noiseModel::Gaussian::Information(info);
        PositiveZFactor factor(GPS_LIDAR, model);
        gtsam::Matrix J_Z;

        auto err = factor.evaluateError(T_gps_lidar,J_Z);

        auto fun = boost::bind(&PositiveZFactor::evaluateError,
                               boost::ref(factor),
                               _1,
                               boost::none);

        gtsam::Matrix J_Z_num = gtsam::numericalDerivative11<gtsam::Vector,
                gtsam::Pose3>(fun, T_gps_lidar,1e-6);
        EXPECT_NEAR(err.norm(), 2.5, 1e-6);
        EXPECT_NEAR((J_Z - J_Z_num).norm(), 0, 1e-8);
    }

}  // namespace wave
