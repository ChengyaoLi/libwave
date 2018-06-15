#include <random>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include "wave/gtsam/lidar_gps_range.hpp"
#include "wave/wave_test.hpp"

namespace wave {

    TEST(range, init) {
        std::cout<< "test" <<std::endl;

        gtsam::Key Lidar_GPS = 1;
        gtsam::Pose3 lidar_gps_s1;
        Mat6 info;
        info.setIdentity();
        double dis = 10;
        auto model = gtsam::noiseModel::Gaussian::Information(info);
        RangeFactor factor1(Lidar_GPS, dis, model);
    }


// Test that the factor produces zero error
    TEST(range, zero_error) {
        gtsam::Key GPS_LIDAR = 1;

        Eigen::Affine3d gps_lidar;

        gps_lidar.matrix() << 0.014639292388417, 0.999713733025263, 0.018924669589204, -1.124956918212964,
                -0.999886807885517, 0.014570840151735, 0.003749937837334, 0.072660555301776,
                0.003473116018466, -0.018977423902277, 0.999813880103270, 1.25,
                0, 0, 0, 1;


        gtsam::Pose3 T_gps_lidar(gps_lidar.matrix());

        double range  = 1.683243186;
        Mat6 info;
        info.setIdentity();
        auto model = gtsam::noiseModel::Gaussian::Information(info);
        RangeFactor factor(GPS_LIDAR,range, model);

        auto err = factor.evaluateError(T_gps_lidar);
        EXPECT_NEAR(err.norm(), 0, 1e-6);
    }

// Test that the factor jacobians are good when error is small
    TEST(range, jacobians) {
            gtsam::Key GPS_LIDAR = 1;

            Eigen::Affine3d gps_lidar;
            Eigen::Affine3d Imatrix;


        gps_lidar.matrix() <<0.014639292388417, 0.999713733025263, 0.018924669589204, -1.124956918212964,
                -0.999886807885517, 0.014570840151735, 0.003749937837334, 0.072660555301776,
                0.003473116018466, -0.018977423902277, 0.999813880103270, -1.25,
                0, 0, 0, 1;


            gtsam::Pose3 T_gps_lidar(gps_lidar.matrix());


        double range  = 1.683243186;
            Mat6 info;
            info.setIdentity();
            auto model = gtsam::noiseModel::Gaussian::Information(info);
            RangeFactor factor(GPS_LIDAR,range, model);
            gtsam::Matrix J_T_LIDAR_GPS;
            auto err = factor.evaluateError(T_gps_lidar,J_T_LIDAR_GPS);

        auto fun = boost::bind(&RangeFactor::evaluateError,
                               boost::ref(factor),
                               _1,
                               boost::none);

        gtsam::Matrix J_LIDER_GPS_num = gtsam::numericalDerivative11<gtsam::Vector,
                gtsam::Pose3>(fun, T_gps_lidar,1e-6);

        EXPECT_NEAR(err.norm(), 0, 1e-6);
        EXPECT_NEAR((J_T_LIDAR_GPS - J_LIDER_GPS_num).norm(), 0, 1e-8);
        }

}  // namespace wave
