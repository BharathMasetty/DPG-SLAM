#pragma once

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include "parameters.h"

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/PriorFactor.h>

namespace dpg_slam {

    class DpgSLAM {
    public:

        /**
         * Create the DPG SLAM object.
         *
         * @param dpg_parameters        DPG specific parameters.
         * @param pose_graph_parameters Pose graph parameters.
         */
        DpgSLAM(const DpgParameters &dpg_parameters, const PoseGraphParameters &pose_graph_parameters);

        // Observe a new laser scan.
        void ObserveLaser(const std::vector<float>& ranges,
                          float range_min,
                          float range_max,
                          float angle_min,
                          float angle_max);

        // Observe new odometry-reported location.
        void ObserveOdometry(const Eigen::Vector2f& odom_loc,
                             const float odom_angle);
// Not sure if we need these
//        // Get latest map.
//        std::vector<Eigen::Vector2f> GetMap();
//
//        // Get latest robot pose.
//        void GetPose(Eigen::Vector2f* loc, float* angle) const;
//
//        /**
//         * Add the currently estimated trajectory and the odometry-only estimate to the visualization.
//         *
//         * Odometry estimate may not start at the same pose.
//         *
//         * @param visualization_msg[out] Visualization message to add trajectory and odometry estimates to.
//         */
//        void publishTrajectory(amrl_msgs::VisualizationMsg &visualization_msg);

    private:

        /**
         * Determine if the robot has moved far enough that we should compare the last used laser scan to the current laser
         * scan.
         *
         * @return True if the robot has moved far enough that we should compare the last used laser scan to the current
         * laser scan, false if we should skip this laser scan.
         */
        bool shouldProcessLaser();

        /**
         * GTSAM factor graph.
         */
        gtsam::NonlinearFactorGraph* graph_;

        /**
         * Initial estimates for GTSAM.
         */
        gtsam::Values initialEstimates_;

        /**
         * DPG specific parameters.
         */
        DpgParameters dpg_parameters_;

        /**
         * Pose graph parameters.
         */
        PoseGraphParameters pose_graph_parameters_;
    };
}  // end dpg_slam
