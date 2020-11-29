//
// Created by amanda on 11/25/20.
//

#pragma once

namespace dpg_slam {

    /**
     * Class that contains all DPG parameters.
     */
    class DpgParameters {
    public:

        // TODO set the rest of the params, maybe make them loadable via rosparam server
        DpgParameters() : num_sectors_(kDefaultNumSectors){

        }

        /**
         * Default value for the number of sectors.
         */
        const uint8_t kDefaultNumSectors = 5;

        // TODO Should these be set via constructor or via rosparam retreiver

        // TODO note where these are referenced in the paper
        /**
         * Number of sectors per node.
         */
        uint8_t num_sectors_;

        /**
         * Number of nodes that should be in the current pose chain.
         */
        uint8_t current_pose_chain_len_;

        /**
         * Number of bins used for change detection.
         */
        uint16_t num_bins_for_change_detection_;

        /**
         * Ratio of changed bins to total bins for a node to be considered changed.
         */
        double delta_change_threshold_;

        /**
         * How much of the current pose graph must be covered by FOVs of the the local submap nodes.
         */
        double current_pose_graph_coverage_threshold_;

        /**
         * Occupancy grid resolution when computing the submap coverage.
         */
        double occ_grid_resolution_;
    };

    /**
     * Class that contains parameters related to creating the pose graph (not DPG related).
     */
    class PoseGraphParameters {
    public:

        // TODO need to make laser offset configurable based on robot
        // May also want to make some of these loadable from rosparam server
        PoseGraphParameters() : icp_maximum_iterations_(kDefaultIcpMaximumIterations),
        icp_maximum_transformation_epsilon_(kDefaultIcpMaximumTransformationEpsilon),
        icp_max_correspondence_distance_(kDefaultIcpMaxCorrespondenceDistance),
        maximum_node_dist_scan_comparison_(kDefaultMaximumNodeDistScanComparison),
        gtsam_max_iterations_(kDefaultGtsamMaxIterations), min_dist_between_nodes_(kDefaultMinDistBetweenNodes),
        min_angle_between_nodes_(kDefaultMinAngleBetweenNodes), new_pass_x_std_dev_(kDefaultNewPassXStdDev),
        new_pass_y_std_dev_(kDefaultNewPassYStdDev), new_pass_theta_std_dev_(kDefaultNewPassThetaStdDev),
        motion_model_transl_error_from_transl_(kDefaultMotionModelTranslErrorFromTransl),
        motion_model_transl_error_from_rot_(kDefaultMotionModelTranslErrorFromRot),
        motion_model_rot_error_from_transl_(kDefaultMotionModelRotErrorFromTransl),
        motion_model_rot_error_from_rot_(kDefaultMotionModelRotErrorFromRot),
        laser_x_in_bl_frame_(kDefaultLaserXInBLFrame), laser_y_in_bl_frame_(kDefaultLaserYInBLFrame),
        laser_orientation_rel_bl_frame_(kDefaultLaserOrientationRelBLFrame) {}

        /**
         * Default maximum number of iterations to run ICP for a single transform estimate.
         */
        const int kDefaultIcpMaximumIterations = 500; // TODO tune

        /**
         * Default maximum allowable translation squared difference between transformation estimates for ICP to be
         * considered converged.
         *
         * See https://pointclouds.org/documentation/classpcl_1_1_registration.html#aec74ab878cca8d62fd1be9942685a8c1.
         */
        const double kDefaultIcpMaximumTransformationEpsilon = 0.00005; // TODO tune

        // TODO should we include this?
        /**
         * Default maximum distance threshold between two correspondent points for ICP to consider them in alignment.
         *
         * https://pointclouds.org/documentation/classpcl_1_1_registration.html#a65596dcc3cb5d2647857226fb3d999a5
         */
        const double kDefaultIcpMaxCorrespondenceDistance = 4.0; // TODO tune

        // TODO want to have any of the following for ICP
        // ICP RANSAC outlier rejection threshold?
        // euclidean fitness epsilon?

        /**
         * Default maximum amount that two nodes can be separated by to try to align their scans.
         */
        const float kDefaultMaximumNodeDistScanComparison = 3.0; // TODO tune

        /**
         * Default maximum number of iterations for one run of GTSAM estimation.
         */
        const float kDefaultGtsamMaxIterations = 100; // TODO tune

        /**
         * Default minimum distance between two consecutive nodes. If the robot's odometry has not estimated a distance
         * greater than this since the last considered laser scan, we should not add a new node to the pose graph
         * (unless the rotation threshold below has been exceeded).
         */
        const float kDefaultMinDistBetweenNodes = 0.5; // TODO tune

        /**
         * Default minimum angle between two consecutive nodes. If the robot's odometry has not estimated an orientation
         * change greater than this since the last considered laser scan, we should not add a new node to the pose graph
         * (unless the translation threshold above has been exceeded).
         */
        const float kDefaultMinAngleBetweenNodes = M_PI_4; // TODO tune

        /**
         * Default standard deviation for the x component of the prior put on the first node in a pass.
         */
        const float kDefaultNewPassXStdDev = 0.2; // TODO tune

        /**
         * Default standard deviation for the y component of the prior put on the first node in a pass.
         */
        const float kDefaultNewPassYStdDev = 0.2; // TODO tune

        /**
         * Default standard deviation for the theta component of the prior put on the first node in a pass.
         */
        const float kDefaultNewPassThetaStdDev = 0.15; // TODO tune

        /**
         * Default multiplier for translational error from translation (used in odometry constraints).
         */
        const float kDefaultMotionModelTranslErrorFromTransl = 0.05; // TODO tune

        /**
         * Default multiplier for translational error from rotation (used in odometry constraints).
         */
        const float kDefaultMotionModelTranslErrorFromRot = 0.05; // TODO tune

        /**
         * Default multiplier for rotational error from translation (used in odometry constraints).
         */
        const float kDefaultMotionModelRotErrorFromTransl = 0.05; // TODO tune

        /**
         * Default multiplier for rotational error from rotation (used in odometry constraints).
         */
        const float kDefaultMotionModelRotErrorFromRot = 0.05; // TODO tune

        /**
         * Default x coordinate of the lidar in the base_link frame.
         */
        const float kDefaultLaserXInBLFrame = 0.2;

        /**
         * Default y coordinate of the lidar in the base_link frame.
         */
        const float kDefaultLaserYInBLFrame = 0.0;

        /**
         * Default orientation of the lidar relative to the base link frame.
         */
        const float kDefaultLaserOrientationRelBLFrame = 0.0;

        /**
         * Maximum number of iterations to run ICP for a single transform estimate.
         */
        int icp_maximum_iterations_;

        /**
         * Maximum allowable translation squared difference between transformation estimates for ICP to be considered
         * converged.
         *
         * See https://pointclouds.org/documentation/classpcl_1_1_registration.html#aec74ab878cca8d62fd1be9942685a8c1.
         */
        double icp_maximum_transformation_epsilon_;

        // TODO should we include this?
        /**
         * Maximum distance threshold between two correspondent points for ICP to consider them in alignment.
         *
         * https://pointclouds.org/documentation/classpcl_1_1_registration.html#a65596dcc3cb5d2647857226fb3d999a5
         */
        double icp_max_correspondence_distance_;

        // TODO want to have any of the following for ICP
        // ICP RANSAC outlier rejection threshold?
        // euclidean fitness epsilon?

        /**
         * Maximum amount that two nodes can be separated by to try to align their scans.
         */
        float maximum_node_dist_scan_comparison_;

        /**
         * Maximum number of iterations for one run of GTSAM estimation.
         */
        float gtsam_max_iterations_;

        /**
         * Minimum distance between two consecutive nodes. If the robot's odometry has not estimated a distance
         * greater than this since the last considered laser scan, we should not add a new node to the pose graph
         * (unless the rotation threshold below has been exceeded).
         */
        float min_dist_between_nodes_;

        /**
         * Minimum angle between two consecutive nodes. If the robot's odometry has not estimated an orientation change
         * greater than this since the last considered laser scan, we should not add a new node to the pose graph
         * (unless the translation threshold above has been exceeded).
         */
        float min_angle_between_nodes_;

        /**
         * Standard deviation for the x component of the prior put on the first node in a pass.
         */
        float new_pass_x_std_dev_;

        /**
         * Standard deviation for the y component of the prior put on the first node in a pass.
         */
        float new_pass_y_std_dev_;

        /**
         * Standard deviation for the theta component of the prior put on the first node in a pass.
         */
        float new_pass_theta_std_dev_;

        /**
         * Multiplier for translational error from translation (used in odometry constraints).
         */
        float motion_model_transl_error_from_transl_;

        /**
         * Multiplier for translational error from rotation (used in odometry constraints).
         */
        float motion_model_transl_error_from_rot_;

        /**
         * Multiplier for rotational error from translation (used in odometry constraints).
         */
        float motion_model_rot_error_from_transl_;

        /**
         * Multiplier for rotational error from rotation (used in odometry constraints).
         */
        float motion_model_rot_error_from_rot_;

        /**
         * X coordinate of the lidar in the base_link frame.
         */
        float laser_x_in_bl_frame_;

        /**
         * Y coordinate of the lidar in the base_link frame.
         */
        float laser_y_in_bl_frame_;

        /**
         * Orientation of the lidar relative to the base link frame.
         */
        float laser_orientation_rel_bl_frame_;

    };
}
