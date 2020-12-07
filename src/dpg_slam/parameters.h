//
// Created by amanda on 11/25/20.
//

#pragma once

#include <ros/ros.h>

namespace dpg_slam {

    /**
     * Parameters related to visualization.
     */
    struct VisualizationParams {
        VisualizationParams() : display_points_fraction_(kDefaultDisplayPointsFraction) {

        }

        /**
         * Default value for the divisor for the fraction of points we should send when displaying the map.
         */
        const int kDefaultDisplayPointsFraction = 10;

        /**
         * When displaying the map, we should display 1 out of every this many points.
         */
        int display_points_fraction_;
    };

    /**
     * Class that contains all DPG parameters.
     */
    class DpgParameters {
    public:

        // TODO set the rest of the params, maybe make them loadable via rosparam server
        DpgParameters(ros::NodeHandle &node_handle) : num_sectors_(kDefaultNumSectors) {
            node_handle.param(kDeltaChangeThresholdParamName, delta_change_threshold_, kDefaultDeltaChangeThreshold);
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
        uint8_t current_pose_chain_len_ =  5;

	/**
         * Number of bins used for change detection.
         */
        uint16_t num_bins_for_change_detection_ = 10;

        /**
         * ROS param name for the ratio of changed bins to total bins for a node to be considered changed.
         */
        static constexpr const char *kDeltaChangeThresholdParamName = "delta_change_threshold";

        /**
         * Default value for the ratio of changed bins to total bins for a node to be considered changed.
         */
        const double kDefaultDeltaChangeThreshold = 0.1;

        /**
         * Ratio of changed bins to total bins for a node to be considered changed.
         */
        double delta_change_threshold_;

        /**
         * How much of the current pose graph must be covered by FOVs of the the local submap nodes.
         */
        double current_pose_graph_coverage_threshold_ = 0.8;

        /**
         * Occupancy grid resolution when computing the submap coverage.
         */
        double occ_grid_resolution_ = 0.10;

        /**
         * Minimum percent of sectors that have to be active for a node to be active.
         */
        float minimum_percent_active_sectors_ = 0.80;

	/**
	 * Proximity from nodes in current pose chain to search for submap nodes.
	 */
	float distance_threshold_for_local_submap_nodes_ = 5.0;
    };

    /**
     * Class that contains parameters related to creating the pose graph (not DPG related).
     */
    class PoseGraphParameters {
    public:

        // TODO need to make laser offset configurable based on robot
        // May also want to make some of these loadable from rosparam server

        /**
         * Constructor.
         *
         * @param node_handle Node handle for dynamically loading parameters from the ROS param server (if that is
         * configured for the param).
         */
        PoseGraphParameters(ros::NodeHandle &node_handle) : icp_maximum_iterations_(kDefaultIcpMaximumIterations),
        icp_maximum_transformation_epsilon_(kDefaultIcpMaximumTransformationEpsilon),
        icp_max_correspondence_distance_(kDefaultIcpMaxCorrespondenceDistance),
        gtsam_max_iterations_(kDefaultGtsamMaxIterations), min_dist_between_nodes_(kDefaultMinDistBetweenNodes),
        min_angle_between_nodes_(kDefaultMinAngleBetweenNodes), new_pass_x_std_dev_(kDefaultNewPassXStdDev),
        new_pass_y_std_dev_(kDefaultNewPassYStdDev), new_pass_theta_std_dev_(kDefaultNewPassThetaStdDev),
        motion_model_transl_error_from_transl_(kDefaultMotionModelTranslErrorFromTransl),
        motion_model_transl_error_from_rot_(kDefaultMotionModelTranslErrorFromRot),
        motion_model_rot_error_from_transl_(kDefaultMotionModelRotErrorFromTransl),
        motion_model_rot_error_from_rot_(kDefaultMotionModelRotErrorFromRot),
        laser_x_in_bl_frame_(kDefaultLaserXInBLFrame), laser_y_in_bl_frame_(kDefaultLaserYInBLFrame),
        laser_orientation_rel_bl_frame_(kDefaultLaserOrientationRelBLFrame),
        max_factors_per_node_(kDefaultMaxObsFactorsPerNode),
        factor_evaluation_skip_number_(kDefaultFactorSkipCount) {
            node_handle.param(kIcpMaximumIterationsParamName, icp_maximum_iterations_, kDefaultIcpMaximumIterations);
            node_handle.param(kIcpMaximumTransformationEpsilonParamName, icp_maximum_transformation_epsilon_, kDefaultIcpMaximumTransformationEpsilon);
            node_handle.param(kIcpMaxCorrespondenceDistanceParamName, icp_max_correspondence_distance_, kDefaultIcpMaxCorrespondenceDistance);
            node_handle.param(kRansacIterationsParamName, ransac_iterations_, kDefaultRansacIterations);
            node_handle.param(kIcpUseReciprocalCorrespondences, icp_use_reciprocal_correspondences_, kDefaultIcpUseReciprocalCorrespondences);
            node_handle.param(kMaximumNodeDistWithinPassScanComparisonParamName, maximum_node_dist_within_pass_scan_comparison_,
                              kDefaultMaximumNodeDistWithinPassScanComparison);
            node_handle.param(kMaximumNodeDistAcrossPassesScanComparisonParamName, maximum_node_dist_across_passes_scan_comparison_,
                              kDefaultMaximumNodeDistAcrossPassesScanComparison);
            node_handle.param(kMinDistBetweenNodesParamName, min_dist_between_nodes_, kDefaultMinDistBetweenNodes);
            node_handle.param(kMinAngleBetweenNodesParamName, min_angle_between_nodes_, kDefaultMinAngleBetweenNodes);
            node_handle.param(kNonSuccessiveScanConstraintsParamName, non_successive_scan_constraints_, kDefaultNonsuccessiveScanConstraints);
            node_handle.param(kOdometryConstraintsParamName, odometry_constraints_, kDefaultOdometryConstraintsParamName);
            node_handle.param(kLaserXVarianceParamName, laser_x_variance_, kDefaultLaserXVariance);
            node_handle.param(kLaserYVarianceParamName, laser_y_variance_, kDefaultLaserYVariance);
            node_handle.param(kLaserThetaVarianceParamName, laser_theta_variance_, kDefaultLaserThetaVariance);
            node_handle.param(kMotionModelTranslErrorFromTranslParamName, motion_model_transl_error_from_transl_, kDefaultMotionModelTranslErrorFromTransl);
            node_handle.param(kMotionModelTranslErrorFromRotParamName, motion_model_transl_error_from_rot_, kDefaultMotionModelTranslErrorFromRot);
            node_handle.param(kMotionModelRotErrorFromTranslParamName, motion_model_rot_error_from_transl_, kDefaultMotionModelRotErrorFromTransl);
            node_handle.param(kMotionModelRotErrorFromRotParamName, motion_model_rot_error_from_rot_, kDefaultMotionModelRotErrorFromRot);
            node_handle.param(kLaserXInBLFrameParamName, laser_x_in_bl_frame_, kDefaultLaserXInBLFrame);
            node_handle.param(kLaserYInBLFrameParamName, laser_y_in_bl_frame_, kDefaultLaserYInBLFrame);
            node_handle.param(kLaserOrientationInBLFrameParamName, laser_orientation_rel_bl_frame_, kDefaultLaserOrientationRelBLFrame);
            node_handle.param(kDownsampleIcpPointsRatioParamName, downsample_icp_points_ratio_, kDefaultDownsampleIcpPointsRatio);
            node_handle.param(kMaxObsFactorsPerNodeParamName, max_factors_per_node_, kDefaultMaxObsFactorsPerNode);
            node_handle.param(kNewPassXStdDevParamName, new_pass_x_std_dev_, kDefaultNewPassXStdDev);
            node_handle.param(kNewPassYStdDevParamName, new_pass_y_std_dev_, kDefaultNewPassYStdDev);
            node_handle.param(kNewPassThetaStdDevParamName, new_pass_theta_std_dev_, kDefaultNewPassThetaStdDev);
        }

        /**
         * Default maximum number of iterations to run ICP for a single transform estimate.
         */
        const int kDefaultIcpMaximumIterations = 500; // TODO tune

        /**
         * ROS Param Name for the maximum number of iterations to run ICP for a single transform estimate.
         */
        static constexpr const char *kIcpMaximumIterationsParamName = "icp_maximum_iterations";

        /**
         * Default maximum allowable translation squared difference between transformation estimates for ICP to be
         * considered converged.
         *
         * See https://pointclouds.org/documentation/classpcl_1_1_registration.html#aec74ab878cca8d62fd1be9942685a8c1.
         */
        const double kDefaultIcpMaximumTransformationEpsilon = 0.000000005; // TODO tune

        /**
         * ROS Param Name for the maximum allowable translation squared difference between transformation estimates for
         * ICP to be considered converged.
         */
        static constexpr const char *kIcpMaximumTransformationEpsilonParamName = "icp_maximum_transformation_epsilon_param_name";

        // TODO should we include this?
        /**
         * Default maximum distance threshold between two correspondent points for ICP to consider them in alignment.
         *
         * https://pointclouds.org/documentation/classpcl_1_1_registration.html#a65596dcc3cb5d2647857226fb3d999a5
         */
        const double kDefaultIcpMaxCorrespondenceDistance = 0.6; // TODO tune
        // ^ 0.7 also works reasonably well.

        /**
         * ROS Param Name for the maximum distance threshold between two correspondent points for ICP to consider them
         * in alignment.
         */
        static constexpr const char* kIcpMaxCorrespondenceDistanceParamName = "icp_max_correspondence_distance";

        // TODO want to have any of the following for ICP
        // ICP RANSAC outlier rejection threshold?
        // euclidean fitness epsilon?

        /**
         * Default number of iterations to run RANSAC during ICP.
         *
         * Note: Haven't really experimented with this much.
         */
        const int kDefaultRansacIterations = 50;

        /**
         * ROS Param Name for the number of iterations to run RANSAC during ICP.
         */
        static constexpr const char *kRansacIterationsParamName = "ransac_iterations";

        /**
         * Default configuration for using reciprocal correspondences in ICP.
         */
        const bool kDefaultIcpUseReciprocalCorrespondences = true;

        /**
         * ROS Param Name for the configuration for using reciprocal correspondences in ICP.
         */
        static constexpr const char *kIcpUseReciprocalCorrespondences = "icp_use_reciprocal_correspondences";

        /**
         * Default maximum amount that two nodes can be separated by to try to align their scans if they are in the same
         * pass.
         */
        const float kDefaultMaximumNodeDistWithinPassScanComparison = 5.0; // TODO tune

        /**
         * ROS Param Name for the maximum amount that two nodes can be separated by to try to align their scans if they are in the same
         * pass.
         */
        static constexpr const char *kMaximumNodeDistWithinPassScanComparisonParamName = "maximum_node_dist_within_pass_scan_comparison";

        /**
         * Default maximum amount that two nodes can be separated by to try to align their scans if they
         * are in different passes.
         */
        const float kDefaultMaximumNodeDistAcrossPassesScanComparison = 2.0; // TODO tune

        /**
         * ROS Param Name for the maximum amount that two nodes can be separated by to try to align their scans if they
         * are in different passes.
         */
        static constexpr const char *kMaximumNodeDistAcrossPassesScanComparisonParamName = "maximum_node_dist_across_passes_scan_comparison";

        /**
         * Default maximum number of iterations for one run of GTSAM estimation.
         */
        const float kDefaultGtsamMaxIterations = 100; // TODO tune

        /**
         * Default minimum distance between two consecutive nodes. If the robot's odometry has not estimated a distance
         * greater than this since the last considered laser scan, we should not add a new node to the pose graph
         * (unless the rotation threshold below has been exceeded).
         */
        const float kDefaultMinDistBetweenNodes = 1.0; // TODO tune

        /**
         * ROS Param Name for the minimum distance between two consecutive nodes.
         */
        static constexpr const char *kMinDistBetweenNodesParamName = "min_distance_between_nodes";

        /**
         * Default minimum angle between two consecutive nodes. If the robot's odometry has not estimated an orientation
         * change greater than this since the last considered laser scan, we should not add a new node to the pose graph
         * (unless the translation threshold above has been exceeded).
         */
        const float kDefaultMinAngleBetweenNodes = M_PI / 6.0; // TODO tune

        /**
         * ROS Param Name for the minimum angle between two consecutive nodes.
         */
        static constexpr const char *kMinAngleBetweenNodesParamName = "min_angle_between_nodes";

        /**
         * Default standard deviation for the x component of the prior put on the first node in a pass.
         */
        const float kDefaultNewPassXStdDev = 0.2; // TODO tune

        /**
         * ROS param name for the standard deviation for the x component of the prior put on the first node in a pass.
         */
        static constexpr const char *kNewPassXStdDevParamName = "new_pass_x_std_dev";

        /**
         * Default standard deviation for the y component of the prior put on the first node in a pass.
         */
        const float kDefaultNewPassYStdDev = 0.2; // TODO tune

        /**
         * ROS param name for the standard deviation for the y component of the prior put on the first node in a pass.
         */
        static constexpr const char *kNewPassYStdDevParamName = "new_pass_y_std_dev";

        /**
         * Default standard deviation for the theta component of the prior put on the first node in a pass.
         */
        const float kDefaultNewPassThetaStdDev = 0.15; // TODO tune

        /**
         * ROS param name for the standard deviation for the theta component of the prior put on the first node in a pass.
         */
        static constexpr const char *kNewPassThetaStdDevParamName = "new_pass_theta_std_dev";

        /**
         * Default multiplier for translational error from translation (used in odometry constraints).
         */
        const float kDefaultMotionModelTranslErrorFromTransl = 0.4; // TODO tune

        /**
         * ROS param name for the multiplier for translational error from translation (used in odometry constraints).
         */
        static constexpr const char *kMotionModelTranslErrorFromTranslParamName = "motion_transl_from_transl";

        /**
         * Default multiplier for translational error from rotation (used in odometry constraints).
         */
        const float kDefaultMotionModelTranslErrorFromRot = 0.4; // TODO tune

        /**
         * ROS param name for the multiplier for translational error from rotation (used in odometry constraints).
         */
        static constexpr const char *kMotionModelTranslErrorFromRotParamName = "motion_transl_from_rot";

        /**
         * Default multiplier for rotational error from translation (used in odometry constraints).
         */
        const float kDefaultMotionModelRotErrorFromTransl = 0.4; // TODO tune

        /**
         * ROS param name for the multiplier for rotational error from translation (used in odometry constraints).
         */
        static constexpr const char *kMotionModelRotErrorFromTranslParamName = "motion_rot_from_transl";

        /**
         * Default multiplier for rotational error from rotation (used in odometry constraints).
         */
        const float kDefaultMotionModelRotErrorFromRot = 0.4; // TODO tune

        /**
         * ROS param name for the multiplier for rotational error from rotation (used in odometry constraints).
         */
        static constexpr const char *kMotionModelRotErrorFromRotParamName = "motion_rot_from_rot";

        /**
         * Default x coordinate of the lidar in the base_link frame.
         */
        const float kDefaultLaserXInBLFrame = 0.2;

        /**
         * ROS param name for the x coordinate of the lidar in the base_link frame.
         */
        static constexpr const char *kLaserXInBLFrameParamName = "laser_x_in_bl_frame";

        /**
         * Default y coordinate of the lidar in the base_link frame.
         */
        const float kDefaultLaserYInBLFrame = 0.0;

        /**
         * ROS param name for the y coordinate of the lidar in the base_link frame.
         */
        static constexpr const char *kLaserYInBLFrameParamName = "laser_y_in_bl_frame";

        /**
         * Default orientation of the lidar relative to the base link frame.
         */
        const float kDefaultLaserOrientationRelBLFrame = 0.0;

        /**
         * ROS param name for the orientation of the lidar relative to the base link frame.
         */
        static constexpr const char *kLaserOrientationInBLFrameParamName = "laser_orientation_in_bl_frame";

        /**
         * ROS Param Name for the configuration for adding constraints between non-successive scans.
         */
        static constexpr const char *kNonSuccessiveScanConstraintsParamName = "non_successive_scan_constraints";

        /**
         * Default configuration for adding constraints between non-successive scans.
         */
        const bool kDefaultNonsuccessiveScanConstraints = true;

        /**
         * ROS Param Name for the configuration for adding constraints from the wheel odometry.
         */
        static constexpr const char *kOdometryConstraintsParamName = "odometry_constraints";

        /**
         * Default configuration for adding constraints from the wheel odometry.
         */
        const bool kDefaultOdometryConstraintsParamName = true;

        /**
         * ROS Param name for the variance in the x dimension to use in laser constraints.
         */
        static constexpr const char *kLaserXVarianceParamName = "laser_x_variance";

        /**
         * Default value for the variance in the x dimension to use in laser constraints.
         */
        const float kDefaultLaserXVariance = 0.5;
        // ^ 0.3 also works reasonably well

        /**
         * ROS Param name for the variance in the y dimension to use in laser constraints.
         */
        static constexpr const char *kLaserYVarianceParamName = "laser_y_variance";

        /**
         * Default value for the variance in the y dimension to use in laser constraints.
         */
        const float kDefaultLaserYVariance = 0.5;
        // ^ 0.3 also works reasonably well

        /**
         * ROS Param name for the variance in the theta dimension to use in laser constraints.
         */
        static constexpr const char *kLaserThetaVarianceParamName = "laser_theta_variance";

        /**
         * Default value for the variance in the theta dimension to use in laser constraints.
         */
        const float kDefaultLaserThetaVariance = 0.3;
        // ^ 0.2 also works reasonably well

        /**
         * ROS param name for the maximum number of non-successive observation factors that we can have per node.
         */
        static constexpr const char *kMaxObsFactorsPerNodeParamName = "max_obs_factors_per_node";

        /**
         * Default maximum number of non-successive observation factors that we can have per node.
         */
        const int kDefaultMaxObsFactorsPerNode = 15;

        /**
         * Default value for the number of factors that we should skip at a time when determining which factors to
         * consider adding.
         */
        const int kDefaultFactorSkipCount = 10;

        /**
         * Default value for the divisor for the fraction of points we should use in ICP.
         */
        const int kDefaultDownsampleIcpPointsRatio = 5;

        /**
         * ROS Param name for the divisor for the fraction of points we should use in ICP.
         */
        static constexpr const char *kDownsampleIcpPointsRatioParamName = "downsample_icp_points";

        /**
         * Divisor for the fraction of points we should use in ICP. (i.e. use 1/this many of the scan points from each
         * scan to send to ICP).
         */
        int downsample_icp_points_ratio_;

        /**
         * Variance in the x dimension to use in laser constraints.
         */
        float laser_x_variance_;

        /**
         * Variance in the y dimension to use in laser constraints.
         */
        float laser_y_variance_;

        /**
         * Variance in the theta dimension to use in laser constraints.
         */
        float laser_theta_variance_;

        /**
         * True if we should add constraints between non-successive scans, false if we should only add observation
         * constraints between successive nodes (should only be false for tuning/debugging purposes).
         */
        bool non_successive_scan_constraints_;

        /**
         * True if we should add constraints from the wheel odometry, false if we should only use observation based
         * constraints.
         */
        bool odometry_constraints_;

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

        /**
         * Number of iterations to run RANSAC when running ICP.
         */
        int ransac_iterations_;

        /**
         * True if ICP should enforce reciprocal correspondences, false if it should not.
         *
         * See https://pcl.readthedocs.io/projects/tutorials/en/latest/registration_api.html.
         */
        bool icp_use_reciprocal_correspondences_;

        // TODO want to have any of the following for ICP
        // ICP RANSAC outlier rejection threshold?
        // euclidean fitness epsilon?

        /**
         * Maximum amount that two nodes can be separated by to try to align their scans if they are within the same pass.
         */
        float maximum_node_dist_within_pass_scan_comparison_;

        /**
         * Maximum amount that two nodes can be separated by to try to align their scans if they are in different passes.
         */
        float maximum_node_dist_across_passes_scan_comparison_;

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

        /**
         * Maximum number of non-successive observation factors that we can have per node.
         */
        int max_factors_per_node_;

        /**
         * Number of factors that we should skip at a time when determining which factors to consider adding.
         *
         * Will cycle back through to start at the next unexamined factor if we don't have the maximum number of factors
         * by this number.
         */
        int factor_evaluation_skip_number_;

    };
}
