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

        // TODO set these

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
