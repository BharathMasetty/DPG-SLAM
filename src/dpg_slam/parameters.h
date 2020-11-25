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

    };
}
