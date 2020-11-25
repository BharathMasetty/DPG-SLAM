//
// Created by amanda on 11/25/20.
//

#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "dpg_measurement.h"

namespace dpg_slam {

    class DpgNode {
    public:

        DpgNode(const Eigen::Vector2f &init_pos, const float &init_orientation, const uint32_t &pass_number,
                const uint32_t &node_number, const Measurement &measurement) : node_loc_(init_pos),
                node_orientation_(init_orientation), changed_(false), is_active_(true), pass_number_(pass_number),
                node_number_(node_number), measurement_(measurement) {

        }

        /**
         * Set the position of the node.
         *
         * This should be called after a pose-graph optimization is called.
         *
         * @param new_loc           New location for the node.
         * @param new_orientation   New orientation for the node.
         */
        void setPosition(const Eigen::Vector2f &new_loc, const float &new_orientation) {
            node_loc_ = new_loc;
            node_orientation_ = new_orientation;
        }

        /**
         * Mark the node as changed.
         *
         * TODO can it be set back to unchanged...? Doesn't seem like it.
         */
        void setChanged() {
            changed_ = true;
        }

        /**
         * Mark the node as inactive.
         */
        void setInactive() {
            is_active_ = false;
        }
    private:

        /**
         * Location of the node, as computed by the pose graph algorithm.
         */
        Eigen::Vector2f node_loc_;

        /**
         * Orientation of the node, as computed by the pose graph algorithm.
         */
        float node_orientation_;

        /**
         * True if a change has been detected, false otherwise.
         */
        bool changed_;

        /**
         * True if the node is active, false if it is inactive.
         */
        bool is_active_;

        /**
         * Pass number when the node was taken.
         */
        uint32_t pass_number_;

        /**
         * Number of the node.
         */
        uint64_t node_number_;

        /**
         * Measurement taken at this node. This data will be relative to the laser scanner, even though the node is
         * relative to the robot's coordinate frame.
         */
        Measurement measurement_;
    };
}