//
// Created by amanda on 11/25/20.
//

#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "dpg_measurement.h"

namespace dpg_slam {

    class DpgNode {
    public:

        /**
         * Create the DPG node.
         *
         * @param init_pos                  Initial position estimate for the node.
         * @param init_orientation          Initial orientation estimate for the node.
         * @param pass_number               Number of the pass that the node is part of.
         * @param node_number               Node number (to be used in factor graph).
         * @param measurement               Measurement taken at the node.
         * @param laser_pos_rel_base_link   Pose of the laser relative to the base_link.
         */
        DpgNode(const Eigen::Vector2f &init_pos, const float &init_orientation, const uint32_t &pass_number,
                const uint32_t &node_number, const Measurement &measurement,
                const std::pair<Eigen::Vector2f, float> &laser_pos_rel_base_link) : node_loc_(init_pos),
                node_orientation_(init_orientation), changed_(false), is_active_(true), pass_number_(pass_number),
                node_number_(node_number), measurement_(measurement), laser_pos_rel_base_link_(laser_pos_rel_base_link) {

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

        /**
         * Get the estimated position of the node.
         * @return
         */
        std::pair<Eigen::Vector2f, float> getEstimatedPosition() const {
            return std::make_pair(node_loc_, node_orientation_);
        }

        /**
         * Get the measurement taken at the node.
         *
         * TODO should this return a reference instead so we can update the labels of the points in the
         * measurement and have that change reflected in the node?
         *
         * @return Measurement taken at the node.
         */
        Measurement getMeasurement() const {
            return measurement_;
        }

        /**
         * Number of the pass that the node belongs to.
         *
         * @return Pass number.
         */
        uint32_t getPassNumber() const {
            return pass_number_;
        }

        /**
         * Number of the node. Provides identifier in factor graph.
         *
         * @return node number.
         */
        uint64_t getNodeNumber() const {
            return node_number_;
        }

        /**
         * Get the cached point cloud representing the measurement at the node, with all points relative to the base
         * link frame (measurement is relative to the laser frame). If the point cloud has not yet been created, it
         * will be created and cached.
         *
         * This contains all points, not just those that are in activated sectors or having a particular label.
         *
         * @return Point cloud relative to base_link representing the full scan taken at this node.
         */
        pcl::PointCloud<pcl::PointXYZ>::Ptr getCachedPointCloudFromNode();

        /**
         * Deactivate sectors of this node that intersect with the given removed points and deactivate the node if it
         * has too few active sectors after this.
         *
         * @param removed_points        Points that have been removed, in the map frame.
         * @param min_active_sectors    Minimum percentage of active sectors for a node to still be active.
         */
        void deactivateIntersectingSectors(const std::vector<Eigen::Vector2f> &removed_points, const float &min_active_sectors);
    	
	/*
	 * Check of the sector corresponding to a given measurement is active or not
	 * @param Index of the measurement points.
	 *
	 * @return bool indicating active sector or not
	 */
	bool isPointInActiveSector(const uint32_t measurementPointIndex);

	/**
	 * To check if the node is active or not
	 */
	bool isNodeActive() {
	   return is_active_;
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

        /**
         * Position of the laser relative to the base link frame.
         */
        std::pair<Eigen::Vector2f, float> laser_pos_rel_base_link_;

        // This has all points in the measurement.
        // TODO consider refreshing/making other copy with only active points
        /**
         * Point cloud relative to base_link representing the full scan at this node.
         */
        pcl::PointCloud<pcl::PointXYZ>::Ptr cached_point_cloud_;
    };
} // end dpg_slam
