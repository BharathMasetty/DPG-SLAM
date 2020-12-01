#pragma once

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include "parameters.h"
#include "dpg_node.h"
#include "math_utils.h"

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/PriorFactor.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <amrl_msgs/VisualizationMsg.h>
#include <boost/functional/hash.hpp>

typedef std::pair<int, int> cellKey;

namespace dpg_slam {
    
    /**
     * Data structure to define the occupancy grid.
     */
    class occupancyGrid {
    public:
       
	occupancyGrid(const std::vector<DpgNode> &Nodes):
		Nodes_(Nodes){
		calculateOccupancyGrid();
		}

	bool isCellOccupied(const DpgNode& node){
		
		//TODO: Fill in - Bharath
		
		return false;
	}
	
	/*
         * To access the gridInfo
         */
        std::unordered_map<cellKey, bool, boost::hash<cellKey>> getGridInfo(){
                return gridInfo;
        }

	/*
         * Mapping from keys based on cell location in map to the boolean representing of there is a map point in the grid or not.
         */
        std::unordered_map<cellKey, bool, boost::hash<cellKey>> gridInfo;
    
    private:
	
	/**
     	 * Convert the dpg_node to ints based on resolution for occupancy grid.
         */
        cellKey convertToKeyForm(const Eigen::Vector2f& loc) const;
		
	/*
	 * To fill in the occupancy grid based on input node vectors 
	 */
	void calculateOccupancyGrid();

	/*
	 * For converting measurement point at a node to map frame
	 */
	std::vector<cellKey> convertLaserRangeToCellKey(const DpgNode& node);
    		
	
	/*
	 * Nodes for which the occupancy grid is to be made.
	 */
	std::vector<DpgNode> Nodes_;

	/*
	 * DPG Parameters
	 */
	DpgParameters dpg_parameters_;

	/**
         * Pose graph parameters.
         */
        PoseGraphParameters pose_graph_parameters_;

    };

    /**
     * Data structure to define a map point in the active and dynamic maps.
     */
    struct dpgMapPoint{

    	Eigen::Vector2f mapPoint;
 	PointLabel label;
    };
    
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
                          const float &range_min,
                          const float &range_max,
                          const float &angle_min,
                          const float &angle_max);

        // Observe new odometry-reported location.
        void ObserveOdometry(const Eigen::Vector2f& odom_loc,
                             const float odom_angle);

        /**
         * Increment the pass number to indicate that we're on the next pass.
         *
         * TODO Need to hook this up to something
         */
        void incrementPassNumber();

        // Get latest map.
        std::vector<Eigen::Vector2f> GetMap();
	
	/**
	 * Get latest active map 
	 */
	std::vector<dpgMapPoint> GetActiveMap();

	// Get latest dynamic map
	std::vector<dpgMapPoint> GetDynamicMap();


        // Get latest robot pose.
        void GetPose(Eigen::Vector2f* loc, float* angle) const;

        /**
         * Add the currently estimated trajectory and the odometry-only estimate to the visualization.
         *
         * Odometry estimate may not start at the same pose.
         *
         * @param visualization_msg[out] Visualization message to add trajectory and odometry estimates to.
         */
        void publishTrajectory(amrl_msgs::VisualizationMsg &visualization_msg);

    private:

        /**
         * Number of the first pass. Subsequent passes will be incremented by 1.
         */
        const uint32_t kInitialPassNumber = 0;

        /**
         * Length of the line segment to use when visualizing a pose in the trajectory.
         */
        const float kTrajectoryPlotLineSegName = 0.4;

        /**
         * Color to use when visualizing the SLAM estimated trajectory.
         */
        const uint32_t kTrajectoryColor = 0x34b4eb;

        /**
         * Color to use when visualizing the odometry.
         */
        const uint32_t kOdometryEstColor = 0x3400b;

        /**
         * List of nodes forming the trajectory.
         */
        std::vector<DpgNode> dpg_nodes_;

        /**
         * GTSAM factor graph.
         */
        gtsam::NonlinearFactorGraph* graph_;

        /**
         * DPG specific parameters.
         */
        DpgParameters dpg_parameters_;

        /**
         * Pose graph parameters.
         */
        PoseGraphParameters pose_graph_parameters_;

        /**
         * Pass number that the robot is currently on.
         */
        uint32_t pass_number_;

	/**
	 * 
	 */
	uint32_t kNumNodesInCurrentPass = 0;

        /**
         * Latest odometry location reported.
         */
        Eigen::Vector2f prev_odom_loc_;

        /**
         * Latest odometry angle reported.
         */
        float prev_odom_angle_;

        /**
         * True if odom has been initialized for the current pass, false if not.
         */
        bool odom_initialized_;

        /**
         * True if the next scan will be the first for the pass, false if not.
         */
        bool first_scan_for_pass_;

        /**
         * Odometry position at the last laser scan used.
         */
        Eigen::Vector2f odom_loc_at_last_laser_align_;

        /**
         * Odometry angle at the last laser scan used.
         */
        float odom_angle_at_last_laser_align_;

        /**
         * Pose estimates from odometry only. May not be in the same frame as the trajectory estimates.
         */
        std::vector<std::pair<Eigen::Vector2f, float>> odom_only_estimates_;

        /**
         * Determine if the robot has moved far enough that we should compare the last used laser scan to the current
         * laser scan.
         *
         * This isn't considered if a scan is the first in its pass.
         *
         * @return True if the robot has moved far enough that we should compare the last used laser scan to the current
         * laser scan, false if we should skip this laser scan.
         */
        bool shouldProcessLaser();

        /**
         * Update the pose graph based on the laser scan. Returns true if DPG should be run after finishing the pose
         * graph update. May not change the pose graph if the estimated robot pose is too close to the last node.
         *
         * @param ranges    Range readings from the laser scan.
         * @param range_min Minimum range value possible from the LiDAR.
         * @param range_max Maximum range value possible from the LiDAR. We'll consider points return this range to
         *                  mean there is no obstacle.
         * @param angle_min Minimum angle of the LiDAR.
         * @param angle_max Maximum angle of the LiDAR.
         *
         * @return True if DPG should be run after finishing the pose graph update, false if DPG should not be run.
         */
        bool updatePoseGraph(const std::vector<float>& ranges,
                             const float &range_min,
                             const float &range_max,
                             const float &angle_min,
                             const float &angle_max);

        /**
         * Update the underlying pose graph.
         *
         * This should add the new node to the nodes list and pose graph and add constraints to the node.
         */
        void updatePoseGraphObsConstraints(DpgNode &new_node);

        /**
         * Get the position of the laser relative to the base link frame.
         *
         * @return position of the laser relative to the base link frame. First component of pair is the position of
         * the laser frame's origin in the base link frame. The second component of the pair is the relative
         * orientation.
         */
        std::pair<Eigen::Vector2f, float> getLaserPositionRelativeToBaselink();

        /**
         * Add a single observation constraint.
         *
         * @param from_node_num     Number of the node that the constaint is from (estimating position of to node
         *                          relative to this node).
         * @param to_node_num       Number of the node that the constraint is to (estimating position of this node
         *                          relative to the from node).
         * @param constraint_info   Constraint information. The first part is the estimated pose of the to node
         *                          relative to the from node. The second part is the covariance.
         */
        void addObservationConstraint(const size_t &from_node_num, const size_t &to_node_num,
                                      std::pair<std::pair<Eigen::Vector2f, float>, Eigen::MatrixXd> &constraint_info);

        /**
         * Create a node that is the first in its pass.
         *
         * @param ranges        Range readings from the laser scan at the node.
         * @param range_min     Minimum range value possible from the LiDAR.
         * @param range_max     Maximum range value possible from the LiDAR. We'll consider points return this range to
         *                      mean there is no obstacle.
         * @param angle_min     Minimum angle of the LiDAR.
         * @param angle_max     Maximum angle of the LiDAR.
         * @param pass_number   Number of the pass.
         *
         * @return DpgNode containing the measurement information that is passed in.
         */
        DpgNode createNewPassFirstNode(const std::vector<float> &ranges, const float &range_min, const float &range_max,
                                       const float &angle_min, const float &angle_max, const uint32_t &pass_number);

        /**
         * Create a node with its initial position as the odometry displacement on top of the position of the last node.
         *
         * @param ranges                    Range readings from the laser scan at the node.
         * @param range_min                 Minimum range value possible from the LiDAR.
         * @param range_max                 Maximum range value possible from the LiDAR. We'll consider points return
         *                                  this range to mean there is no obstacle.
         * @param angle_min                 Minimum angle of the LiDAR.
         * @param angle_max                 Maximum angle of the LiDAR.
         * @param odom_displacement         Estimated translation relative to the last node.
         * @param odom_orientation_change   Estimated rotation relative to the last node.
         * @param pass_number               Pass number.
         *
         * @return DpgNode with an estimated position of the odometry displacement on top of the last node's estimated
         * position and measurement info based on the given laser range data.
         */
        DpgNode createRelativePositionedNode(const std::vector<float> &ranges, const float &range_min,
                                             const float &range_max, const float &angle_min, const float &angle_max,
                                             const Eigen::Vector2f &odom_displacement,
                                             const float &odom_orientation_change, const uint32_t &pass_number);

        /**
         * Create a DpgNode with the given laser scan information.
         *
         * @param ranges                    Range readings from the laser scan at the node.
         * @param range_min                 Minimum range value possible from the LiDAR.
         * @param range_max                 Maximum range value possible from the LiDAR. We'll consider points return
         *                                  this range to mean there is no obstacle.
         * @param angle_min                 Minimum angle of the LiDAR.
         * @param angle_max                 Maximum angle of the LiDAR.
         * @param init_pos                  Initial position estimate for the node.
         * @param init_orientation          Initial orientation estimate for the node.
         * @param pass_number               Pass number of the node.
         * @param node_number               Node number of the node (used to identify the node in the factor graph).
         *
         * @return DpgNode containing measurment data based on the given laser range data.
         */
        DpgNode createNode(const std::vector<float>& ranges, const float &range_min, const float &range_max,
                           const float &angle_min, const float &angle_max, const Eigen::Vector2f &init_pos,
                           const float &init_orientation, const uint32_t &pass_number, const uint32_t &node_number);

        /**
         * Run ICP on the measurements of the two nodes to get the estimated position of node 2 in the frame of node 1.
         *
         * @param node_1    First node with measurements. Want to estimate the position of node 2 relative to this node.
         * @param node_2    Second node with measurements. Want to estimate the position of this node relative to node 1.
         *
         * @return Pair with first entry as estimated position of node 2 relative to node 1 based on scan alignment and
         * second entry as estimated covariance.
         */
        std::pair<std::pair<Eigen::Vector2f, float>, Eigen::MatrixXd> runIcp(DpgNode &node_1, DpgNode &node_2);
    
     	/*
	 * computes the local submapGrid and the currGrid
	 * each map is an unordered map from keys in x,y to boolean which says if the grid is occupied or not (int, int)->bool
	 * TODO: Need to figure out the data structure for occupancy grids/
	 */
	std::vector<occupancyGrid> computeLocalSubMap();
	
	/*
	 * To find the nodes covering the occupancy map generated by current pose chain
	 * @param occupancy grid made for current pose chain.
	 * @return vector of nodes covering the current occupancy grid.
	 */	
	std::vector<DpgNode> getNodesCoveringCurrGrid(const occupancyGrid &currGrid);

	/*
	 * To check if two occupancy grids overlap
	 * @param Grid1
	 * @param Grid2
	 * @return boolean indicating if there is any intersection of feild of views.
	 */
	bool checkGridIntersection(const occupancyGrid &Grid1, const occupancyGrid &Grid2);

	/**
	 * This method compares each cell in the currGrid to the corresponding cell is the submap.
	 * @param currGrid 		occupancy grid of current pose chain obtained from computeLocalSubmap
	 * @param submapGrid		occupancy grid of local submap obtained from computeLocalSubmap
	 *
	 * @return Vector of dynamic map points with associated labels for measurements in current Grid
	 */
	std::vector<dpgMapPoint> detectAndLabelChanges(const occupancyGrid& currGrid, const occupancyGrid& submapGrid);

	/**
	 * @param removedPoints 	Points from the dynamic map points obtained from detectAndLabelChanges that
	 * 				are labelled as "REMOVED".
	 *
	 * @return vector of inactive nodes.
	 */
	std::vector<DpgNode> updateActiveAndDynamicMaps(const std::vector<DpgNode> &nodes, const std::vector<dpgMapPoint> &removedPoints);

	/**
	 * Method to delete inactive nodes from dpg graph
	 * NOTE: This is not a top priority.
	 */
	void reduceDPGSize();
	
    	/**
	 * Main execution Loop for DPG
	 */
	void updateDPG();	
    
    };
}  // end dpg_slam
