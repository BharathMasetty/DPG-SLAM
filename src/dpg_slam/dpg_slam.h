#pragma once

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include "parameters.h"
#include "dpg_node.h"
#include "math_utils.h"
#include <unordered_set>

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/PriorFactor.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <amrl_msgs/VisualizationMsg.h>
#include <boost/functional/hash.hpp>

#include <nav_msgs/OccupancyGrid.h>
#include <gtsam/nonlinear/ISAM2.h>

typedef std::pair<int, int> cellKey;

enum CellStatus {UNKNOWN, FREE, OCCUPIED};

struct PointIdInfo {
    PointIdInfo(const uint64_t &node_num,
                const uint64_t &point_num_in_node) : node_num_(node_num), point_num_in_node_(point_num_in_node) {

    }
    uint64_t node_num_;
    uint64_t point_num_in_node_;
};

struct OccupiedCellInfo {
    std::vector<PointIdInfo> points_;
};

namespace dpg_slam {
    
    /**
     * Data structure to define the occupancy grid.
     */
    class occupancyGrid {
    public:
    
    // Constructor without any nodes 
    occupancyGrid(const DpgParameters &dpg_parameters, const PoseGraphParameters &pose_graph_parameters) :
	      dpg_parameters_(dpg_parameters), pose_graph_parameters_(pose_graph_parameters) {}	    
 
    // Constructor for combining two occupancy grids.
    occupancyGrid(const occupancyGrid &grid1, const occupancyGrid &grid2, const DpgParameters &dpg_parameters, 
	      const PoseGraphParameters &pose_graph_parameters) : dpg_parameters_(dpg_parameters), 
    	      pose_graph_parameters_(pose_graph_parameters), min_cell_x_(INT_MAX), max_cell_x_(INT_MIN),
               min_cell_y_(INT_MAX), max_cell_y_(INT_MIN), include_inactive_(false), include_added_(true), 
	       include_static_(true) {
 	combineOccupancyGrids(grid1, grid2);
    }	
    
    // Constructor for creating an occupancygrid with just one node.
    occupancyGrid(const DpgNode &current_node, const DpgParameters &dpg_parameters,
	       const PoseGraphParameters &pose_graph_parameters) : dpg_parameters_(dpg_parameters), 
     	       pose_graph_parameters_(pose_graph_parameters), min_cell_x_(INT_MAX), max_cell_x_(INT_MIN),
               min_cell_y_(INT_MAX), max_cell_y_(INT_MIN), include_inactive_(false), include_added_(true), 
	       include_static_(true), is_single_node_grid_(true) {
	Nodes_.push_back(current_node);
	calculateOccupancyGrid();
    }    
       
    occupancyGrid(const std::vector<DpgNode> &Nodes, const DpgParameters &dpg_parameters,
               const PoseGraphParameters &pose_graph_parameters) : Nodes_(Nodes), dpg_parameters_(dpg_parameters),
               pose_graph_parameters_(pose_graph_parameters), min_cell_x_(INT_MAX), max_cell_x_(INT_MIN),
               min_cell_y_(INT_MAX), max_cell_y_(INT_MIN), include_inactive_(false), include_added_(true), include_static_(true),
	       is_single_node_grid_(false) {
        calculateOccupancyGrid();
    }
    
    /**
     * Alternate constructor that will allow us to visualize the results as an occupancy grid.
     *
     * @param Nodes                     Nodes to convert to an occupancy grid.
     * @param dpg_parameters            DPG specific parameters.
     * @param pose_graph_parameters     General pose graph parameters.
     * @param include_inactive          Set to true if the occupancy grid should include inactive (including removed)
     *                                  points, false if it should only include things that would be in the active map.
     * @param include_added             Set to true if the occupancy grid should include added points.
     * @param include_static            Set to true if the occupancy grid should include static points.
     */
    occupancyGrid(const std::vector<DpgNode> &Nodes, const DpgParameters &dpg_parameters,
                  const PoseGraphParameters &pose_graph_parameters, const bool include_inactive,
                  const bool include_added, const bool include_static) : Nodes_(Nodes), dpg_parameters_(dpg_parameters),
                  pose_graph_parameters_(pose_graph_parameters), min_cell_x_(INT_MAX), max_cell_x_(INT_MIN),
                  min_cell_y_(INT_MAX), max_cell_y_(INT_MIN), include_inactive_(include_inactive),
                  include_added_(include_added), include_static_(include_static), is_single_node_grid_(false) {
        calculateOccupancyGrid();
    }
	
    // Assignment operator for copying an occupancy grid.
    occupancyGrid & operator = (const occupancyGrid &grid) {
    
    	if(this != &grid) {
		// Probably will not need to copy mode member variables then these three.
		// But add here if more info need to be copied. 
		Nodes_ = grid.Nodes_;
		gridInfo = grid.gridInfo;
		occupied_cell_info_ = grid.occupied_cell_info_; 	
	};
	return *this;
    }

    /**
     * Get the occupancy value for the given cell key.
     *
     * @param cell_key  Cell key to get the occupancy value for.
     *
     * @return occupancy value for the given cell key.
     */
    CellStatus getCellStatus(const cellKey &cell_key) const {
        if (gridInfo.find(cell_key) == gridInfo.end()) {
            return UNKNOWN;
        }
        return gridInfo.at(cell_key);
    }

    /**
     * Get the points that contributed to the occupancy of a cell.
     *
     * @param cell_key  Cell key.
     *
     * @return Points that fall within the cell.
     */
    std::vector<PointIdInfo> getPointsInOccCell(const cellKey &cell_key) const {
        if (occupied_cell_info_.find(cell_key) == occupied_cell_info_.end()) {
            return {};
        }
        return occupied_cell_info_.at(cell_key).points_;
    }      
    
    /**
     * To access the gridInfo
     */
    std::unordered_map<cellKey, CellStatus, boost::hash<cellKey>> getGridInfo() const {
        return gridInfo;
    }

    /**
     * Convert this to an ocupancy grid message so we can publish it.
     *
     * @param occ_grid_msg[out]     Occupancy grid message to populate.
     * @param distinguish_free[in]  True if we should add free cells as well as occupied cells (will be true for
     * visualizing submap, will be false when we want to get the occ grid for the different results layers).
     */
    void toOccGridMsg(nav_msgs::OccupancyGrid &occ_grid_msg, bool distinguish_free);

    /**
     * Mapping from keys based on cell location in map to the boolean representing of there is a map point in the grid or not.
     */
    std::unordered_map<cellKey, CellStatus, boost::hash<cellKey>> gridInfo;

    private:
    
        /**
         * Convert the dpg_node to ints based on resolution for occupancy grid.
         */
         cellKey convertToKeyForm(const Eigen::Vector2f& loc) const;

        /*
         * To fill in the occupancy grid based on input node vectors
         */
        void calculateOccupancyGrid();
	
	/**
	 * For combining two existing occupancy grids
	 */
	void combineOccupancyGrids(const occupancyGrid &grid1, const occupancyGrid &grid2);


        /*
         * For converting measurement point at a node to map frame
         */
        void convertLaserRangeToCellKey(const DpgNode& node);

        /**
         * For getting the cells falling inside the foc of a node that are free.
         */
        std::vector<cellKey> getIntermediateFreeCellsInFOV(const Eigen::Vector2f &LaserLoc, const Eigen::Vector2f &scanLoc,  const float& range);

        /*
         * For labling the occupied cells.
         *
         * @param vector of occupied cellKeys
         */
        void setOccupiedCells(const std::vector<cellKey> &occupiedCells);

        /**
         * For labling the free cells.
         *
         * @param vector of free cellKeys
         */
        void setFreeCells(const std::vector<cellKey> &freeCells);


        std::unordered_map<cellKey, OccupiedCellInfo, boost::hash<cellKey>> occupied_cell_info_;

        /*
         * Nodes for which the occupancy grid is to be made.
         */
        std::vector<DpgNode> Nodes_;

        /*
         * DPG Parameters
         */
        DpgParameters dpg_parameters_;

        /**
         * Pose graph parameters
         */
        PoseGraphParameters pose_graph_parameters_;

        /**
         * Minimum value for the x part of the key. Provides occ grid range.
         */
        int min_cell_x_;

        /**
         * Maximum value for the x part of the key. Provides occ grid range.
         */
        int max_cell_x_;

        /**
         * Minimum value for the y part of the key. Provides occ grid range.
         */
        int min_cell_y_;

        /**
         * Maximum value for the y part of the key. Provides occ grid range.
         */
        int max_cell_y_;

        /**
         * Set to true if the occupancy grid should include inactive (including removed) points, false if it should
         * only include things that would be in the active map.
         */
        bool include_inactive_;

        /**
         * Set to true if the occupancy grid should include added points.
         */
        bool include_added_;

        /**
         * Set to true if the occupancy grid should include static points.
         */
        bool include_static_;

	/**
	 * Set true of the occupancy gris is only for one node.
	 */
	bool is_single_node_grid_;

	/**
	 * is grid empty or not
	 */
	bool is_grid_empty_;
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
         * @param dpg_parameters            DPG specific parameters.
         * @param pose_graph_parameters     Pose graph parameters.
         * @param visualization_parameters  Visualization parameters.
         */
        DpgSLAM(const DpgParameters &dpg_parameters, const PoseGraphParameters &pose_graph_parameters,
                const VisualizationParams &visualization_parameters);

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
         * Get the active and dynamic map points. Each list will be populated with the matching points in the map frame.
         *
         * TODO is there any way to optimize this so we don't have to basically compute a massive point cloud every time?
         * Doesn't seem like it because the poses can change and the activations can change at each time step.
         *
         * @param active_static_points[out]     Static points in the active map.
         * @param active_added_points[out]      Added points in the active map.
         * @param dynamic_removed_points[out]   Removed points (from inactive and active nodes). Make up the removed part of the dynamic map.
         * @param dynamic_added_points[out]     Added points from all nodes. Make up the added part of the dynamic map.
         */
        void getActiveAndDynamicMapPoints(std::vector<Eigen::Vector2f> &active_static_points,
                                          std::vector<Eigen::Vector2f> &active_added_points,
                                          std::vector<Eigen::Vector2f> &dynamic_removed_points,
                                          std::vector<Eigen::Vector2f> &dynamic_added_points);

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
         * ISAM2 object for incremental smoothing and mapping.
         */
        gtsam::ISAM2 *isam_;

        /**
         * DPG specific parameters.
         */
        DpgParameters dpg_parameters_;

        /**
         * Pose graph parameters.
         */
        PoseGraphParameters pose_graph_parameters_;

        /**
         * Visualization params.
         */
        VisualizationParams visualization_params_;

        /**
         * Pass number that the robot is currently on.
         */
        uint32_t pass_number_;

        /**
         * List of nodes in the current pass
         */
        std::vector<DpgNode> current_pass_nodes_;

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
         * Total distance that the car has moved since the last laser alignment.
         *
         * This is needed because comparing positions didn't deal with 3-point-turn-like maneuvers well.
         */
        float cumulative_dist_since_laser_laser_align_;

        /**
         * Pose estimates from odometry only. May not be in the same frame as the trajectory estimates.
         */
        std::vector<std::pair<Eigen::Vector2f, float>> odom_only_estimates_;
	
        /**
         * Reoptimize the poses.
         *
         * This is an expensive operation where we recompute the scan constraints using the offset between the
         * estimated pose (instead of using odometry as a seed). This is used between passes to clean up constraints
         * that reflect poor ICP alignment.
         */
        void reoptimize();

        /**
         * Optimize the pose graph and update the estimated poses in the nodes.
         *
         * @param new_node_init_estimates Initial pose estimated for nodes that have been added to the graph since the
         * last optimization.
         */
        void optimizeGraph(gtsam::Values &new_node_init_estimates);

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
         * Detect and label changes for the current pose chain.
         *
         * @param current_submap_occ_grids[in]  Occupancy grids representing each one of the current pose chain nodes.
         * @param local_submap_occ_grid[in]     Occupancy grid for the local submap. // TODO should this also be separated by
         * @param removed_points[out]           Points that have been removed in this iteration of detecting and labeling.
         */
        void detectAndLabelChangesForCurrentPoseChain(const std::vector<occupancyGrid> &current_submap_occ_grids,
                                                      const occupancyGrid &local_submap_occ_grid,
                                                      std::vector<PointIdInfo> &removed_points);

        /**
         * Detect and label the changes for the current pose chain node and the overlapping submap nodes.
         *
         * @param current_node_occ_grid[in] Current pose chain node's occupancy grid.
         * @param local_submap_occ_grid[in] Local submap occupancy grid.
         * @param added_points[out]         Points that were identified as added.
         * @param removed_points[out]       Points that were identified as removed.
         */
        void detectAndLabelChangesForCurrentNode(
                const occupancyGrid &current_node_occ_grid,
                const occupancyGrid &local_submap_occ_grid,
                std::vector<PointIdInfo> &added_points,
                std::vector<PointIdInfo> &removed_points);

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
         * Downsample the given point cloud and put it in the downsampled_cloud pointer.
         *
         * @param input_cloud[in]           Cloud to downsample.
         * @param downsample_divisor[in]    Divisor for fraction of points to keep (every 1/this many points).
         * @param downsampled_cloud[out]    Smaller size cloud that was downsampled from the input cloud.
         */
        void downsamplePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                                  const int &downsample_divisor,
                                  pcl::PointCloud<pcl::PointXYZ>::Ptr &downsampled_cloud);

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
         * @param node_1[in]        First node with measurements. Want to estimate the position of node 2 relative to
         *                          this node.
         * @param node_2[in]        Second node with measurements. Want to estimate the position of this node relative
         *                          to node 1.
         * @param icp_results[out]  Pair with first entry as estimated position of node 2 relative to node 1 based on
         *                          scan alignment and second entry as estimated covariance.
         *
         * @return True if the ICP run converged, false if it didn't converge.
         */
        bool runIcp(DpgNode &node_1, DpgNode &node_2, std::pair<std::pair<Eigen::Vector2f, float>, Eigen::MatrixXd> &icp_results);
    
        /*
         * computes the local submapGrid and the currGrid
         * each map is an unordered map from keys in x,y to boolean which says if the grid is occupied or not (int, int)->bool
         * TODO: Need to figure out the data structure for occupancy grids/
         */
	std::pair<std::vector<occupancyGrid>, occupancyGrid> computeLocalSubMap();

        /*
         * To find the nodes covering the occupancy map generated by current pose chain
         * @param occupancy grid made for current pose chain.
         * @return vector of nodes covering the current occupancy grid.
         */
        occupancyGrid getSubMapCoveringCurrPoseChain(const std::vector<DpgNode> &poseChain,
							   const std::vector<occupancyGrid> &poseChainGrids);
	/**
	 * Modifies the cells in the current uncovered cells set by checking if the current verison 
	 * of the localsubmap covers any of those points.
	 * @param currentUncoveredCells  set of cells that are uncovered by local submap  yet.
	 * @param current version of local submap
	 */	
 	void getUpdatedCoverageForCurrentPoseChain(std::unordered_set<cellKey, boost::hash<cellKey>> &currentUncoveredCells, 
						  const occupancyGrid &localSubMap);
	/**
	 * Check if enough bins are changed at current node to commit changes in the labels of observations
	 * @param added_points at the node
	 * @param removed_points at the node
	 *
	 * @return whether or not to commit changes.
	 */
	bool computeBinScoreAndCommitLabelsForNode(const std::vector<PointIdInfo> &addded_points,
                                                   const std::vector<PointIdInfo> &removed_points);
        /*
         * To check if two occupancy grids overlap
         * @param Grid1
         * @param Grid2
         * @return boolean indicating if there is any intersection of feild of views.
         */
        bool checkGridIntersection(const occupancyGrid &Grid1, const occupancyGrid &Grid2);

        /**
         * Method to delete inactive nodes from dpg graph
         * NOTE: This is not a top priority.
         */
        void reduceDPGSize();

        /**
         * Main execution Loop for DPG
         */
        void executeDPG();

	/**
	 * Turn off sectors containing removed points in sectors of nodes from previous pass.
	 * @param removed points list.
	 */
	void updateNodesAndSectorStatus(const std::vector<PointIdInfo> &removedPoints);
    };
}  // end dpg_slam
