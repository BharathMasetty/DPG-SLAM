
#include "dpg_slam.h"
#include <pcl/registration/icp.h>
#include "icp_cov/cov_func_point_to_point.h"

#include <visualization/visualization.h>

#include <ros/ros.h>

using namespace gtsam;
using namespace Eigen;

namespace dpg_slam {

    DpgSLAM::DpgSLAM(const DpgParameters &dpg_parameters,
                     const PoseGraphParameters &pose_graph_parameters) : dpg_parameters_(dpg_parameters),
                     pose_graph_parameters_(pose_graph_parameters), pass_number_(kInitialPassNumber),
                     odom_initialized_(false), first_scan_for_pass_(true) {

        graph_ = new NonlinearFactorGraph();
    }

    void DpgSLAM::incrementPassNumber() {
        pass_number_++;
        odom_initialized_ = false;
        first_scan_for_pass_ = true;
    }

    void DpgSLAM::ObserveLaser(const std::vector<float>& ranges,
                      const float &range_min,
                      const float &range_max,
                      const float &angle_min,
                      const float &angle_max) {
        if (!odom_initialized_) {
            ROS_ERROR_STREAM("Odom not initialized. Skipping laser processing.");
            return;
        }

        bool should_update_dpg = updatePoseGraph(ranges, range_min, range_max, angle_min, angle_max);
        if (!should_update_dpg) {
            return;
        }

        // TODO DPG processing
    }

    void DpgSLAM::publishTrajectory(amrl_msgs::VisualizationMsg &vis_msg) {
        for (const DpgNode &node : dpg_nodes_) {
            std::pair<Vector2f, float> trajectory_point = node.getEstimatedPosition();

            Vector2f second_point(kTrajectoryPlotLineSegName * cos(trajectory_point.second), kTrajectoryPlotLineSegName * sin(trajectory_point.second));
            visualization::DrawLine(trajectory_point.first,
                                    trajectory_point.first + second_point,
                                    kTrajectoryColor, vis_msg);
        }

        for (const auto &odom_pos : odom_only_estimates_) {
            Vector2f second_point(kTrajectoryPlotLineSegName * cos(odom_pos.second), kTrajectoryPlotLineSegName * sin(odom_pos.second));
            visualization::DrawLine(odom_pos.first,
                                    odom_pos.first + second_point,
                                    kOdometryEstColor, vis_msg);
        }
    }

    bool DpgSLAM::updatePoseGraph(const std::vector<float>& ranges, const float &range_min, const float &range_max,
                                  const float &angle_min, const float &angle_max) {

        // If this is the first scan for a pass, then we should not have an odom constraint between the new node and
        // the previous. Instead have a loose constraint relating it to 0, 0, 0.
        bool no_prev_odom_ = false;
        if (first_scan_for_pass_) {
            first_scan_for_pass_ = false;
            no_prev_odom_ = true;
        }

        if (no_prev_odom_) {

            ROS_INFO_STREAM("First node for pass");

            // Add prior instead of odom
            DpgNode new_node = createNewPassFirstNode(ranges, range_min, range_max, angle_min, angle_max,
                                                      pass_number_);
            Pose2 position_prior(0.0, 0.0, 0.0);
            noiseModel::Diagonal::shared_ptr prior_noise =
                    noiseModel::Diagonal::Sigmas(Vector3(pose_graph_parameters_.new_pass_x_std_dev_,
                                                         pose_graph_parameters_.new_pass_y_std_dev_,
                                                         pose_graph_parameters_.new_pass_theta_std_dev_));
            graph_->add(PriorFactor<Pose2>(new_node.getNodeNumber(), position_prior, prior_noise));

            odom_only_estimates_.emplace_back(std::make_pair(prev_odom_loc_, prev_odom_angle_));
            odom_loc_at_last_laser_align_ = prev_odom_loc_;
            odom_angle_at_last_laser_align_ = prev_odom_angle_;

            if (pass_number_ == kInitialPassNumber) {

                // If this is the very first node, just add the prior, make the node, and return
                // TODO should we also set the labels for all measurements in the first node to static since we won't
                //  run DPG on them separately?
                dpg_nodes_.push_back(new_node);
                return false;
            } else {

                // If this is not the first pass, also compare observation to last node in last pass and other nodes in
                // close enough locations
                updatePoseGraphObsConstraints(new_node);
            }
        } else {

            // If this isn't the first node in the pass, we should ensure we've moved enough to warrant a new node
            if (!shouldProcessLaser()) {
                return false;
            }

            // Get the estimated position change since the last node due to odometry
            std::pair<Vector2f, float> relative_loc_latest_pose = math_utils::inverseTransformPoint(prev_odom_loc_, prev_odom_angle_, odom_loc_at_last_laser_align_, odom_angle_at_last_laser_align_);
            Vector2f odom_est_loc_displ = relative_loc_latest_pose.first;
            float odom_est_angle_displ = relative_loc_latest_pose.second;

            // Create the node with the initial position estimate as the last node's position plus the odom
            DpgNode new_node = createRelativePositionedNode(ranges, range_min, range_max, angle_min, angle_max,
                                                            odom_est_loc_displ,
                                                            odom_est_angle_displ, pass_number_);

            // Add an odometry constraint
            float transl_std_dev = (pose_graph_parameters_.motion_model_transl_error_from_transl_ * (odom_est_loc_displ.norm())) +
                                   (pose_graph_parameters_.motion_model_transl_error_from_rot_ * fabs(odom_est_angle_displ));
            float rot_std_dev = (pose_graph_parameters_.motion_model_rot_error_from_transl_ * (odom_est_loc_displ.norm())) +
                                (pose_graph_parameters_.motion_model_rot_error_from_rot_ * fabs(odom_est_angle_displ));

            // TODO should x and y standard deviation be different?
            noiseModel::Diagonal::shared_ptr odometryNoise =
                    noiseModel::Diagonal::Sigmas(Vector3(transl_std_dev , transl_std_dev, rot_std_dev));
            Pose2 odometry_offset_est(odom_est_loc_displ.x(), odom_est_loc_displ.y(), odom_est_angle_displ);
            graph_->add(BetweenFactor<Pose2>(dpg_nodes_.back().getNodeNumber(), new_node.getNodeNumber(), odometry_offset_est, odometryNoise));

            odom_only_estimates_.emplace_back(std::make_pair(prev_odom_loc_, prev_odom_angle_));
            odom_loc_at_last_laser_align_ = prev_odom_loc_;
            odom_angle_at_last_laser_align_ = prev_odom_angle_;

            // Add observation constraints
            updatePoseGraphObsConstraints(new_node);
        }

        // If we've gotten here, we should run DPG
        return true;
    }

    void DpgSLAM::updatePoseGraphObsConstraints(DpgNode &new_node) {

        DpgNode preceding_node = dpg_nodes_.back();

        ROS_INFO_STREAM("Preceding node pos " << preceding_node.getEstimatedPosition().first.x() << ", " << preceding_node.getEstimatedPosition().first.y() << ", " << preceding_node.getEstimatedPosition().second);

        // Add laser factor for previous pose and this node
        std::pair<std::pair<Vector2f, float>, Eigen::MatrixXd> successive_scan_offset =
                runIcp(preceding_node, new_node);
        addObservationConstraint(preceding_node.getNodeNumber(), new_node.getNodeNumber(), successive_scan_offset);

        // Add constraints for non-successive scans
        // TODO should we limit how many connections we make here (i.e. if we connect 2 and 8,
        //  don't also connect 3 and 8?)
        for (size_t i = 0; i < dpg_nodes_.size() - 1; i++) {
            DpgNode node = dpg_nodes_[i];
            if ((node.getEstimatedPosition().first - new_node.getEstimatedPosition().first).norm()
            <= pose_graph_parameters_.maximum_node_dist_scan_comparison_) {
                ROS_INFO_STREAM("Loop closing between node " << i << " at " <<
                node.getEstimatedPosition().first.x() << ", " << node.getEstimatedPosition().first.y() << ", " << node.getEstimatedPosition().second << " and node "
                << new_node.getNodeNumber() << " at " << new_node.getEstimatedPosition().first.x() << ", " <<
                new_node.getEstimatedPosition().first.y() << ", " << new_node.getEstimatedPosition().second);
                std::pair<std::pair<Vector2f, float>, Eigen::MatrixXd> non_successive_scan_offset =
                        runIcp(node, new_node);
                ROS_INFO_STREAM("Output transform " << non_successive_scan_offset.first.first.x() << ", " << non_successive_scan_offset.first.first.y() << ", " << non_successive_scan_offset.first.second);
                addObservationConstraint(node.getNodeNumber(), new_node.getNodeNumber(), non_successive_scan_offset);
            }
        }

        dpg_nodes_.push_back(new_node);

        // Get initial estimates for each node based on their current estimated position
        // With the exception of the new node, this will be the estimate from the last GTSAM update
        gtsam::Values init_estimates;
        for (size_t i = 0; i < dpg_nodes_.size(); i++) {
            // TODO should we do this each time or just add the estimate for a new node?
            // The former will converge faster, but the latter has less duplicate computation
            DpgNode node = dpg_nodes_[i];
            std::pair<Vector2f, float> est_pose = node.getEstimatedPosition();
            init_estimates.insert(node.getNodeNumber(), Pose2(est_pose.first.x(), est_pose.first.y(), est_pose.second));
        }

        // Optimize the trajectory and update the nodes' position estimates
        LevenbergMarquardtParams optimization_params;
        optimization_params.maxIterations = pose_graph_parameters_.gtsam_max_iterations_;
        // TODO do we need other params here?
        Values result = LevenbergMarquardtOptimizer(*graph_, init_estimates, optimization_params).optimize();
        init_estimates.print("Initial estimates");
        result.print("Results");

        for (DpgNode &dpg_node : dpg_nodes_) {

            // Node number is the key, so we'll access the results using that
            Pose2 estimated_pose = result.at<Pose2>(dpg_node.getNodeNumber());
            dpg_node.setPosition(Vector2f(estimated_pose.x(), estimated_pose.y()), estimated_pose.theta());
        }
    }

    void DpgSLAM::addObservationConstraint(const size_t &from_node_num, const size_t &to_node_num,
                                           std::pair<std::pair<Vector2f, float>, Eigen::MatrixXd> &constraint_info) {
        Pose2 factor_transl(constraint_info.first.first.x(), constraint_info.first.first.y(),
                            constraint_info.first.second);
        noiseModel::Gaussian::shared_ptr factor_noise = noiseModel::Gaussian::Covariance(constraint_info.second);
        ROS_INFO_STREAM("Adding constraint from node " << from_node_num << " to node " << to_node_num <<" factor " << factor_transl.x() << ", " << factor_transl.y() << ", " << factor_transl.theta());
        graph_->add(BetweenFactor<Pose2>(from_node_num, to_node_num, factor_transl, factor_noise));
    }

    std::pair<Vector2f, float> DpgSLAM::getLaserPositionRelativeToBaselink() {
        return std::make_pair(Vector2f(pose_graph_parameters_.laser_x_in_bl_frame_,
                                              pose_graph_parameters_.laser_y_in_bl_frame_),
                              pose_graph_parameters_.laser_orientation_rel_bl_frame_);
    }

    std::pair<std::pair<Vector2f, float>, Eigen::MatrixXd> DpgSLAM::runIcp(DpgNode &node_1, DpgNode &node_2) {

        std::pair<Vector2f, float> node_2_in_node_1 =
                math_utils::inverseTransformPoint(node_2.getEstimatedPosition().first,
                                                  node_2.getEstimatedPosition().second,
                                                  node_1.getEstimatedPosition().first,
                                                  node_1.getEstimatedPosition().second);
        Vector2f est_loc_displ = node_2_in_node_1.first;
        float est_angle_displ = node_2_in_node_1.second;
//        ROS_INFO_STREAM("Rotated displ est " << est_loc_displ.x() << ", " << est_loc_displ.y());
//        ROS_INFO_STREAM("Est angle displ " << est_angle_displ);

        Matrix4f transform_guess;
        transform_guess << cos(est_angle_displ), -sin(est_angle_displ), 0, est_loc_displ.x(),
        sin(est_angle_displ), cos(est_angle_displ), 0, est_loc_displ.y(),
        0, 0, 1, 0,
        0, 0, 0, 1;
        ROS_INFO_STREAM("Transform guess " << transform_guess);

        // TODO should we pre-transform node2 based on what we can estimate from odometry and then combine that with
        //  the ICP output (as an attempt to give it an initial guess since it can't seem to do that on its own)
        // or just let ICP figure it out? Going with the latter option for now, but this may not work well enough

        // Using XYZ clouds since PCL doesn't seem to have ICP support for 2D clouds
        // Just going to set z coord to 0
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

        // TODO are these in the right order or should they be swapped
        pcl::PointCloud<pcl::PointXYZ>::Ptr node_1_cloud = node_1.getCachedPointCloudFromNode(
                getLaserPositionRelativeToBaselink());
        pcl::PointCloud<pcl::PointXYZ>::Ptr node_2_cloud = node_2.getCachedPointCloudFromNode(
                getLaserPositionRelativeToBaselink());

        icp.setInputSource(node_2_cloud);
        icp.setInputTarget(node_1_cloud);

        // TODO set configs. Should we set any others?
        icp.setMaximumIterations(pose_graph_parameters_.icp_maximum_iterations_);
        icp.setTransformationEpsilon(pose_graph_parameters_.icp_maximum_transformation_epsilon_);
        icp.setMaxCorrespondenceDistance(pose_graph_parameters_.icp_max_correspondence_distance_);

        pcl::PointCloud<pcl::PointXYZ> icp_output_cloud;
        icp.align(icp_output_cloud, transform_guess);
        Eigen::Matrix4f est_transform = icp.getFinalTransformation();
        if (est_transform(2, 3) != 0.0) {
            // TODO may want to check equality within some small bound rather than exact equality
            ROS_WARN_STREAM("Z estimate for transform was not 0, but was " << est_transform(2, 3));
        }
        if (est_transform(2, 2) != 1.0) {
            // TODO may want to check equality within some small bound rather than exact equality
            ROS_WARN_STREAM("Bottom entry in rotation matrix was not 1, which suggests non-zero pitch or roll: " << est_transform);
        }

        // Estimate the covariance
        Eigen::MatrixXd est_cov;
        calculate_ICP_COV(node_2_cloud, node_1_cloud, est_transform, est_cov);

        // Extract the angle and translation from the transform estimate
        Eigen::Matrix2f rotation_mat = est_transform.block<2, 2>(0, 0);
        Eigen::Rotation2Df rotation_2d;
        rotation_2d.fromRotationMatrix(rotation_mat);
        float offset_angle = rotation_2d.angle();
        std::pair<Vector2f, float> transform = std::make_pair(
                Vector2f(est_transform(0, 3), est_transform(1, 3)), offset_angle);
        ROS_INFO_STREAM("Returning ICP");
        ROS_INFO_STREAM("Estimated transform " << est_transform);
        ROS_INFO_STREAM("converged? " << icp.hasConverged());
        ROS_INFO_STREAM("fitness score " << icp.getFitnessScore());
        return std::make_pair(transform, est_cov);
    }

    DpgNode DpgSLAM::createNewPassFirstNode(const std::vector<float> &ranges,
                                            const float &range_min,
                                            const float &range_max,
                                            const float &angle_min,
                                            const float &angle_max,
                                            const uint32_t &pass_number) {

        // Create a node that is the first in its pass. Estimate the pose to be at 0
        std::pair<Vector2f, float> est_pos = std::make_pair(Vector2f(0.0, 0.0), 0.0);
        uint32_t node_number = dpg_nodes_.size();
        return createNode(ranges, range_min, range_max, angle_min, angle_max, est_pos.first, est_pos.second,
                          pass_number, node_number);
    }

    DpgNode DpgSLAM::createRelativePositionedNode(const std::vector<float> &ranges,
                                                  const float &range_min,
                                                  const float &range_max,
                                                  const float &angle_min,
                                                  const float &angle_max,
                                                  const Vector2f &odom_displacement,
                                                  const float &odom_orientation_change,
                                                  const uint32_t &pass_number) {

        // Create a node that is not the first in its pass

        // Estimate the pose to be the previous node's pose plus the odometry displacement since the last node
        std::pair<Vector2f, float> prev_node_est_position;
        if (dpg_nodes_.empty()) {
            prev_node_est_position = std::make_pair(Vector2f(0, 0), 0.0);
        } else {
            prev_node_est_position = dpg_nodes_.back().getEstimatedPosition();
        }
        std::pair<Vector2f, float> est_pos = math_utils::transformPoint(odom_displacement,
                odom_orientation_change, prev_node_est_position.first, prev_node_est_position.second);
        ROS_INFO_STREAM("New node est pos " << est_pos.first.x() << ", " << est_pos.first.y() << ", " << est_pos.second);
        uint32_t node_number = dpg_nodes_.size();
        return createNode(ranges, range_min, range_max, angle_min, angle_max, est_pos.first, est_pos.second,
                          pass_number, node_number);
    }

    DpgNode DpgSLAM::createNode(const std::vector<float>& ranges,
                                const float &range_min,
                                const float &range_max,
                                const float &angle_min,
                                const float &angle_max,
                                const Vector2f &init_pos,
                                const float &init_orientation,
                                const uint32_t &pass_number,
                                const uint32_t &node_number) {
        float angle_inc = (angle_max - angle_min) / (ranges.size() - 1.0);
        std::vector<MeasurementPoint> measurement_points;
        uint8_t num_sectors = dpg_parameters_.num_sectors_;

        float points_in_sector = ((float) ranges.size()) / num_sectors;

        // Make a measurement point for each range reading within range and assign it a sector
        for (size_t i = 0; i < ranges.size(); i++) {
            // TODO should we exclude points outside of range_max? Verify this
            if (ranges[i] < range_max) {
                uint8_t sector_num = i / points_in_sector; // TODO is this correct?
                float angle = angle_inc * i + angle_min;
                measurement_points.emplace_back(angle, ranges[i], sector_num);
            }
        }

        Measurement measurement(num_sectors, measurement_points);

        return DpgNode(init_pos, init_orientation, pass_number, node_number, measurement);
    }

    void DpgSLAM::ObserveOdometry(const Vector2f& odom_loc,
                         const float odom_angle) {
        if (!odom_initialized_) {
            odom_initialized_ = true;
        }

        prev_odom_angle_ = odom_angle;
        prev_odom_loc_ = odom_loc;
    }

    void DpgSLAM::GetPose(Eigen::Vector2f* loc, float* angle) const {
        std::pair<Eigen::Vector2f, float> locInfo;
        if (dpg_nodes_.empty()) {
            Vector2f empty_vec(0, 0);
            locInfo = std::make_pair(empty_vec, 0);
        } else {
            locInfo = dpg_nodes_.back().getEstimatedPosition();
        }

        // Return the latest pose estimate of the robot plus whatever odometry has not yet been incorporated into the
        // trajectory.
        Eigen::Vector2f& loc_ = *loc;
        float& angle_ = *angle;

        Eigen::Vector2f unrotated_odom_est_loc_displ = prev_odom_loc_ - odom_loc_at_last_laser_align_;
        float odom_est_angle_displ = math_utils::AngleDiff(prev_odom_angle_, odom_angle_at_last_laser_align_);
        Eigen::Rotation2Df rotate(-1 * odom_angle_at_last_laser_align_);
        Vector2f odom_est_loc_displ = rotate * unrotated_odom_est_loc_displ;

        Rotation2Df eig_rotation(locInfo.second);
        Vector2f rotated_offset = eig_rotation * odom_est_loc_displ;

        loc_ = locInfo.first + rotated_offset;
        angle_ = locInfo.second + odom_est_angle_displ;
//        ROS_INFO_STREAM("Pose " << loc_.x() << ", " << loc_.y() << ", " << angle_);
    }

    std::vector<Vector2f> DpgSLAM::GetMap() {
        std::vector<Vector2f> map;

        // Reconstruct the map as a single aligned point cloud from all saved poses
        // and their respective scans.
        for (size_t i = 0; i < dpg_nodes_.size(); i++) {
            DpgNode node = dpg_nodes_[i];
            pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud = node.getCachedPointCloudFromNode(
                    getLaserPositionRelativeToBaselink());

            std::pair<Vector2f, float> node_pose = node.getEstimatedPosition();
            for (pcl::PointXYZ point : *point_cloud) {
                Vector2f bl_point(point.x, point.y);
                map.push_back(math_utils::transformPoint(bl_point, 0, node_pose.first, node_pose.second).first);
            }
        }
        return map;
    }

    bool DpgSLAM::shouldProcessLaser() {

        float angle_diff = math_utils::AngleDist(prev_odom_angle_, odom_angle_at_last_laser_align_);
        float loc_diff = (prev_odom_loc_ - odom_loc_at_last_laser_align_).norm();

        if ((loc_diff > pose_graph_parameters_.min_dist_between_nodes_)
        || (angle_diff > pose_graph_parameters_.min_angle_between_nodes_)) {
            return true;
        }

        // Check if odom has changed enough since last time we processed the laser
        return false;
    }
}