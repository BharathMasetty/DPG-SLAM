
#include "dpg_slam.h"
#include <pcl/registration/icp.h>
#include "icp_cov/cov_func_point_to_point.h"
#include <visualization/visualization.h>
#include <ros/ros.h>
#include <unordered_set>
#include <gtsam/nonlinear/ISAM2.h>

using namespace gtsam;
using namespace Eigen;

namespace dpg_slam {

    DpgSLAM::DpgSLAM(const DpgParameters &dpg_parameters,
                     const PoseGraphParameters &pose_graph_parameters,
                     const VisualizationParams &visualization_parameters) : dpg_parameters_(dpg_parameters),
                     pose_graph_parameters_(pose_graph_parameters), visualization_params_(visualization_parameters),
                     pass_number_(kInitialPassNumber), odom_initialized_(false), first_scan_for_pass_(true),
                     cumulative_dist_since_laser_laser_align_(0.0) {
        graph_ = new NonlinearFactorGraph();
        isam_ = new ISAM2(); // TODO need params?
    }

    void DpgSLAM::incrementPassNumber() {
        pass_number_++;
        odom_initialized_ = false;
        first_scan_for_pass_ = true;
        current_pass_nodes_.clear();

        reoptimize();
        ROS_INFO_STREAM("Done reoptimizing");
    }

    void DpgSLAM::reoptimize() {
        delete graph_;
        delete isam_;
        isam_ = new ISAM2(); // TODO need params?
        graph_ = new NonlinearFactorGraph();
        uint8_t pass_num = -1;
        for (size_t i = 0; i < dpg_nodes_.size(); i++) {
            DpgNode curr_node = dpg_nodes_[i];
            if (curr_node.getPassNumber() != pass_num) {
                Pose2 position_prior(0.0, 0.0, 0.0);
                noiseModel::Diagonal::shared_ptr prior_noise =
                        noiseModel::Diagonal::Sigmas(Vector3(pose_graph_parameters_.new_pass_x_std_dev_,
                                                             pose_graph_parameters_.new_pass_y_std_dev_,
                                                             pose_graph_parameters_.new_pass_theta_std_dev_));
                graph_->add(PriorFactor<Pose2>(curr_node.getNodeNumber(), position_prior, prior_noise));
                pass_num = curr_node.getPassNumber();
            } else {

                // Odometry constraint.

                // Get the estimated position change since the last node due to odometry
                std::pair<Vector2f, float> relative_loc_latest_pose = math_utils::inverseTransformPoint(
                        odom_only_estimates_[i].first, odom_only_estimates_[i].second,
                        odom_only_estimates_[i - 1].first, odom_only_estimates_[i - 1].second);
                Vector2f odom_est_loc_displ = relative_loc_latest_pose.first;
                float odom_est_angle_displ = relative_loc_latest_pose.second;

                // Add an odometry constraint
                float transl_std_dev = (pose_graph_parameters_.motion_model_transl_error_from_transl_ * (odom_est_loc_displ.norm())) +
                                       (pose_graph_parameters_.motion_model_transl_error_from_rot_ * fabs(odom_est_angle_displ));
                float rot_std_dev = (pose_graph_parameters_.motion_model_rot_error_from_transl_ * (odom_est_loc_displ.norm())) +
                                    (pose_graph_parameters_.motion_model_rot_error_from_rot_ * fabs(odom_est_angle_displ));

                // TODO should x and y standard deviation be different?
                noiseModel::Diagonal::shared_ptr odometryNoise =
                        noiseModel::Diagonal::Sigmas(Vector3(transl_std_dev , transl_std_dev, rot_std_dev));
                Pose2 odometry_offset_est(odom_est_loc_displ.x(), odom_est_loc_displ.y(), odom_est_angle_displ);
                if (pose_graph_parameters_.odometry_constraints_) {
                    graph_->add(BetweenFactor<Pose2>(dpg_nodes_[i - 1].getNodeNumber(), curr_node.getNodeNumber(),
                                                     odometry_offset_est, odometryNoise));
                }

            }

            if (i == 0) {
                continue;
            }

            DpgNode prev_node = dpg_nodes_[i - 1];
            std::pair<std::pair<Vector2f, float>, Eigen::MatrixXd> successive_scan_offset;
            bool converged = runIcp(prev_node, curr_node, successive_scan_offset); // TODO should we do anything with output?
            if (!converged) {
                ROS_ERROR_STREAM("Successive scan alignment didn't converge. Consider changing this to drop that scan");
            }
            addObservationConstraint(prev_node.getNodeNumber(), curr_node.getNodeNumber(), successive_scan_offset);

            for (size_t j = 0; j < i - 1; j+= 1) {
                DpgNode loop_closure_node = dpg_nodes_[j];

                float node_dist = (loop_closure_node.getEstimatedPosition().first - curr_node.getEstimatedPosition().first).norm();
                float node_dist_threshold = (loop_closure_node.getPassNumber() == curr_node.getPassNumber()) ?
                        pose_graph_parameters_.maximum_node_dist_within_pass_scan_comparison_ : pose_graph_parameters_.maximum_node_dist_across_passes_scan_comparison_;

                if (node_dist <= node_dist_threshold) {

                    std::pair<std::pair<Vector2f, float>, Eigen::MatrixXd> non_successive_scan_offset;
                    if (runIcp(loop_closure_node, curr_node, non_successive_scan_offset)) {
                        addObservationConstraint(loop_closure_node.getNodeNumber(), curr_node.getNodeNumber(),
                                                 non_successive_scan_offset);
                    }
                }
            }

        }
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
        optimizeGraph(init_estimates);
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

	ROS_INFO_STREAM("DPG Execution Start");
	if (pass_number_ > 1)
		executeDPG();
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
                current_pass_nodes_.push_back(new_node);

                gtsam::Values init_estimate_for_new_node;
                init_estimate_for_new_node.insert(new_node.getNodeNumber(), Pose2(new_node.getEstimatedPosition().first.x(),
                                                                                  new_node.getEstimatedPosition().first.y(),
                                                                                  new_node.getEstimatedPosition().second));
                optimizeGraph(init_estimate_for_new_node);
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
            if (pose_graph_parameters_.odometry_constraints_) {
                graph_->add(BetweenFactor<Pose2>(dpg_nodes_.back().getNodeNumber(), new_node.getNodeNumber(), odometry_offset_est, odometryNoise));
            }

            odom_only_estimates_.emplace_back(std::make_pair(prev_odom_loc_, prev_odom_angle_));
            odom_loc_at_last_laser_align_ = prev_odom_loc_;
            odom_angle_at_last_laser_align_ = prev_odom_angle_;

            // Add observation constraints
            updatePoseGraphObsConstraints(new_node);
        }

        ROS_INFO_STREAM("Num edges " << graph_->size());
        ROS_INFO_STREAM("Num nodes " << graph_->keys().size());

        // If we've gotten here, we should run DPG
        return true;
    }

    void DpgSLAM::updatePoseGraphObsConstraints(DpgNode &new_node) {

        DpgNode preceding_node = dpg_nodes_.back();

//        ROS_INFO_STREAM("Preceding node pos " << preceding_node.getEstimatedPosition().first.x() << ", " << preceding_node.getEstimatedPosition().first.y() << ", " << preceding_node.getEstimatedPosition().second);

        // Add laser factor for previous pose and this node
        std::pair<std::pair<Vector2f, float>, Eigen::MatrixXd> successive_scan_offset;
        bool converged = runIcp(preceding_node, new_node, successive_scan_offset); // TODO should we do anything with output?
        if (!converged) {
            ROS_ERROR_STREAM("Successive scan alignment didn't converge. Consider changing this to drop that scan");
        }
        addObservationConstraint(preceding_node.getNodeNumber(), new_node.getNodeNumber(), successive_scan_offset);

        // Add constraints for non-successive scans
        // TODO should we limit how many connections we make here (i.e. if we connect 2 and 8,
        //  don't also connect 3 and 8?)

        if (pose_graph_parameters_.non_successive_scan_constraints_) {
            if (dpg_nodes_.size() > 1) {
                for (size_t i = 0; i < std::max((size_t) 0, dpg_nodes_.size() - 2); i++) {
                    DpgNode node = dpg_nodes_[i];

                    float node_dist = (node.getEstimatedPosition().first - preceding_node.getEstimatedPosition().first).norm();
                    float node_dist_threshold = (node.getPassNumber() == preceding_node.getPassNumber()) ?
                                                pose_graph_parameters_.maximum_node_dist_within_pass_scan_comparison_ : pose_graph_parameters_.maximum_node_dist_across_passes_scan_comparison_;

                    if (node_dist <= node_dist_threshold) {
//                        ROS_INFO_STREAM("Loop closing between node " << i << " at " <<
//                                                                     node.getEstimatedPosition().first.x() << ", "
//                                                                     << node.getEstimatedPosition().first.y() << ", "
//                                                                     << node.getEstimatedPosition().second
//                                                                     << " and node "
//                                                                     << preceding_node.getNodeNumber() << " at "
//                                                                     << preceding_node.getEstimatedPosition().first.x()
//                                                                     << ", "
//                                                                     << preceding_node.getEstimatedPosition().first.y()
//                                                                     << ", "
//                                                                     << preceding_node.getEstimatedPosition().second);
                        std::pair<std::pair<Vector2f, float>, Eigen::MatrixXd> non_successive_scan_offset;
                        if (runIcp(node, preceding_node, non_successive_scan_offset)) {
//                            ROS_INFO_STREAM("Output transform " << non_successive_scan_offset.first.first.x() << ", "
//                                                                << non_successive_scan_offset.first.first.y() << ", "
//                                                                << non_successive_scan_offset.first.second);
                            addObservationConstraint(node.getNodeNumber(), preceding_node.getNodeNumber(),
                                                     non_successive_scan_offset);
                        }
                    }
                }
            }
        }
        dpg_nodes_.push_back(new_node);
        current_pass_nodes_.push_back(new_node);

        gtsam::Values init_estimate_for_new_node;
        init_estimate_for_new_node.insert(new_node.getNodeNumber(), Pose2(new_node.getEstimatedPosition().first.x(),
                                                                          new_node.getEstimatedPosition().first.y(),
                                                                          new_node.getEstimatedPosition().second));
        optimizeGraph(init_estimate_for_new_node);
    }

    void DpgSLAM::optimizeGraph(gtsam::Values &new_node_init_estimates) {

        // Optimize the trajectory and update the nodes' position estimates
        // TODO do we need other params here?
        isam_->update(*graph_, new_node_init_estimates);
        Values result = isam_->calculateEstimate();

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
//        ROS_INFO_STREAM("Adding constraint from node " << from_node_num << " to node " << to_node_num <<" factor " << factor_transl.x() << ", " << factor_transl.y() << ", " << factor_transl.theta());
        graph_->add(BetweenFactor<Pose2>(from_node_num, to_node_num, factor_transl, factor_noise));
    }

    std::pair<Vector2f, float> DpgSLAM::getLaserPositionRelativeToBaselink() {
        return std::make_pair(Vector2f(pose_graph_parameters_.laser_x_in_bl_frame_,
                                              pose_graph_parameters_.laser_y_in_bl_frame_),
                              pose_graph_parameters_.laser_orientation_rel_bl_frame_);
    }

    void DpgSLAM::downsamplePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                                       const int &downsample_divisor,
                                       pcl::PointCloud<pcl::PointXYZ>::Ptr &downsampled_cloud) {

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        int cloud_count = 0;
        for (pcl::PointXYZ point : *input_cloud) {
            if ((cloud_count % downsample_divisor) == 0) {
                cloud->push_back(point);
            }
            cloud_count++;
        }
        downsampled_cloud = cloud;
    }

    bool DpgSLAM::runIcp(DpgNode &node_1, DpgNode &node_2, std::pair<std::pair<Vector2f, float>, Eigen::MatrixXd> &icp_results) {

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
//        ROS_INFO_STREAM("Transform guess " << transform_guess);

        // TODO should we pre-transform node2 based on what we can estimate from odometry and then combine that with
        //  the ICP output (as an attempt to give it an initial guess since it can't seem to do that on its own)
        // or just let ICP figure it out? Going with the latter option for now, but this may not work well enough

        // Using XYZ clouds since PCL doesn't seem to have ICP support for 2D clouds
        // Just going to set z coord to 0
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

//        ROS_INFO_STREAM("Ransac default iterations " << icp.getRANSACIterations());
//        ROS_INFO_STREAM("Ransac rejection " << icp.getRANSACOutlierRejectionThreshold());
//        ROS_INFO_STREAM("Use reciprocal correspondences " << icp.getUseReciprocalCorrespondences());

        // TODO are these in the right order or should they be swapped
        pcl::PointCloud<pcl::PointXYZ>::Ptr node_1_cloud = node_1.getCachedPointCloudFromNode();
        pcl::PointCloud<pcl::PointXYZ>::Ptr node_2_cloud = node_2.getCachedPointCloudFromNode();

        pcl::PointCloud<pcl::PointXYZ>::Ptr node_1_cloud_downsampled;
        pcl::PointCloud<pcl::PointXYZ>::Ptr node_2_cloud_downsampled;
        downsamplePointCloud(node_1_cloud, pose_graph_parameters_.downsample_icp_points_ratio_,
                             node_1_cloud_downsampled);
        downsamplePointCloud(node_2_cloud, pose_graph_parameters_.downsample_icp_points_ratio_,
                             node_2_cloud_downsampled);

        icp.setInputSource(node_2_cloud_downsampled);
        icp.setInputTarget(node_1_cloud_downsampled);

        // TODO set configs. Should we set any others?
        icp.setMaximumIterations(pose_graph_parameters_.icp_maximum_iterations_);
        icp.setTransformationEpsilon(pose_graph_parameters_.icp_maximum_transformation_epsilon_);
        icp.setMaxCorrespondenceDistance(pose_graph_parameters_.icp_max_correspondence_distance_);
        icp.setRANSACIterations(pose_graph_parameters_.ransac_iterations_);
        icp.setUseReciprocalCorrespondences(pose_graph_parameters_.icp_use_reciprocal_correspondences_);

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
            return false;
        }

        // Estimate the covariance
        Eigen::MatrixXd est_cov;
        calculate_ICP_COV(node_2_cloud, node_1_cloud, est_transform, est_cov, pose_graph_parameters_.laser_x_variance_,
                          pose_graph_parameters_.laser_y_variance_, pose_graph_parameters_.laser_theta_variance_);

        // Extract the angle and translation from the transform estimate
        Eigen::Matrix2f rotation_mat = est_transform.block<2, 2>(0, 0);
        Eigen::Rotation2Df rotation_2d;
        rotation_2d.fromRotationMatrix(rotation_mat);
        float offset_angle = rotation_2d.angle();
        std::pair<Vector2f, float> transform = std::make_pair(
                Vector2f(est_transform(0, 3), est_transform(1, 3)), offset_angle);
//        ROS_INFO_STREAM("Returning ICP");
//        ROS_INFO_STREAM("Estimated transform " << est_transform);
//        ROS_INFO_STREAM("converged? " << icp.hasConverged());
//        ROS_INFO_STREAM("fitness score " << icp.getFitnessScore());
        icp_results = std::make_pair(transform, est_cov);
        return icp.hasConverged();
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
            uint8_t sector_num = i / points_in_sector; // TODO is this correct?
            float angle = angle_inc * i + angle_min;
            measurement_points.emplace_back(angle, ranges[i], sector_num, range_max);
        }

        Measurement measurement(num_sectors, measurement_points, angle_min, angle_max, range_max, angle_inc);

        return DpgNode(init_pos, init_orientation, pass_number, node_number, measurement, getLaserPositionRelativeToBaselink());
    }

    void DpgSLAM::ObserveOdometry(const Vector2f& odom_loc,
                         const float odom_angle) {
        if (!odom_initialized_) {
            odom_initialized_ = true;
        }

        cumulative_dist_since_laser_laser_align_ += (odom_loc - prev_odom_loc_).norm();

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
        int point_num = 0;
        for (size_t i = 0; i < dpg_nodes_.size(); i++) {
            DpgNode node = dpg_nodes_[i];
            pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud = node.getCachedPointCloudFromNode();

            std::pair<Vector2f, float> node_pose = node.getEstimatedPosition();
            for (pcl::PointXYZ point : *point_cloud) {
                if ((point_num % visualization_params_.display_points_fraction_) == 0) {
                    Vector2f bl_point(point.x, point.y);
                    map.push_back(math_utils::transformPoint(bl_point, 0, node_pose.first, node_pose.second).first);
                }
                point_num++;
            }
        }
        return map;
    }

    bool DpgSLAM::shouldProcessLaser() {

        float angle_diff = math_utils::AngleDist(prev_odom_angle_, odom_angle_at_last_laser_align_);

        if ((cumulative_dist_since_laser_laser_align_ > pose_graph_parameters_.min_dist_between_nodes_)
        || (angle_diff > pose_graph_parameters_.min_angle_between_nodes_)) {
            cumulative_dist_since_laser_laser_align_ = 0.0;
            return true;
        }

        // Check if odom has changed enough since last time we processed the laser
        return false;
    }

    std::pair<std::vector<occupancyGrid>, occupancyGrid> DpgSLAM::computeLocalSubMap(){
    
        // Grid of current Pose chain
        std::vector<DpgNode> currPoseChain;
        uint32_t maxNumNodes = dpg_parameters_.current_pose_chain_len_;
        if (current_pass_nodes_.size() <= maxNumNodes){
            // Copy all nodes from current pass
            currPoseChain = current_pass_nodes_;
        }
        else {
            // Copy latest few from current pass
            currPoseChain.assign(current_pass_nodes_.end()-maxNumNodes, current_pass_nodes_.end());
        }

	ROS_INFO_STREAM("Current Pose Chain Created, size: " << currPoseChain.size());

	// create occupancy grids for nodes in current pose chain
	std::vector<occupancyGrid> currPoseChainGrids;
	for (uint32_t i=0; i< currPoseChain.size(); i++) {
	    DpgNode currNode = currPoseChain[i];
	    currPoseChainGrids.emplace_back(currNode, dpg_parameters_, pose_graph_parameters_);
	}
	
	ROS_INFO_STREAM("Occupancy Grids Created for Current Pose Chain Nodes, size: " << currPoseChainGrids.size());

        // Grid for FOV nodes
        occupancyGrid localSubMapGrid = getSubMapCoveringCurrPoseChain(currPoseChain, currPoseChainGrids);
	
	return std::make_pair(currPoseChainGrids, localSubMapGrid);
    }
    
    occupancyGrid DpgSLAM::getSubMapCoveringCurrPoseChain(const std::vector<DpgNode> &poseChain, 
		    						const std::vector<occupancyGrid> &poseChainGrids) {
	
	occupancyGrid subMapOccupancyGrid(dpg_parameters_, pose_graph_parameters_);
	std::unordered_set<cellKey, boost::hash<cellKey>> currentUncoveredCells;
	bool isLocalSubmapInitialized = false;

	// populate the list of uncovered cells
	for (uint32_t i=0; i<poseChainGrids.size(); i++) {
		std::unordered_map<cellKey, CellStatus, boost::hash<cellKey>> currNodeGridInfo = poseChainGrids[i].getGridInfo();
		for (const auto &kv : currNodeGridInfo) { 
			currentUncoveredCells.insert(kv.first);
		}
	}
	uint64_t totalUncoveredCells = currentUncoveredCells.size();
	uint64_t currentUncoveredCellsSize  = currentUncoveredCells.size();
	
	// compute submap that covers the currentUncoveredCells
	// Get max laser range
	float proximityThreshold = dpg_parameters_.distance_threshold_for_local_submap_nodes_;
	float coverageThreshold = dpg_parameters_.current_pose_graph_coverage_threshold_;

	for (uint32_t j=0; j<(dpg_nodes_.size()-current_pass_nodes_.size()); j++) { 
		
		DpgNode pastNode = dpg_nodes_[j];
		if (!pastNode.isNodeActive())
			continue;

                std::pair<Vector2f, float> pastNodeInfo = pastNode.getEstimatedPosition();
                Vector2f pastNodePosition = pastNodeInfo.first;
		// checking if the past node is close to any of the current nodes.
		bool isInProximity = false;
		for (uint32_t i=0; i<poseChain.size(); i++) {
			DpgNode currNode = poseChain[i];
                	std::pair<Vector2f, float> currentNodeInfo = currNode.getEstimatedPosition();
               		Vector2f currentNodePosition = currentNodeInfo.first;
			float distanceToPastNode = (currentNodePosition - pastNodePosition).norm();
			if (distanceToPastNode <= proximityThreshold)
				break;
		}		
		
		if (!isInProximity)
			continue;	

		if (!isLocalSubmapInitialized) {
			occupancyGrid tempCombinedGrid(dpg_nodes_[j], dpg_parameters_, pose_graph_parameters_);
			getUpdatedCoverageForCurrentPoseChain(currentUncoveredCells, tempCombinedGrid);
			uint32_t newCurrentUncoveredCellsSize  = currentUncoveredCells.size();
			if (newCurrentUncoveredCellsSize < currentUncoveredCellsSize) {
                        	subMapOccupancyGrid = tempCombinedGrid;
                        	isLocalSubmapInitialized = true;
                        	currentUncoveredCellsSize = newCurrentUncoveredCellsSize;
                	}
		} else{
			occupancyGrid tempGrid(dpg_nodes_[j], dpg_parameters_, pose_graph_parameters_);
                        occupancyGrid tempCombinedGrid(subMapOccupancyGrid, tempGrid, dpg_parameters_, pose_graph_parameters_);
			getUpdatedCoverageForCurrentPoseChain(currentUncoveredCells, tempCombinedGrid);
			uint32_t newCurrentUncoveredCellsSize  = currentUncoveredCells.size();
			if (newCurrentUncoveredCellsSize < currentUncoveredCellsSize) {
				subMapOccupancyGrid = tempCombinedGrid;
				currentUncoveredCellsSize = newCurrentUncoveredCellsSize;
			}
		}

		double currentCoverage = 1 - (double)currentUncoveredCellsSize/totalUncoveredCells;
		if (currentCoverage >= coverageThreshold)
			break;
	}
        
	return subMapOccupancyGrid;
    } 

    void DpgSLAM::getUpdatedCoverageForCurrentPoseChain(std::unordered_set<cellKey, boost::hash<cellKey>> &currentUncoveredCells,
		    				       const occupancyGrid &localSubMap) {
	std::unordered_map<cellKey, CellStatus, boost::hash<cellKey>> subMapGridInfo = localSubMap.getGridInfo();
	for (const auto& uncoveredCell : currentUncoveredCells) {
		if(subMapGridInfo.find(uncoveredCell) != subMapGridInfo.end()) {
			// if the key exists in local submap- erase from uncovered cells set.
			currentUncoveredCells.erase(uncoveredCell);
		}
	}	
    }

    void DpgSLAM::detectAndLabelChangesForCurrentPoseChain(const std::vector<occupancyGrid> &current_submap_occ_grids,
                                                           const occupancyGrid &local_submap_occ_grid,
                                                           std::vector<PointIdInfo> &removed_points) {
        std::vector<PointIdInfo> added_points;
        std::vector<PointIdInfo> removed_points_non_deduped;

        for (const occupancyGrid curr_node_grid : current_submap_occ_grids) {
            detectAndLabelChangesForCurrentNode(curr_node_grid, local_submap_occ_grid, added_points,
                                                removed_points_non_deduped);
        }

        for (const PointIdInfo &added_point : added_points) {
            dpg_nodes_[added_point.node_num_].setPointLabel(added_point.point_num_in_node_, ADDED);
        }

        std::unordered_map<uint64_t, std::unordered_set<uint64_t>> removed_by_node_num;
        for (const PointIdInfo &removed_point : removed_points_non_deduped) {
            if (removed_by_node_num.find(removed_point.node_num_) == removed_by_node_num.end()) {
                removed_by_node_num[removed_point.node_num_] = {};
            }
            removed_by_node_num[removed_point.node_num_].insert(removed_point.point_num_in_node_);
        }

        for (const auto &removed_for_node : removed_by_node_num) {
            for (const uint64_t &removed_index : removed_for_node.second) {
                dpg_nodes_[removed_index].setPointLabel(removed_index, REMOVED);
                removed_points.emplace_back(PointIdInfo(removed_for_node.first, removed_index));
            }
        }
    }

     void DpgSLAM::detectAndLabelChangesForCurrentNode(
            const occupancyGrid &current_node_occ_grid,
            const occupancyGrid &local_submap_occ_grid,
            std::vector<PointIdInfo> &added_points,
            std::vector<PointIdInfo> &removed_points) {
		
	 std::vector<PointIdInfo> added_points_at_current_node;
	 std::vector<PointIdInfo> removed_points_at_current_node;
	 bool commitChanges = false;
         
	 for (const auto &occ_grid_entry : current_node_occ_grid.getGridInfo()) {
             CellStatus local_submap_occupancy = local_submap_occ_grid.getCellStatus(occ_grid_entry.first);
             if ((occ_grid_entry.second == OCCUPIED) && (local_submap_occupancy == FREE)) {
                   std::vector<PointIdInfo> new_added = current_node_occ_grid.getPointsInOccCell(occ_grid_entry.first);
                   added_points_at_current_node.insert(added_points_at_current_node.end(), new_added.begin(), new_added.end());
		   ROS_INFO_STREAM("Added");
            	} else if ((occ_grid_entry.second == FREE) && (local_submap_occupancy == OCCUPIED)) {
                   std::vector<PointIdInfo> new_removed = local_submap_occ_grid.getPointsInOccCell(occ_grid_entry.first);
                   removed_points_at_current_node.insert(removed_points_at_current_node.end(), new_removed.begin(), new_removed.end());
		   ROS_INFO_STREAM("Removed");
            	}
            }
 	 
	 // if newly detected changes at node are non zero, check the score.
	 if (added_points_at_current_node.size()+removed_points_at_current_node.size() > 0) {
	 	commitChanges = computeBinScoreAndCommitLabelsForNode(added_points_at_current_node,
									   removed_points_at_current_node);
	 }
	 if (!commitChanges) {
		return;
	 }

	 // commit changes if we get here.
	 added_points.insert(added_points.end(), added_points_at_current_node.begin(), added_points_at_current_node.end());
	 removed_points.insert(removed_points.end(), removed_points_at_current_node.begin(), removed_points_at_current_node.end());
    }

    bool DpgSLAM::computeBinScoreAndCommitLabelsForNode(const std::vector<PointIdInfo> &added_points,
		    					const std::vector<PointIdInfo> &removed_points) {

         uint16_t totalBins = dpg_parameters_.num_bins_for_change_detection_;
	 double change_threshold = dpg_parameters_.delta_change_threshold_;
	 std::unordered_set<uint16_t> changedBins;
	 bool commitChanges = false;
	 double current_changed_ratio = 0.0;
	 std::vector<PointIdInfo> changedPoints = added_points;
	 changedPoints.insert(changedPoints.end(), removed_points.begin(), removed_points.end());

	 for(const PointIdInfo& point : changedPoints) {
		DpgNode node = dpg_nodes_[point.node_num_];
                Measurement MeasurementOfPoint = node.getMeasurement();
		std::vector<MeasurementPoint> ranges = MeasurementOfPoint.getMeasurements();
		std::pair<float, float> angleRanges = MeasurementOfPoint.getMeasurementRange();
                float angle = ranges[point.point_num_in_node_].getAngle();
		uint32_t binNumber = round(totalBins*(angle-angleRanges.first)/(angleRanges.second-angleRanges.first));
		changedBins.insert(binNumber);
		current_changed_ratio = changedBins.size()/totalBins;
		if (current_changed_ratio >= change_threshold) {
			commitChanges = true;
			break;
		}	
	 }
	 return commitChanges;
    }

    void DpgSLAM::getActiveAndDynamicMapPoints(std::vector<Vector2f> &active_static_points, std::vector<Vector2f> &active_added_points,
                                               std::vector<Vector2f> &dynamic_removed_points, std::vector<Vector2f> &dynamic_added_points) {
        for (const DpgNode &node : dpg_nodes_) {
            std::pair<Vector2f, float> lidar_pose_in_map = math_utils::transformPoint(getLaserPositionRelativeToBaselink().first, getLaserPositionRelativeToBaselink().second,
                                                                                      node.getEstimatedPosition().first, node.getEstimatedPosition().second);

            // TODO do we want to subsample? Including every point will be a LOT
            for (const MeasurementPoint &point : node.getMeasurement().getMeasurements()) {
                PointLabel label = point.getLabel();
                if ((label == NOT_YET_LABELED) || (label == MAX_RANGE)) {
                    continue;
                }
                Vector2f point_pos = math_utils::transformPoint(point.getPointInLaserFrame(), 0, lidar_pose_in_map.first, lidar_pose_in_map.second).first;
                if (node.isActive() && node.getMeasurement().isSectorActive(point.getSectorNum())) {
                    if (label == STATIC) {
                        active_static_points.emplace_back(point_pos);
                    } else if (label == ADDED) {
                        active_added_points.emplace_back(point_pos);
                    }
                }
                if (label == ADDED) {
                    dynamic_added_points.emplace_back(point_pos);
                } else if (label == REMOVED) {
                    dynamic_removed_points.emplace_back(point_pos);
                }
            }
        }
	ROS_INFO_STREAM("Active static Map Size: " << active_static_points.size());
	ROS_INFO_STREAM("Active added Map Size: " << active_added_points.size());
	ROS_INFO_STREAM("Dynamic added Map Size: " << dynamic_added_points.size());
	ROS_INFO_STREAM("Dynamic removed Map Size: " << dynamic_removed_points.size());
    }

    void DpgSLAM::executeDPG(){
	// Should we do this only once??
	std::pair<std::vector<occupancyGrid>, occupancyGrid> Grids = computeLocalSubMap();
	ROS_INFO_STREAM("Occupancy Grids Created");
	std::vector<occupancyGrid> currentPoseChainGrids = Grids.first;
	occupancyGrid localSubMapGrid = Grids.second;
	std::vector<PointIdInfo> removedPoints;
	detectAndLabelChangesForCurrentPoseChain(currentPoseChainGrids, localSubMapGrid, removedPoints);
	ROS_INFO_STREAM("Changes are Detected and Updated");
	updateNodesAndSectorStatus(removedPoints);
	ROS_INFO_STREAM("Deactivated sectors intersecting with removed points");
	// Updating map points.
	std::vector<Vector2f> active_static_points;
       	std::vector<Vector2f> active_added_points;
        std::vector<Vector2f> dynamic_removed_points; 
	std::vector<Vector2f> dynamic_added_points;
	getActiveAndDynamicMapPoints(active_static_points, active_added_points, dynamic_removed_points, dynamic_added_points);
	active_static_points_ = active_static_points;
	active_added_points_ = active_added_points;
	dynamic_removed_points_ = dynamic_removed_points;
	dynamic_added_points_ = dynamic_added_points;
    }

    void DpgSLAM::updateNodesAndSectorStatus(const std::vector<PointIdInfo> &removedPoints) {
   		
     	   // gather removed points in cartisian frame
	   std::vector<Vector2f> removedVector;
	   for(const PointIdInfo& point : removedPoints){
	   	DpgNode node = dpg_nodes_[point.node_num_];
		std::pair<Eigen::Vector2f, float> NodeInfo = node.getEstimatedPosition();
		Vector2f NodeLocation = NodeInfo.first;
		float NodeAngle = NodeInfo.second;
		std::pair<Vector2f, float> lidar_pose_in_map = math_utils::transformPoint(getLaserPositionRelativeToBaselink().first, getLaserPositionRelativeToBaselink().second,
                                                                                     	NodeLocation, NodeAngle);
		std::vector<MeasurementPoint> ranges = node.getMeasurement().getMeasurements();
		float range = ranges[point.point_num_in_node_].getRange();
		float angle = ranges[point.point_num_in_node_].getAngle();
		Vector2f PointInLaserFrame(range*cos(angle), range*sin(angle));
		Vector2f pointInMapFrame = math_utils::transformPoint(PointInLaserFrame, 0,  lidar_pose_in_map.first, lidar_pose_in_map.second).first; 
		removedVector.push_back(pointInMapFrame);
				

	   }

	   float min_frac_sectors_active = dpg_parameters_.minimum_percent_active_sectors_; 
	   for (uint32_t i=0; i<(dpg_nodes_.size()-current_pass_nodes_.size()); i++) {
	   	dpg_nodes_[i].deactivateIntersectingSectors(removedVector, min_frac_sectors_active); 	
	   } 
    }

    void occupancyGrid::calculateOccupancyGrid(){
        // Filling up the occupancy grid for each node.
        for(uint32_t i=0; i<Nodes_.size(); i++){
            DpgNode node = Nodes_[i];
            if (include_inactive_ || node.isNodeActive()) {
                convertLaserRangeToCellKey(node);
	    }
	}
    }

    cellKey occupancyGrid::convertToKeyForm(const Eigen::Vector2f& loc) const {
        int key_form_x = round(loc.x() / dpg_parameters_.occ_grid_resolution_);
            int key_form_y = round(loc.y() / dpg_parameters_.occ_grid_resolution_);

        cellKey cell_loc = std::make_pair(key_form_x, key_form_y);
        return cell_loc;
    }

    void occupancyGrid::combineOccupancyGrids(const occupancyGrid &grid1, const occupancyGrid &grid2) {
   		 
   	gridInfo = grid1.gridInfo;	
    	occupied_cell_info_ = grid1.occupied_cell_info_;
	std::unordered_map<cellKey, OccupiedCellInfo, boost::hash<cellKey>> occupied_cell_info2 = grid2.occupied_cell_info_;
	
	// Insert elements of second grid appropriately.
	for (const auto &occ_grid_entry : grid2.gridInfo) {
		cellKey key = occ_grid_entry.first;
		CellStatus value = occ_grid_entry.second;
		 OccupiedCellInfo cellInfoValue = occupied_cell_info2[key];
		// Returns false if key is common so we can handle it seperately.
		bool isNewCell = gridInfo.emplace(key, value).second;
		occupied_cell_info_.emplace(key, cellInfoValue);
		// Handle common cells by prioritizing OCCUPIED
		if (!isNewCell)	{
			if (value == OCCUPIED){
				gridInfo[key] = value;
				std::vector<PointIdInfo> points = occupied_cell_info_[key].points_;
				points.insert(points.end(), cellInfoValue.points_.begin(), cellInfoValue.points_.end());
				occupied_cell_info_[key].points_ = points;
			}
		}
	
	}
    }

    void occupancyGrid::convertLaserRangeToCellKey(const DpgNode& node) { 
        //Returns occupied Cells in active sectors of the node.
        DpgNode node_ = node;
        Measurement scanMeasurementAtNode = node_.getMeasurement();
        std::vector<MeasurementPoint> measurementsAtNode = scanMeasurementAtNode.getMeasurements();
        std::pair<Eigen::Vector2f, float> nodeLocInfo = node_.getEstimatedPosition();
        Eigen::Vector2f nodePosition = nodeLocInfo.first;
        float nodeAngle = nodeLocInfo.second;
        std::vector<cellKey> occupiedCells;
        std::vector<cellKey> unoccupiedCells;
        float LaserX = pose_graph_parameters_.laser_x_in_bl_frame_;
        float LaserY = pose_graph_parameters_.laser_y_in_bl_frame_;

        std::pair<Vector2f, float> LaserInMapFrame =
                math_utils::transformPoint(Vector2f(LaserX, LaserY),
                                           pose_graph_parameters_.laser_orientation_rel_bl_frame_,
                                           nodePosition, nodeAngle);

        // Assuming that the size of measurements_ vector at a node and what we get from getCachedPointCloud are the same
        for (uint32_t i =0; i<measurementsAtNode.size(); i++) {
            MeasurementPoint point = measurementsAtNode[i];
            Vector2f scanPoint = math_utils::transformPoint(point.getPointInLaserFrame(), 0, LaserInMapFrame.first, LaserInMapFrame.second).first;
            uint8_t sector_num = point.getSectorNum();

            if ((include_inactive_) || (scanMeasurementAtNode.isSectorActive(sector_num))) {
                if ((point.getLabel() == MAX_RANGE) || (include_static_ && (point.getLabel() == PointLabel::STATIC)) ||
                    (include_added_ && (point.getLabel() == PointLabel::ADDED)) || (point.getLabel() == PointLabel::REMOVED)) {
                    Eigen::Vector2f pointLocationInMapFrame(
                            math_utils::transformPoint(scanPoint, 0, nodePosition, nodeAngle).first);
                    cellKey cell = convertToKeyForm(pointLocationInMapFrame);
                    max_cell_x_ = std::max(cell.first, max_cell_x_);
                    min_cell_x_ = std::min(cell.first, min_cell_x_);
                    min_cell_y_ = std::min(cell.second, min_cell_y_);
                    max_cell_y_ = std::max(cell.second, max_cell_y_);

                    if (point.getLabel() != MAX_RANGE) {

                        occupied_cell_info_[cell].points_.emplace_back(node.getNodeNumber(), i);
                        occupiedCells.push_back(cell);
                    }

                    float range = point.getRange();

                    // Get Laser Line
                    std::vector<cellKey> emptyCellsbetweenLaserAndPoint = getIntermediateFreeCellsInFOV(LaserInMapFrame.first,
                                                                                                        pointLocationInMapFrame,
                                                                                                        range);
                    unoccupiedCells.insert(unoccupiedCells.end(), emptyCellsbetweenLaserAndPoint.begin(),
                                           emptyCellsbetweenLaserAndPoint.end());
                }
            }
        }

        setFreeCells(unoccupiedCells);
        setOccupiedCells(occupiedCells);
    }

    void occupancyGrid::setOccupiedCells(const std::vector<cellKey> &occupiedCells) {
        for (uint32_t i=0;i<occupiedCells.size();i++) {
            gridInfo[occupiedCells[i]] = OCCUPIED;
        }
    }
    
    void occupancyGrid::setFreeCells(const std::vector<cellKey> &freeCells) {
        for (uint32_t i=0;i<freeCells.size();i++) {
            // Only allow to set FREE of it is not OCCUPIED before

            if (gridInfo[freeCells[i]] != OCCUPIED){
                gridInfo[freeCells[i]] = FREE;
            }
        }
    }

    void occupancyGrid::toOccGridMsg(nav_msgs::OccupancyGrid &occ_grid_msg, bool distinguish_free) {
        occ_grid_msg.info.resolution = dpg_parameters_.occ_grid_resolution_;
        occ_grid_msg.info.width = max_cell_x_ - min_cell_x_;
        occ_grid_msg.info.height = max_cell_y_ - min_cell_y_;

        occ_grid_msg.info.origin.orientation.w = 1;
        occ_grid_msg.info.origin.position.x = min_cell_x_ * occ_grid_msg.info.resolution;
        occ_grid_msg.info.origin.position.y = min_cell_y_ * occ_grid_msg.info.resolution;

        occ_grid_msg.header.frame_id = "map";


        occ_grid_msg.data.resize(occ_grid_msg.info.width * occ_grid_msg.info.height);
        for (size_t i = 0; i < occ_grid_msg.data.size(); i++) {
            occ_grid_msg.data[i] = -1;
        }

        for (const auto &cell_info : gridInfo) {
            cellKey cell_pos = cell_info.first;
            int data_index = (cell_pos.first - min_cell_x_) + (occ_grid_msg.info.width * (cell_pos.second - min_cell_y_));
            if (cell_info.second == OCCUPIED) {
                occ_grid_msg.data[data_index] = 100;
            } else if ((cell_info.second == FREE) && (distinguish_free)) {
                occ_grid_msg.data[data_index] = 0;
            }
        }
    }

    std::vector<cellKey> occupancyGrid::getIntermediateFreeCellsInFOV(const Eigen::Vector2f &LaserLoc, const Eigen::Vector2f &scanLoc, const float& range) {
        /* Dividing the range line into bins and calculating the intermediate points on the
         * scan line using the parametric line equation. The we identify the cellKey for the
         * intermediate point to get add it to the list of unoccupied cells.
         *
         * NOTE: There might be some repetitions/missing in the intermediate cells found this w
         * depending on the resoution. Better to have smaller bins but not too small.
         */

        std::vector<cellKey> unOccupiedCells;
        float startX = LaserLoc.x();
        float startY = LaserLoc.y();
        float endX  = scanLoc.x();
        float endY = scanLoc.y();
        // Might need to alter this resolution
        uint32_t num_bins = round(range/(dpg_parameters_.occ_grid_resolution_));
        float increment = 1.0/num_bins;
        for (float t=0.0; t<1.0; t=t+increment){
            Vector2f intermediatePoint((1-t)*startX + t*endX, (1-t)*startY + t*endY);
            cellKey intermediateCell = convertToKeyForm(intermediatePoint);
            unOccupiedCells.push_back(intermediateCell);
        }
        return unOccupiedCells;
    }

}
