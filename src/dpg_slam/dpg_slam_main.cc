//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    slam-main.cc
\brief   Main entry point for slam
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <signal.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <termios.h>

#include <iostream>
#include <vector>
#include <memory>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/Localization2DMsg.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "gflags/gflags.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "cobot_msgs/CobotOdometryMsg.h"
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "ros/package.h"

// GTSam includes.
#include "gtsam/geometry/Pose2.h"
#include "gtsam/inference/Key.h"
#include "gtsam/slam/BetweenFactor.h"
#include "gtsam/slam/PriorFactor.h"
#include "gtsam/nonlinear/NonlinearFactorGraph.h"
#include "gtsam/nonlinear/GaussNewtonOptimizer.h"
#include "gtsam/nonlinear/Marginals.h"
#include "gtsam/nonlinear/Values.h"

#include "dpg_node.h"
#include "dpg_slam.h"

//#include "config_reader/config_reader.h"
//#include "shared/math/math_util.h"
//#include "shared/math/line2d.h"

//#include "slam.h"
//#include "vector_map/vector_map.h"
#include "visualization/visualization.h"
#include <std_msgs/Empty.h>

using amrl_msgs::VisualizationMsg;
//using geometry::line2f;
//using geometry::Line;
using gtsam::NonlinearFactorGraph;
//using math_util::DegToRad;
//using math_util::RadToDeg;
using ros::Time;
using std::cout;
using std::endl;
using std::string;
using std::vector;
using Eigen::Vector2f;
using visualization::ClearVisualizationMsg;
//using visualization::DrawArc;
//using visualization::DrawPoint;
//using visualization::DrawLine;
//using visualization::DrawParticle;

// Create command line arguements
DEFINE_string(laser_topic, "/scan", "Name of ROS topic for LIDAR data");
DEFINE_string(odom_topic, "/odom", "Name of ROS topic for odometry data");
DEFINE_string(cobot_odom_topic, "/Cobot/Odometry", "Name of ROS topic for cobot odometry data");
DEFINE_string(new_pass_topic, "/new_pass", "Name of ROS topic that when we've received a message indicates we're on a "
                                           "new pass");
DEFINE_bool(gtsam_test, false, "Run GTSam test");
DEFINE_bool(write_results, false, "Write grid results to file");

DECLARE_int32(v);

bool run_ = true;
std::unique_ptr<dpg_slam::DpgSLAM> slam_;
ros::Publisher visualization_publisher_;
ros::Publisher localization_publisher_;
ros::Publisher reoptimization_complete_pub_;
VisualizationMsg vis_msg_;
sensor_msgs::LaserScan last_laser_msg_;
nav_msgs::OccupancyGrid occ_grid_;
Vector2f prev_odom_;
float prev_angle_;

void InitializeMsgs() {
  std_msgs::Header header;
  header.frame_id = "map";
  header.seq = 0;

 vis_msg_ = visualization::NewVisualizationMessage("map", "slam");
}

void PublishMap() {
  static double t_last = 0;
  if (ros::Time::now().toSec() - t_last < 0.5) {
    // Rate-limit visualization.
    return;
  }
  t_last = ros::Time::now().toSec();
  vis_msg_.header.stamp = ros::Time::now();
  ClearVisualizationMsg(vis_msg_);

  const vector<Vector2f> map = slam_->GetMap();
  vector<Vector2f> active_static_points;
  vector<Vector2f> active_added_points;
  vector<Vector2f> dynamic_added_points;
  vector<Vector2f> dynamic_removed_points;
  vector<Vector2f> cells_to_plot;
  vector<Vector2f> submap_cells_to_plot;
  
  active_static_points = slam_->GetActiveStaticPoints(); 
  active_added_points = slam_->GetActiveAddedPoints();
  dynamic_added_points  = slam_->GetDynamicAddedPoints();
  dynamic_removed_points = slam_->GetDynamicRemovedPoints();
  cells_to_plot = slam_->getCellsToPlot();
  submap_cells_to_plot = slam_->getSubMapCellsToPlot();

//  printf("Map: %lu points\n", map.size());
  for (const Vector2f& p : map) {
    visualization::DrawPoint(p, 0xC0C0C0, vis_msg_);
  }
  
  const Vector2f activeMapOffset(25.0, 25.0);
  const Vector2f dynamicMapOffset(-25.0, -25.0);
  for (const Vector2f& p : active_static_points) {
    visualization::DrawPoint(p+activeMapOffset, 0x34b4eb, vis_msg_);
  }
  for (const Vector2f& p : active_added_points) {
    visualization::DrawPoint(p+activeMapOffset, 0xa569bd, vis_msg_);
  }
  for (const Vector2f& p : dynamic_added_points) {
    visualization::DrawPoint(p+dynamicMapOffset, 0x27ae60, vis_msg_);
  }
  for (const Vector2f& p : dynamic_removed_points) {
    visualization::DrawPoint(p+dynamicMapOffset, 0xc0392b, vis_msg_);
  }
  for (const Vector2f& p : cells_to_plot) {
    visualization::DrawPoint(p, 0x34b4eb, vis_msg_);
  }

  for (const Vector2f& p : submap_cells_to_plot) {
    visualization::DrawPoint(p, 0xeb1056, vis_msg_);
  }

  slam_->publishTrajectory(vis_msg_);
  visualization_publisher_.publish(vis_msg_);
}

void PublishPose() {
  Vector2f robot_loc(0, 0);
  float robot_angle(0);
  slam_->GetPose(&robot_loc, &robot_angle);
  amrl_msgs::Localization2DMsg localization_msg;
  localization_msg.pose.x = robot_loc.x();
  localization_msg.pose.y = robot_loc.y();
  localization_msg.pose.theta = robot_angle;
  localization_publisher_.publish(localization_msg);

  

}

void LaserCallback(const sensor_msgs::LaserScan& msg) {
  if (FLAGS_v > 0) {
    printf("Laser t=%f\n", msg.header.stamp.toSec());
  }
  last_laser_msg_ = msg;
  slam_->ObserveLaser(
      msg.ranges,
      msg.range_min,
      msg.range_max,
      msg.angle_min,
      msg.angle_max);
  PublishMap();
  PublishPose();
}

void OdometryCallback(const nav_msgs::Odometry& msg) {
  if (FLAGS_v > 0) {
    printf("Odometry t=%f\n", msg.header.stamp.toSec());
  }
  const Vector2f odom_loc(msg.pose.pose.position.x, msg.pose.pose.position.y);
  const float odom_angle =
      2.0 * atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
  slam_->ObserveOdometry(odom_loc, odom_angle);
}

void CobotOdometryCallback(const cobot_msgs::CobotOdometryMsg &cobot_msg) {
    if (FLAGS_v > 0) {
        printf("Odometry t=%f\n", cobot_msg.header.stamp.toSec());
    }

    // TODO It seems like the cobot odometry provides the relative movement since the last odom message, but this needs to be verified.
    std::pair<Vector2f, float> new_pose = math_utils::transformPoint(Vector2f(cobot_msg.dx, cobot_msg.dy),
                                                                     cobot_msg.dr, prev_odom_, prev_angle_);
    prev_odom_ = new_pose.first;
    prev_angle_ = new_pose.second;
    slam_->ObserveOdometry(new_pose.first, new_pose.second);
}

void NewPassCallback(const std_msgs::Empty &msg) {
    ROS_INFO_STREAM("New pass!");
    slam_->incrementPassNumber(FLAGS_write_results);
    PublishMap();
    reoptimization_complete_pub_.publish(std_msgs::Empty());
}

int gtsam_test(int argc, char** argv) {
  using namespace gtsam;
  // 1. Create a factor graph container and add factors to it
  NonlinearFactorGraph graph;

  // 2a. Add a prior on the first pose, setting it to the origin
  // A prior factor consists of a mean and a noise model (covariance matrix)
  auto priorNoise = noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.3, 0.3, 0.1));
  graph.add(PriorFactor<Pose2>(1, Pose2(0, 0, 0), priorNoise));

  // For simplicity, we will use the same noise model for odometry and loop closures
  auto model = noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.2, 0.2, 0.1));

  // 2b. Add odometry factors
  // Create odometry (Between) factors between consecutive poses
  graph.emplace_shared<BetweenFactor<Pose2> >(1, 2, Pose2(2, 0, 0), model);
  graph.emplace_shared<BetweenFactor<Pose2> >(2, 3, Pose2(2, 0, M_PI_2), model);
  graph.emplace_shared<BetweenFactor<Pose2> >(3, 4, Pose2(2, 0, M_PI_2), model);
  graph.emplace_shared<BetweenFactor<Pose2> >(4, 5, Pose2(2, 0, M_PI_2), model);

  // 2c. Add the loop closure constraint
  // This factor encodes the fact that we have returned to the same pose. In real systems,
  // these constraints may be identified in many ways, such as appearance-based techniques
  // with camera images. We will use another Between Factor to enforce this constraint:
  graph.emplace_shared<BetweenFactor<Pose2> >(5, 2, Pose2(2, 0, M_PI_2), model);
  graph.print("\nFactor Graph:\n");  // print

  // 3. Create the data structure to hold the initialEstimate estimate to the solution
  // For illustrative purposes, these have been deliberately set to incorrect values
  Values initialEstimate;
  initialEstimate.insert(1, Pose2(0.5, 0.0, 0.2));
  initialEstimate.insert(2, Pose2(2.3, 0.1, -0.2));
  initialEstimate.insert(3, Pose2(4.1, 0.1, M_PI_2));
  initialEstimate.insert(4, Pose2(4.0, 2.0, M_PI));
  initialEstimate.insert(5, Pose2(2.1, 2.1, -M_PI_2));
  initialEstimate.print("\nInitial Estimate:\n");  // print

  // 4. Optimize the initial values using a Gauss-Newton nonlinear optimizer
  // The optimizer accepts an optional set of configuration parameters,
  // controlling things like convergence criteria, the type of linear
  // system solver to use, and the amount of information displayed during
  // optimization. We will set a few parameters as a demonstration.
  GaussNewtonParams parameters;
  // Stop iterating once the change in error between steps is less than this value
  parameters.relativeErrorTol = 1e-5;
  // Do not perform more than N iteration steps
  parameters.maxIterations = 100;
  parameters.verbosity = GaussNewtonParams::Verbosity::VALUES;
  // Create the optimizer ...
  ROS_ERROR_STREAM("Optimizing");
  gtsam::GaussNewtonOptimizer optimizer(graph, initialEstimate, parameters);
  // ... and optimize
  gtsam::Values result = optimizer.optimize();
  result.print("Final Result:\n");

  // 5. Calculate and print marginal covariances for all variables
  cout.precision(3);
  gtsam::Marginals marginals(graph, result);
  cout << "x1 covariance:\n" << marginals.marginalCovariance(1) << endl;
  cout << "x2 covariance:\n" << marginals.marginalCovariance(2) << endl;
  cout << "x3 covariance:\n" << marginals.marginalCovariance(3) << endl;
  cout << "x4 covariance:\n" << marginals.marginalCovariance(4) << endl;
  cout << "x5 covariance:\n" << marginals.marginalCovariance(5) << endl;

  return 0;
}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);

  // TODO remove this
  ROS_INFO_STREAM("Running GTSAM demo to verify install");
  ROS_INFO_STREAM(gtsam_test(argc, argv));

  // Initialize ROS.
  ros::init(argc, argv, "dpg_slam");
  ros::NodeHandle n;

  // TODO actually initialize params
  dpg_slam::PoseGraphParameters pose_graph_params(n);
  dpg_slam::DpgParameters dpg_params(n);
  dpg_slam::VisualizationParams visualization_params;


    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%d-%m-%Y_%H:%M:%S");
    std::string time_str = oss.str();

  slam_ = std::make_unique<dpg_slam::DpgSLAM>(dpg_params, pose_graph_params, visualization_params, &n, vis_msg_, time_str);
  InitializeMsgs();

  visualization_publisher_ =
      n.advertise<VisualizationMsg>("visualization", 1);
  localization_publisher_ =
      n.advertise<amrl_msgs::Localization2DMsg>("localization", 1);
  reoptimization_complete_pub_ =
      n.advertise<std_msgs::Empty>("reoptimization_complete", 1);
  
  ros::Subscriber laser_sub = n.subscribe(
      FLAGS_laser_topic.c_str(),
      1,
      LaserCallback);
  ros::Subscriber odom_sub = n.subscribe(
      FLAGS_odom_topic.c_str(),
      1,
      OdometryCallback);
  ros::Subscriber new_pass_sub = n.subscribe(
          FLAGS_new_pass_topic.c_str(),
          1,
          NewPassCallback);

    ros::Subscriber cobot_odom_sub = n.subscribe(
            FLAGS_cobot_odom_topic.c_str(),
            1,
            CobotOdometryCallback);
  ROS_INFO_STREAM("Spinning");
  ros::spin();

  return 0;
}
