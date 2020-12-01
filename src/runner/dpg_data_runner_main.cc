
#include <ros/ros.h>
#include "gflags/gflags.h"
#include <std_msgs/Empty.h>
#include "dpg_slam/parameters.h"

DEFINE_string(new_pass_topic, "/new_pass", "Name of ROS topic that when we've received a message indicates we're on a "
                                           "new pass");
DEFINE_bool(run_mit, false, "Run on the MIT dataset instead of GDC");

DEFINE_string(gdc_dataset_folder, "~/projects/data_DPG-SLAM/DPGSlamData", "Folder containing gdc bags");
DEFINE_string(mit_dataset_folder, "", "Folder containing MIT dataset bags");

const float kGdcLaserXInBLFrame = 0.2;
const float kGdcLaserYInBLFrame = 0.0;
const float kGdcLaserOrientationRelBLFrame = 0.0;

const float kMitLaserXInBLFrame = 0.2; // TODO set this
const float kMitLaserYInBLFrame = 0.0; // TODO set this
const float kMitLaserOrientationRelBLFrame = 0.0; // TODO set this

ros::Publisher new_pass_pub_;

void playRosbag(const std::string &rosbag_name, const float &playback_rate, const float &start_time, const float &duration=-1.0) {
    // TODO can we do this in the main thread or does it need to live in another thread
    std::string duration_string;
    if (duration > 0) {
        duration_string = " -u " + std::to_string(duration) + " ";
    }
    std::string run_cmd = "rosbag play " + rosbag_name + duration_string + " -r " + std::to_string(playback_rate) + " -s "
            + std::to_string(start_time) + " --topics /odom /scan ";
    ROS_INFO_STREAM("System result: " << system(run_cmd.c_str()));
    new_pass_pub_.publish(std_msgs::Empty());
    ros::Duration(2).sleep();
    // TODO publish new pass message and sleep briefly
}

/**
 * Set the parameters specific to the GDC data.
 *
 * Need to make sure this overrides any set in the MIT config (or that those are deleted before rerunning the script
 * with GDC).
 *
 * @param node_handle Node handle used for setting params.
 */
void setGdcRosParams(ros::NodeHandle &node_handle) {

    node_handle.setParam(dpg_slam::PoseGraphParameters::kLaserXInBLFrameParamName, kGdcLaserXInBLFrame);
    node_handle.setParam(dpg_slam::PoseGraphParameters::kLaserYInBLFrameParamName, kGdcLaserYInBLFrame);
    node_handle.setParam(dpg_slam::PoseGraphParameters::kLaserOrientationInBLFrameParamName,
                         kGdcLaserOrientationRelBLFrame);
}

/**
 * Set the parameters specific to the MIT data.
 *
 * Need to make sure this overrides any set in the GDC config (or that those are deleted before rerunning the script
 * with MIT).
 *
 * @param node_handle Node handle used for setting params.
 */
void setMitRosParams(ros::NodeHandle &node_handle) {

    node_handle.setParam(dpg_slam::PoseGraphParameters::kLaserXInBLFrameParamName, kMitLaserXInBLFrame);
    node_handle.setParam(dpg_slam::PoseGraphParameters::kLaserYInBLFrameParamName, kMitLaserYInBLFrame);
    node_handle.setParam(dpg_slam::PoseGraphParameters::kLaserOrientationInBLFrameParamName,
                         kMitLaserOrientationRelBLFrame);
}

std::string getBagPath(const std::string &folder_name, const std::string &bag_name) {
    return folder_name + "/" + bag_name;
}

void runOnGdcRosBags() {
    // TODO consider adding duration to cut off the end of bags where we just go back and forth forever
    playRosbag(getBagPath(FLAGS_gdc_dataset_folder, "2020-11-25-09-32-29.bag"), 0.5, 13.0, 74.0);
    playRosbag(getBagPath(FLAGS_gdc_dataset_folder, "2020-11-25-09-35-41.bag"), 0.3, 7.0, 53.0);
    // TODO add other files
}

void runOnMitRosBags() {
    // TODO
}

int main(int argc, char** argv) {
    google::ParseCommandLineFlags(&argc, &argv, false);

    // Initialize ROS.
    ros::init(argc, argv, "gdc_data_runner");
    ros::NodeHandle n;

    bool run_mit = FLAGS_run_mit;

    if (run_mit) {
        ROS_INFO_STREAM("Setting MIT-specific params");
        setMitRosParams(n);
    } else {
        ROS_INFO_STREAM("Setting GDC-specific params");
        setGdcRosParams(n);
    }
    ROS_INFO_STREAM("Start DPG SLAM now");

    new_pass_pub_ = n.advertise<std_msgs::Empty>(FLAGS_new_pass_topic.c_str(), 1);

    while (new_pass_pub_.getNumSubscribers() == 0) {
        ros::Duration(0.1).sleep();
    }

    ros::AsyncSpinner spinner(5); // TODO do we need this to be this high?
    spinner.start();
    if (run_mit) {
        ROS_INFO_STREAM("Run on MIT dataset");
        runOnMitRosBags();
    } else {
        ROS_INFO_STREAM("Run on GDC dataset");
        runOnGdcRosBags();
    }

    return 0;
}