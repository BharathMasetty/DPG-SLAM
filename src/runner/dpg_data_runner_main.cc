
#include <ros/ros.h>
#include "gflags/gflags.h"
#include <std_msgs/Empty.h>
#include "dpg_slam/parameters.h"
#include <mutex>
#include <condition_variable>

DEFINE_string(new_pass_topic, "/new_pass", "Name of ROS topic that when we've received a message indicates we're on a "
                                           "new pass");
DEFINE_bool(run_mit, false, "Run on the MIT dataset instead of GDC");

DEFINE_string(gdc_dataset_folder, "~/projects/data_DPG-SLAM/DPGSlamData", "Folder containing gdc bags");
DEFINE_string(mit_dataset_folder, "~/projects/data_DPG-SLAM/mit_dpg_slam/reading_room", "Folder containing MIT dataset bags");

const float kGdcLaserXInBLFrame = 0.2;
const float kGdcLaserYInBLFrame = 0.0;
const float kGdcLaserOrientationRelBLFrame = 0.0;

const float kMitLaserXInBLFrame = 0.2; // TODO set this
const float kMitLaserYInBLFrame = 0.0; // TODO set this
const float kMitLaserOrientationRelBLFrame = 0.0; // TODO set this

std::mutex reoptimization_done_mutex_;
std::condition_variable reoptimization_done_cv_;
bool dpg_ready_ = false;

ros::Publisher new_pass_pub_;

void ReoptimizationCompleteCallback(const std_msgs::Empty &empty_msg) {
    {
        std::lock_guard<std::mutex> lk(reoptimization_done_mutex_);
        dpg_ready_ = true;
    }
    reoptimization_done_cv_.notify_one();
}

void playRosbag(const std::string &rosbag_name, const float &playback_rate, const float &start_time, const float &duration=-1.0) {
    ROS_INFO_STREAM("Playing next rosbag");
    // TODO can we do this in the main thread or does it need to live in another thread
    std::string duration_string;
    if (duration > 0) {
        duration_string = " -u " + std::to_string(duration) + " ";
    }
    std::string run_cmd = "rosbag play " + rosbag_name + duration_string + " /Cobot/Laser:=/scan -r " + std::to_string(playback_rate) + " -s "
            + std::to_string(start_time) + " --topics /odom /scan /Cobot/Odometry /Cobot/Laser";
    ROS_INFO_STREAM("System result: " << system(run_cmd.c_str()));
    new_pass_pub_.publish(std_msgs::Empty());
    std::unique_lock<std::mutex> lk(reoptimization_done_mutex_);
    reoptimization_done_cv_.wait(lk, []{return dpg_ready_;});
    ros::Duration(1).sleep();
    dpg_ready_ = false;
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
    node_handle.setParam(dpg_slam::PoseGraphParameters::kMinAngleBetweenNodesParamName, M_PI / 6);
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
    node_handle.setParam(dpg_slam::PoseGraphParameters::kMinAngleBetweenNodesParamName, 0.3);
}

std::string getBagPath(const std::string &folder_name, const std::string &bag_name) {
    return folder_name + "/" + bag_name;
}

void runOnGdcRosBags() {
    // TODO consider adding duration to cut off the end of bags where we just go back and forth forever
    playRosbag(getBagPath(FLAGS_gdc_dataset_folder, "2020-11-25-09-32-29.bag"), 1.2, 13.0, 74);
    playRosbag(getBagPath(FLAGS_gdc_dataset_folder, "2020-11-25-09-35-41.bag"), 0.9, 7.0, 53.0);
    playRosbag(getBagPath(FLAGS_gdc_dataset_folder, "2020-11-25-09-42-01.bag"), 0.7, 0);
    playRosbag(getBagPath(FLAGS_gdc_dataset_folder, "2020-11-25-09-43-56.bag"), 0.6, 0);
    playRosbag(getBagPath(FLAGS_gdc_dataset_folder, "2020-11-25-09-48-06.bag"), 0.4, 0);
    playRosbag(getBagPath(FLAGS_gdc_dataset_folder, "2020-11-25-09-49-55.bag"), 0.3, 0);
    playRosbag(getBagPath(FLAGS_gdc_dataset_folder, "2020-11-25-09-52-37.bag"), 0.2, 0);
    playRosbag(getBagPath(FLAGS_gdc_dataset_folder, "2020-11-25-09-54-42.bag"), 0.2, 0);
    playRosbag(getBagPath(FLAGS_gdc_dataset_folder, "2020-11-25-09-56-35.bag"), 0.2, 0);
    playRosbag(getBagPath(FLAGS_gdc_dataset_folder, "2020-11-25-09-58-44.bag"), 0.2, 0);
    playRosbag(getBagPath(FLAGS_gdc_dataset_folder, "2020-11-25-10-00-28.bag"), 0.2, 0);
}

void runOnMitRosBags() {
    playRosbag(getBagPath(FLAGS_mit_dataset_folder, "run1__1_25_2009___18_50_b21.bag"), 1.2, 25);
    playRosbag(getBagPath(FLAGS_mit_dataset_folder, "run2__1_19_2009___2_29_b21.bag"), 0.8, 40, 300);
    playRosbag(getBagPath(FLAGS_mit_dataset_folder, "run3__3_19_2009___2_29_b21.bag"), 0.5,4, 270);
    playRosbag(getBagPath(FLAGS_mit_dataset_folder, "run4__1_25_2009___19_4_b21.bag"), 0.5,4, 300);
    playRosbag(getBagPath(FLAGS_mit_dataset_folder, "run5__3_19_2009___0_28_b21.bag"), 0.5,2,270);
    playRosbag(getBagPath(FLAGS_mit_dataset_folder, "run6__1_25_2009___19_49_b21.bag"), 0.5,2,310);
    playRosbag(getBagPath(FLAGS_mit_dataset_folder, "run7__1_25_2009___20_12_b21.bag"), 0.5,2,240);
    playRosbag(getBagPath(FLAGS_mit_dataset_folder, "run8__3_19_2009___1_8_b21.bag"), 0.5,2,270);
    playRosbag(getBagPath(FLAGS_mit_dataset_folder, "run9__3_19_2009___1_17_b21.bag"), 0.5,2,290);
    playRosbag(getBagPath(FLAGS_mit_dataset_folder, "run10__3_19_2009___1_28_b21.bag"), 0.50,2,280);
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
    ROS_INFO_STREAM("Set any override ROS params then Start DPG SLAM now");

    new_pass_pub_ = n.advertise<std_msgs::Empty>(FLAGS_new_pass_topic.c_str(), 1);

    while (new_pass_pub_.getNumSubscribers() == 0) {
        ros::Duration(0.1).sleep();
    }

    ros::Subscriber optimization_done_sub = n.subscribe(
            "reoptimization_complete",
            1,
            ReoptimizationCompleteCallback);

    ros::AsyncSpinner spinner(2); // TODO do we need this to be this high?
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
