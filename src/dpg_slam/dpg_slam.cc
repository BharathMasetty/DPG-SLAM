

#include "dpg_slam.h"

namespace dpg_slam {

    DpgSLAM::DpgSLAM(const DpgParameters &dpg_parameters,
                     const PoseGraphParameters &pose_graph_parameters) : dpg_parameters_(dpg_parameters),
                     pose_graph_parameters_(pose_graph_parameters) {
        // TODO
    }

    void DpgSLAM::ObserveLaser(const std::vector<float>& ranges,
                      float range_min,
                      float range_max,
                      float angle_min,
                      float angle_max) {
        // TODO
    }

    // Observe new odometry-reported location.
    void DpgSLAM::ObserveOdometry(const Eigen::Vector2f& odom_loc,
                         const float odom_angle) {
        // TODO
    }

    bool DpgSLAM::shouldProcessLaser() {

        // Check if odom has changed enough since last time we processed the laser
        return false; // TODO fixed
    }
}