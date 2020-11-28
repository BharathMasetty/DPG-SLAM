
#include "dpg_node.h"
#include "math_utils.h"

namespace dpg_slam {
    pcl::PointCloud<pcl::PointXYZ>::Ptr DpgNode::getCachedPointCloudFromNode(
            const std::pair<Eigen::Vector2f, float> &laser_pos_rel_base_link) {
        // TODO verify all of this - currently totally untested
        if (cached_point_cloud_->empty()) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            cloud->width = measurement_.getMeasurements().size();
            cloud->resize(measurement_.getMeasurements().size());
            for (size_t i = 0; i < measurement_.getMeasurements().size(); i++) {
                MeasurementPoint point = measurement_.getMeasurements()[i];
                Eigen::Vector2f cloud_point_laser_frame(point.getRange() * cos(point.getAngle()), point.getRange() * sin(point.getAngle()));
                Eigen::Vector2f cloud_point_in_bl_frame = math_utils::transformPoint(
                        cloud_point_laser_frame, 0.0, laser_pos_rel_base_link.first,
                        laser_pos_rel_base_link.second).first;
                cloud->at(i).x = cloud_point_in_bl_frame.x();
                cloud->at(i).y = cloud_point_in_bl_frame.y();
                cloud->at(i).z = 0.0;
            }
            cached_point_cloud_ = cloud;
        }
        return cached_point_cloud_;
    }

}