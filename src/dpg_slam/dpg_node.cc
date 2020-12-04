
#include "dpg_node.h"
#include "math_utils.h"

using Eigen::Vector2f;

namespace dpg_slam {
    pcl::PointCloud<pcl::PointXYZ>::Ptr DpgNode::getCachedPointCloudFromNode() {
        // TODO verify all of this - currently totally untested
        if (cached_point_cloud_ == NULL) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

            for (size_t i = 0; i < measurement_.getMeasurements().size(); i++) {
                MeasurementPoint point = measurement_.getMeasurements()[i];
                if (point.getLabel() == MAX_RANGE) {
                    continue;
                }
                Eigen::Vector2f cloud_point_in_bl_frame = math_utils::transformPoint(
                        point.getPointInLaserFrame(), 0.0, laser_pos_rel_base_link_.first,
                        laser_pos_rel_base_link_.second).first;
                cloud->push_back(pcl::PointXYZ(cloud_point_in_bl_frame.x(), cloud_point_in_bl_frame.y(), 0));
            }
            cached_point_cloud_ = cloud;
        }
        return cached_point_cloud_;
    }

    void DpgNode::deactivateIntersectingSectors(const std::vector<Eigen::Vector2f> &removed_points,
                                                const float &min_active_sectors) {
        if (!is_active_) {
            return;
        }

        std::pair<Vector2f, float> lidar_pos_in_map_frame =
                math_utils::transformPoint(laser_pos_rel_base_link_.first, laser_pos_rel_base_link_.second, node_loc_,
                                           node_orientation_);

        // Assuming we've already deactivated the sectors in this node that points that have been labeled removed

        // Turn off sectors that contain the removed points in the field of view

        // Get the points in the lidar frame and find their angle
        std::vector<std::pair<Vector2f, float>> points_in_polar_coords_rel_lidar;
        for (const Vector2f & point : removed_points) {
            Vector2f point_rel_lidar = math_utils::inverseTransformPoint(point, 0,
                                                                         lidar_pos_in_map_frame.first,
                                                                         lidar_pos_in_map_frame.second).first;
            if (point_rel_lidar.norm() > measurement_.getMaxRange()) {
                continue;
            }
            points_in_polar_coords_rel_lidar.emplace_back(point_rel_lidar, atan2(point_rel_lidar.y(),
                                                                                 point_rel_lidar.x()));
        }

        std::vector<MeasurementPoint> measurement_points = measurement_.getMeasurements();

        float angle_min = measurement_.getMeasurementRange().first;
        float angle_max = measurement_.getMeasurementRange().second;
        float angle_inc = measurement_.getAngleInc();
        float sector_size = (angle_max - angle_min) / measurement_.getNumSectors();

        for (const std::pair<Vector2f, float> &removed_point : points_in_polar_coords_rel_lidar) {
            float point_angle = removed_point.second;
            if ((point_angle > angle_max) || (point_angle < angle_min)) {
                continue;
            }

            // If the sector that the measurment would be in is already deactivated, then we don't need to do any more
            // with this point
            uint8_t sector_num = (point_angle - angle_min) / sector_size;
            if (!measurement_.isSectorActive(sector_num)) {
                break;
            }

            // Get the points from the scan that would be on either side of this point
            // If this point is closer than the closer one, then
            float approx_index = (point_angle - angle_min) / angle_inc;
            int index_floor = floor(approx_index);
            float fov_range = measurement_points[index_floor].getRange();

            // Use the closer range measurement to be conservative about deactivation
            if (index_floor < (measurement_points.size() - 1)) {
                fov_range = std::min(fov_range, measurement_points[index_floor + 1].getRange());
            }

            // If the removed point is closer than the measurement points, then it intersects the sector and we should
            // deactivate the sector
            if (fov_range > removed_point.first.norm()) {
                measurement_.deactivateSector(sector_num);
            }
        }

        if (min_active_sectors > measurement_.getPercentSectorsActive()) {
            setInactive();
        }
    }

}
