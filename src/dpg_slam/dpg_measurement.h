//
// Created by amanda on 11/25/20.
//

#pragma once

#include <ros/ros.h>
#include <unordered_map>
#include <vector>

namespace dpg_slam {

    /**
     * Label for the points in a scan indicating if it is static (never been missing in an observation),
     * added (is in current version of map, but wasn't always there), removed
     * (was at one point in map, but isn't in current version), or not_yet_labeled
     * (when we haven't yet compared the points).
     *
     * TODO - should we just default to static? When do the labels get changed?
     */
    enum PointLabel {STATIC, ADDED, REMOVED, NOT_YET_LABELED};

    /**
     * This class represents an individual point from a laser scan.
     */
    class MeasurementPoint {
    public:

        // TODO depending on how this is accessed, it might be more efficient to store each component in the measurement
        //  class in a vector and link them by ID, but for now, defining it like this to prioritize clarity over
        // efficiency

        /**
         * Constructor.
         *
         * @param angle         Angle of the laser scan relative to the lidar.
         * @param range         Range measurement.
         * @param sector_num    Sector number.
         */
        MeasurementPoint(const float &angle, const float &range, const uint8_t &sector_num) : angle_(angle),
        range_(range), label_(NOT_YET_LABELED), sector_num_(sector_num) {

        }

        /**
         * Set the label for the point.
         *
         * Should not be NOT_YET_LABELED
         *
         * @param new_label New label that the point should have
         */
        void setLabel(const PointLabel &new_label) {
            label_ = new_label;
        }

        /**
         * Get the angle of the ray that observed this point relative to the LiDAR frame.
         *
         * @return Angle of the ray that observed this point relative to the LiDAR frame.
         */
        float getAngle() {
            return angle_;
        }

        /**
         * Get the range measurement of this point.
         *
         * @return Range measurement of this point.
         */
        float getRange() {
            return range_;
        }

    private:

        /**
         * Angle of the beam that took this measurement, relative to the frame of the LiDAR.
         */
        float angle_;

        /**
         * Range measurement recorded (distance from LiDAR to obstacle).
         */
        float range_;

        /**
         * Label for the point.
         */
        PointLabel label_;

        /**
         * Number of the sector that this point falls into.
         */
        uint8_t sector_num_;
    };

    class Measurement {
    public:

        /**
         * Create a measurement based on a full scan.
         *
         * @param num_sectors   Number of sectors in the scan.
         * @param measurements  Individual measurements that compose scan.
         */
        Measurement(const uint8_t &num_sectors,
                    const std::vector<MeasurementPoint> &measurements) : measurements_(measurements) {
            for (uint8_t sector_num = 0; sector_num < num_sectors; sector_num++) {
                sector_activation_[sector_num] = true;
            }
        }

        // TODO still not sure what constructor should look like
        // Should it be responsible for assigning sectors to the measurement points?

        /**
         * Deactivate the given sector
         * @param sector_to_deactivate
         */
        void deactivateSector(const uint8_t &sector_to_deactivate) {
            if (sector_activation_.find(sector_to_deactivate) != sector_activation_.end()) {
                sector_activation_[sector_to_deactivate] = false;
            } else {
                ROS_ERROR("Tried to deactivate sector that doesn't exist");
            }
        }

        /**
         * Get the individual range measurements that compose this scan.
         *
         * @return individual range measurements that compose this scan.
         */
        std::vector<MeasurementPoint> getMeasurements() const {
            return measurements_;
        }

    private:

        /**
         * Vector of individual measurement points
         */
        std::vector<MeasurementPoint> measurements_;

        /**
         * Map indicating if a sector is active or not. True is active, false is inactive.
         */
        std::unordered_map<uint8_t, bool> sector_activation_;
    };
} // end dpg_slam
