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
    enum PointLabel {STATIC, ADDED, REMOVED, NOT_YET_LABELED, MAX_RANGE};

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
         * @param max_range     Maximum range reading (we should ignore any that are not less than this).
         */
        MeasurementPoint(const float &angle, const float &range, const uint8_t &sector_num, const float &max_range) : angle_(angle),
        range_(range), label_(NOT_YET_LABELED), sector_num_(sector_num) {
            if (range_ >= max_range) {
                label_ = MAX_RANGE;
            }
        }

        /**
         * Set the label for the point.
         *
         * Should not be NOT_YET_LABELED
         *
         * @param new_label New label that the point should have
         */
        void setLabel(const PointLabel &new_label) {
            if (label_ != MAX_RANGE) {
                label_ = new_label;
            }
        }

        /**
         * Get the angle of the ray that observed this point relative to the LiDAR frame.
         *
         * @return Angle of the ray that observed this point relative to the LiDAR frame.
         */
        float getAngle() const {
            return angle_;
        }

        /**
         * Get the range measurement of this point.
         *
         * @return Range measurement of this point.
         */
        float getRange() const {
            return range_;
        }

        /**
         * Get the sector number of this point
         *
         * @return sector number of this point
         */
        uint8_t getSectorNum() const {
             return sector_num_;
        }

        /**
         * Get the label for the point.
         *
         * @return Label for the point.
         */
        PointLabel getLabel() const {
            return label_;
        }

        /**
         * Get the point in the laser frame (in cartesian coordinates).
         *
         * @return Point in the laser frame (in cartesian coords).
         */
        Eigen::Vector2f getPointInLaserFrame() const {
            return Eigen::Vector2f(getRange() * cos(getAngle()), getRange() * sin(getAngle()));
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
         * @param num_sectors       Number of sectors in the scan.
         * @param measurements      Individual measurements that compose scan.
         * @param angle_min         Minimum possible angle.
         * @param angle_max         Maximum possible angle.
         * @param max_range         Maximum possible range reading.
         * @param angle_increment   Change in angle between neighboring scan points.
         */
        Measurement(const uint8_t &num_sectors,
                    const std::vector<MeasurementPoint> &measurements, const float &angle_min,
                    const float &angle_max, const float &max_range, const float &angle_increment) : num_sectors_(num_sectors), activated_sectors_(num_sectors),
                    max_range_(max_range), angle_increment_(angle_increment), measurements_(measurements) {
            for (uint8_t sector_num = 0; sector_num < num_sectors; sector_num++) {
                sector_activation_[sector_num] = true;
            }
        }

        /**
         * Deactivate the given sector
         * @param sector_to_deactivate
         */
        void deactivateSector(const uint8_t &sector_to_deactivate) {
            if (sector_activation_.find(sector_to_deactivate) != sector_activation_.end()) {
                if (sector_activation_[sector_to_deactivate]) {
                    activated_sectors_--;
                    sector_activation_[sector_to_deactivate] = false;
                }
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

        /**
         * Get the percent of sectors that are active.
         *
         * @return Percent of sectors that are active.
         */
        float getPercentSectorsActive() const {
            return ((float) activated_sectors_) / num_sectors_;
        }

        /**
         * Get the measurement range for the measurement (angle min/max, relative to the laser frame).
         *
         * @return the measurement range for the measurement (angle min/max, relative to the laser frame).
         */
        std::pair<float, float> getMeasurementRange() const {
            return std::make_pair(angle_min_, angle_max_);
        }

        /**
         * Get the number of sectors.
         *
         * @return Number of sectors.
         */
        uint8_t getNumSectors() const {
            return num_sectors_;
        }

        /**
         * Get the maximum range of the lidar.
         *
         * @return maximum range of the lidar.
         */
        float getMaxRange() const {
            return max_range_;
        }

        /**
         * Get the angle between neighboring points in the scan.
         *
         * @return the angle between neighboring points in the scan.
         */
        float getAngleInc() const {
            return angle_increment_;
        }

	/*
	 * TO know if a sector is active or inactive
	 *
	 * @param sector_num_
	 *
	 * @return bool representing active or inactive sector.
	 */
	bool isSectorActive(const uint8_t sector_num) {
	    return sector_activation_[sector_num];
	}

    private:

        /**
         * Number of sectors in the measurement.
         */
        uint8_t num_sectors_;

        /**
         * Number of activated sectors.
         */
        uint8_t activated_sectors_;

        /**
         * Minimum angle for the scan.
         */
        float angle_min_;

        /**
         * Maximum angle for the scan.
         */
        float angle_max_;

        /**
         * Maximum possible range for the scan.
         */
        float max_range_;

        /**
         * The angle between neighboring points in the scan.
         */
        float angle_increment_;

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
