//
// Created by amanda on 11/25/20.
//

#pragma once

#include <unordered_map>
#include <vector>

namespace dpg_slam {

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
        range_(range), sector_num_(sector_num) {

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
         * Number of the sector that this point falls into.
         */
        uint8_t sector_num_;
    };

    class Measurement {
    public:

        // TODO still not sure what constructor should look like
        // Should it be responsible for assigning sectors to the measurement points?

        void deactivateSector(const uint8_t &sector_to_deactivate) {
            if (sector_activation_.find(sector_to_deactivate) != sector_activation_.end()) {
                sector_activation_[sector_to_deactivate] = false;
            }
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
