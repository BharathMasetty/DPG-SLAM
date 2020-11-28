//
// Created by amanda on 11/27/20.
//

#pragma once

#include <eigen3/Eigen/Geometry>

namespace math_utils {

    // Below functions are taken from cs393r_starter
    template <typename T>
    T AngleMod(T angle) {
        angle -= (M_PI * 2.0) * rint(angle / (M_PI * 2.0));
        return angle;
    }

    template <typename T>
    T AngleDist(T a0, T a1) {
        return std::fabs<T>(AngleMod<T>(a0 - a1));
    }

    template <typename T>
    T AngleDiff(T a0, T a1) {
        return AngleMod<T>(a0 - a1);
    }
    // Above functions are taken from cs393r_starter

    /**
     * Get the pose of the src frame object in the target frame given the pose of the source frame in the target frame.
     *
     * Ex. get the pose of an object in the map frame given the pose in the base link frame and the pose of the base
     * link in the robot's frame.
     *
     * @param src_frame_point                     Location of the point in the source frame (base link frame in the example).
     * @param src_frame_angle                     Angle of the point in the source frame (base link frame in the example).
     * @param src_frame_pos_rel_target_frame      Position of the origin of the source frame in the target frame
     *                                            (position of the base link frame in the map frame in the example).
     * @param src_frame_angle_rel_target_frame    Angle of the source frame relative to the target frame (angle from map
     *                                            x axis to base link x axis in the example).
     *
     * @return Pose of point in the target frame (map frame in this example).
     */
    std::pair<Eigen::Vector2f, float> transformPoint(const Eigen::Vector2f &src_frame_point, const float &src_frame_angle,
                   const Eigen::Vector2f &src_frame_pos_rel_target_frame,
                   const float &src_frame_angle_rel_target_frame);

    /**
     * Get the pose of the src frame object in the target frame given the pose of the target frame in the source frame.
     *
     * Ex. get the pose of an object relative to base link given the pose of the object in the map frame and the pose of
     * the robot in the map frame
     *
     * @param src_frame_point                     Location of the point in the source frame (map frame in the example).
     * @param src_frame_angle                     Angle of the point in the source frame (map frame in the example).
     * @param target_frame_pos_rel_src_frame      Position of the origin of the target frame in the src frame
     *                                            (position of the base link frame in the map frame in the example).
     * @param target_frame_angle_rel_src_frame    Angle of the target frame relative to the src frame (angle from map
     *                                            x axis to base link x axis in the example).
     * @return Pose of point in the target frame (base link frame in this example).
     */
    std::pair<Eigen::Vector2f, float> inverseTransformPoint(const Eigen::Vector2f &src_frame_point,
                                                            const float &src_frame_angle,
                                                            const Eigen::Vector2f &target_frame_pos_rel_src_frame,
                                                            const float &target_frame_angle_rel_src_frame);

}