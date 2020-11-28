
#include "math_utils.h"

namespace math_utils {

    std::pair<Eigen::Vector2f, float> transformPoint(const Eigen::Vector2f &src_frame_point, const float &src_frame_angle,
                                              const Eigen::Vector2f &src_frame_pos_rel_target_frame,
                                              const float &src_frame_angle_rel_target_frame) {

        // Rotate the point first
        Eigen::Rotation2Df rotation_mat(src_frame_angle_rel_target_frame);
        Eigen::Vector2f rotated_still_src_transl = rotation_mat * src_frame_point;

        // Then translate
        Eigen::Vector2f rotated_and_translated = src_frame_pos_rel_target_frame + rotated_still_src_transl;
        float target_angle = AngleMod(src_frame_angle_rel_target_frame + src_frame_angle);

        return std::make_pair(rotated_and_translated, target_angle);
    }

    std::pair<Eigen::Vector2f, float> inverseTransformPoint(const Eigen::Vector2f &src_frame_point,
                                                     const float &src_frame_angle,
                                                     const Eigen::Vector2f &target_frame_pos_rel_src_frame,
                                                     const float &target_frame_angle_rel_src_frame) {

        // Translate the point
        Eigen::Vector2f translated = src_frame_point - target_frame_pos_rel_src_frame;

        // Then rotate
        Eigen::Rotation2Df rotation_mat(-target_frame_angle_rel_src_frame);
        Eigen::Vector2f rotated_and_translated = rotation_mat * translated;

        float target_angle = AngleMod(src_frame_angle - target_frame_angle_rel_src_frame);
        return std::make_pair(rotated_and_translated, target_angle);
    }
}