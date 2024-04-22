/*
 * Copyright (c) 2020 - 2021, VinAI. All rights reserved. All information
 * information contained herein is proprietary and confidential to VinAI.
 * Any use, reproduction, or disclosure without the written permission
 * of VinAI is prohibited.
 */

#ifndef SVM_AUTORC_UTILS_H
#define SVM_AUTORC_UTILS_H

#include <Eigen/Dense>
#include <random>
#include "defines.h"
#include "optimizer.h"
#include "transform_util.h"

namespace util {

inline std::pair<double, double> calculateError(const Eigen::Matrix4d& ext1,
                                                const Eigen::Matrix4d& ext2)
{
    // translation error
    Eigen::Vector3d translation1 = ext1.block<3, 1>(0, 3);
    Eigen::Vector3d translation2 = ext2.block<3, 1>(0, 3);
    double translationError      = (translation1 - translation2).norm();

    // rotation error
    Eigen::Matrix3d rotation1 = ext1.block<3, 3>(0, 0);
    Eigen::Matrix3d rotation2 = ext2.block<3, 3>(0, 0);
    double trace              = (rotation1.transpose() * rotation2).trace();
    double rotationError      = std::acos((trace - 1.0) / 2.0) * 180 / M_PI;
    return std::make_pair(translationError, rotationError);
}

inline void genDisturbance(Eigen::Matrix<float, 4, 6>& disturbances)
{
    std::uniform_real_distribution<float> t_dist(0.001, 0.01);
    std::uniform_real_distribution<float> r_dist(1.0, 4.0);
    std::uniform_real_distribution<float> sign_dist(0.0f, 1.0f);
    std::random_device gen;
    for (size_t camid = 0; camid < CamID::NUM_CAM; camid++)
    {
        disturbances(camid, 0) = t_dist(gen) * (int(sign_dist(gen) > 0.5) * 2 - 1);
        disturbances(camid, 1) = t_dist(gen) * (int(sign_dist(gen) > 0.5) * 2 - 1);
        disturbances(camid, 2) = t_dist(gen) * (int(sign_dist(gen) > 0.5) * 2 - 1);
        disturbances(camid, 3) = r_dist(gen) * (int(sign_dist(gen) > 0.5) * 2 - 1);
        disturbances(camid, 4) = r_dist(gen) * (int(sign_dist(gen) > 0.5) * 2 - 1);
        disturbances(camid, 5) = r_dist(gen) * (int(sign_dist(gen) > 0.5) * 2 - 1);
    }
}

inline void addDisturbance(CamID fixed, std::array<Eigen::Matrix4d, 4>& initExt,
                           const Eigen::Matrix<float, 4, 6>& disturbances)
{
    for (size_t camid = 0; camid < CamID::NUM_CAM; camid++)
    {
        if ((camid == CamID::B && fixed == CamID::B) || (camid == CamID::F && fixed == CamID::F))
        {
            continue;
        }
        if (camid == 1) continue;

        LOG_INFO("adding disturbance for cam: {}", camid);
        Eigen::Matrix4d disturbance;
        Eigen::Matrix3d disturbance_rot_mat;
        Vec3f disturbance_rot_euler;  // R(euler)

        Mat_<double> disturbance_t = (Mat_<double>(3, 1) << disturbances(camid, 0),
                                      disturbances(camid, 1), disturbances(camid, 2));
        disturbance_rot_euler << disturbances(camid, 3), disturbances(camid, 4),
            disturbances(camid, 5);
        disturbance_rot_mat = TransformUtil::eulerAnglesToRotationMatrix(disturbance_rot_euler);
        disturbance =
            TransformUtil::R_T2RT(TransformUtil::eigen2mat(disturbance_rot_mat), disturbance_t);
        initExt[camid] *= disturbance;
    }
}

}  // namespace util

#endif  // SVM_AUTORC_UTILS_H
