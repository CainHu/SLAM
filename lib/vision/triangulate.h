//
// Created by Cain on 2024/9/18.
//

#ifndef SLAM_VISION_TRIANGULATE_H
#define SLAM_VISION_TRIANGULATE_H

#include "common.h"

#define TYPE SLAM_DATA_TYPE

namespace slam {
    // TODO: 函数传入参数不应该多于4个
    /*!
     * 三角化
     * @param point_i 特征点在归一化平面i的坐标
     * @param point_j 特征点在归一化平面j的坐标
     * @param R 归一化平面 j 到 i 的旋转
     * @param t 归一化平面 j 指向 i 的位移在 i 上的投影
     * @param p 三角化得到的特征点在归一化平面 i 的投影
     * @param sigma 重投影误差的方差
     * @param depth_min 特征点的最小深度
     * @return 特征点是否三角化成功
     */
    TYPE triangulate_with(const Vec3 *point_i, const Vec3 *point_j, const Mat3_3 &R, const Vec3 &t, Vec3 &p, TYPE sigma=TYPE(1), TYPE depth_min=TYPE(0));

    // TODO: 函数传入参数不应该多于4个
    /*!
     * 利用三角化，选择出正确的R, t
     * @param match_pairs 匹配特征点
     * @param R_vec 候选的R
     * @param t_vec 候选的t
     * @param points 三角化得到的特征点坐标
     * @param is_outliers 特征点是否为外点
     * @param num_inliers 匹配特征点中有多少内点
     * @return
     */
    int select_pose_by_triangulated(const vector<pair<const Vec3*, const Vec3*>> &match_pairs,
                                    const vector<Mat3_3> &R_vec, const vector<Vec3> &t_vec,
                                    vector<Vec3> &points, vector<bool> &is_outliers, unsigned long num_inliers);
}

#undef TYPE

#endif //SLAM_VISION_TRIANGULATE_H
