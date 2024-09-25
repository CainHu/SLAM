//
// Created by Cain on 2024/9/24.
//

#ifndef SLAM_VISION_ESSENTIAL_H
#define SLAM_VISION_ESSENTIAL_H

#include "common.h"

#define TYPE SLAM_DATA_TYPE

namespace slam {
    // TODO: 函数传入参数不应该多于4个
    /*!
     * 八点最小二乘 + RANSAC 计算单应矩阵
     * @param match_pairs 匹配特征点在归一化平面的坐标
     * @param E 本质单应矩阵
     * @param is_outliers 标记特征点是否为外点
     * @param sigma RANSAC 方差
     * @param confidence RANSAC 置信度
     * @param max_iters RANSAC 最大迭代次数
     * @return 内点个数
     */
    unsigned long compute_essential_8pts(const vector<pair<const Vec3*, const Vec3*>> &match_pairs,
                                         Mat3_3 &E, vector<bool> &is_outliers,
                                         TYPE sigma=TYPE(0.01), TYPE confidence=TYPE(0.99), unsigned int max_iters=1000);

    // TODO: 函数传入参数不应该多于4个
    /*!
     * 八点最小二乘 + RANSAC 计算单应矩阵
     * @param match_pairs 匹配特征点在归一化平面的坐标
     * @param E 本质单应矩阵
     * @param is_outliers 标记特征点是否为外点
     * @param sigma RANSAC 方差
     * @param confidence RANSAC 置信度
     * @param max_iters RANSAC 最大迭代次数
     * @return 内点个数
     */
    unsigned long compute_essential_7pts(const vector<pair<const Vec3*, const Vec3*>> &match_pairs,
                                         Mat3_3 &E, vector<bool> &is_outliers,
                                         TYPE sigma=TYPE(0.01), TYPE confidence=TYPE(0.99), unsigned int max_iters=1000);

    // TODO: 函数传入参数不应该多于4个
    /*!
     * 八点最小二乘 + RANSAC 计算单应矩阵
     * @param match_pairs 匹配特征点在归一化平面的坐标
     * @param E 本质单应矩阵
     * @param is_outliers 标记特征点是否为外点
     * @param sigma RANSAC 方差
     * @param confidence RANSAC 置信度
     * @param max_iters RANSAC 最大迭代次数
     * @return 内点个数
     */
    unsigned long compute_essential_5pts(const vector<pair<const Vec3*, const Vec3*>> &match_pairs,
                                         Mat3_3 &E, vector<bool> &is_outliers,
                                         TYPE sigma=TYPE(0.01), TYPE confidence=TYPE(0.99), unsigned int max_iters=1000);

    /*!
     * 本质矩阵分解
     * @param E 本质矩阵
     * @param R_vec 一系列归一化平面 j 到 i 的旋转
     * @param t_vec 一系列归一化平面 j 指向 i 的位移在 i 上的投影
     * @return 是否有解
     */
    bool essential_decomposition(const Mat3_3 &E, vector<Mat3_3> &R_vec, vector<Vec3> &t_vec);

    // TODO: 函数传入参数不应该多于4个
    /*!
     * 通过计算本质矩阵来恢复 R, t, n
     * @param match_pairs 匹配特征点
     * @param R 归一化平面 j 到 i 的旋转
     * @param t 归一化平面 j 指向 i 的位移在 i 上的投影
     * @param points 三角化得到的特征点在归一化平面 i 的投影
     * @param is_outliers 特征点是否为外点
     * @param solve 本质矩阵求解的函数指针
     * @param sigma RANSAC 方差
     * @param confidence RANSAC 置信度
     * @param max_iters RANSAC 最大迭代次数
     * @return
     */
    bool reconstruct_from_essential(const vector<pair<const Vec3*, const Vec3*>> &match_pairs,
                                    Mat3_3 &R, Vec3 &t, vector<Vec3> &points, vector<bool> &is_outliers,
                                    unsigned long (*solve)(const vector<pair<const Vec3*, const Vec3*>>&,
                                                           Mat3_3&, vector<bool>&,
                                                           TYPE, TYPE, unsigned int)=compute_essential_5pts,
                                    TYPE sigma=TYPE(0.001), TYPE confidence=TYPE(0.99), unsigned int max_iters=1000);
}

#undef TYPE

#endif //SLAM_VISION_ESSENTIAL_H
