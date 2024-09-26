//
// Created by Cain on 2024/9/25.
//

#ifndef SLAM_VISION_FUNDAMENTAL_H
#define SLAM_VISION_FUNDAMENTAL_H

#include "common.h"

#define TYPE SLAM_DATA_TYPE

namespace slam {
    // TODO: 函数传入参数不应该多于4个
    /*!
     * 八点最小二乘 + RANSAC 计算基础矩阵
     * @param match_pairs 匹配特征点在归一化平面的坐标
     * @param F 基础矩阵
     * @param is_outliers 标记特征点是否为外点
     * @param sigma RANSAC 方差
     * @param confidence RANSAC 置信度
     * @param max_iters RANSAC 最大迭代次数
     * @return 内点个数
     */
    unsigned long compute_fundamental_8pts(const vector<pair<const Vec3*, const Vec3*>> &match_pairs,
                                           Mat3_3 &F, vector<bool> &is_outliers,
                                           TYPE sigma=TYPE(0.01), TYPE confidence=TYPE(0.99), unsigned int max_iters=1000);

    // TODO: 函数传入参数不应该多于4个
    /*!
     * 七点最小二乘 + RANSAC 计算基础矩阵
     * @param match_pairs 匹配特征点在归一化平面的坐标
     * @param F 基础矩阵
     * @param is_outliers 标记特征点是否为外点
     * @param sigma RANSAC 方差
     * @param confidence RANSAC 置信度
     * @param max_iters RANSAC 最大迭代次数
     * @return 内点个数
     */
    unsigned long compute_fundamental_7pts(const vector<pair<const Vec3*, const Vec3*>> &match_pairs,
                                           Mat3_3 &F, vector<bool> &is_outliers,
                                           TYPE sigma=TYPE(0.01), TYPE confidence=TYPE(0.99), unsigned int max_iters=1000);
}

#endif //SLAM_VISION_FUNDAMENTAL_H
