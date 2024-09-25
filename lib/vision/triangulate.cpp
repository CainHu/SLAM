//
// Created by Cain on 2024/9/18.
//

#include "triangulate.h"

#define TYPE SLAM_DATA_TYPE

TYPE slam::triangulate_with(const Vec3 *point_i, const Vec3 *point_j, const Mat3_3 &R, const Vec3 &t, Vec3 &p, TYPE sigma, TYPE depth_min) {
    auto sigma2 = sigma * sigma;
    auto th_e2 = TYPE(3.841) * sigma2;
    auto th_score = TYPE(5.991) * sigma2;
    TYPE score = TYPE(-1);

    Vec3 RTt = R.transpose() * t;
    Mat4_3r A;
    A.row(0) << TYPE(1), TYPE(0), -point_i->x();
    A.row(1) << TYPE(0), TYPE(1), -point_i->y();
    A.row(2) = R.col(0).transpose() - point_j->x() * R.col(2).transpose();
    A.row(3) = R.col(1).transpose() - point_j->y() * R.col(2).transpose();
    Vec4 b;
    b << TYPE(0), TYPE(0), RTt[0] - RTt[2] * point_j->x(), RTt[1] - RTt[2] * point_j->y();

    // 使用QR分解求解最小二乘问题，这样数值精度更加稳定
    p = A.fullPivHouseholderQr().solve(b);

    // 判断是否outlier
    if (p.z() < depth_min) {
        return score;
    }

    Vec3 pj = R.transpose() * (p - t);
    if (pj.z() < depth_min) {
        return score;
    }

    TYPE e2_i = (*point_i - p / p.z()).squaredNorm();
    if (e2_i > th_e2) {
        return score;
    }

    TYPE e2_j = (*point_j - pj / pj.z()).squaredNorm();
    if (e2_j > th_e2) {
        return score;
    }

    score = th_score - e2_i + th_score - e2_j;

    return score;
}

int slam::select_pose_by_triangulated(const vector<pair<const Vec3 *, const Vec3 *>> &match_pairs,
                                      const vector<Mat3_3> &R_vec, const vector<Vec3> &t_vec,
                                      vector<slam::Vec3> &points, vector<bool> &is_outliers, unsigned long num_inliers) {
    auto num_poses = (int)R_vec.size();
    unsigned long num_points = match_pairs.size();

    if (is_outliers.size() != num_points) {
        is_outliers.resize(num_points, false);
    }

    vector<TYPE> score_vec(num_poses, TYPE(0));
    vector<vector<bool>> is_outliers_vec(num_poses, is_outliers);
    vector<vector<Vec3>> points_vec(num_poses, vector<Vec3>(num_points));

    vector<unsigned long> num_inliers_vec(num_poses);
    for (int i = 0; i < num_poses; ++i) {
        num_inliers_vec[i] = 0;
        for (unsigned long k = 0; k < num_points; ++k) {
            if (is_outliers[k]) {
                continue;
            }

            TYPE score = triangulate_with(match_pairs[k].first, match_pairs[k].second, R_vec[i], t_vec[i], points_vec[i][k]);
            if (score < TYPE(0)) {
                is_outliers_vec[i][k] = true;
                continue;
            }

            score_vec[i] += score;
            ++num_inliers_vec[i];
        }
    }

    int max_inlier_points_index = 0;
    unsigned long max_inlier_points = num_inliers_vec[0];
    for (int i = 1; i < num_poses; ++i) {
        if (num_inliers_vec[i] > num_inliers_vec[max_inlier_points_index] ||
            (num_inliers_vec[i] == num_inliers_vec[max_inlier_points_index] && score_vec[i] > score_vec[max_inlier_points_index])) {
            max_inlier_points_index = i;
            max_inlier_points = num_inliers_vec[i];
        }
    }

    // 至少要超过90%的点成功被三角化
    unsigned long min_inlier_points = 9 * num_inliers / 10;
    if (max_inlier_points < min_inlier_points) {
#ifdef PRINT_INFO
        std::cout << "max_inlier_points = " << max_inlier_points << ", min_inlier_points = " << min_inlier_points << std::endl;
            std::cout << "inlier_points[0] = " << inlier_points[0] << std::endl;
            std::cout << "inlier_points[1] = " << inlier_points[1] << std::endl;
            std::cout << "inlier_points[2] = " << inlier_points[2] << std::endl;
            std::cout << "inlier_points[3] = " << inlier_points[3] << std::endl;
#endif
        return -1;
    }

//    // 记录有多少组解使得70%的点都能三角化
//    unsigned long lim_inlier_points = 7 * max_inlier_points / 10;
//    unsigned long num_similar = 0;
//    for (int i = 0; i < num_poses; ++i) {
//        if (num_inliers_vec[i] > lim_inlier_points) {
//            ++num_similar;
//        }
//    }

//    // 不允许超过1组解使得70%的点都能三角化
//    if (num_similar > 1) {
//#ifdef PRINT_INFO
//        std::cout << "num_similar > 1" << std::endl;
//#endif
//        return -1;
//    }

    // 保存最正确的解
    points = points_vec[max_inlier_points_index];
    is_outliers = is_outliers_vec[max_inlier_points_index];

    return max_inlier_points_index;
}