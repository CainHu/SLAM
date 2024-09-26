//
// Created by Cain on 2024/9/25.
//

#include "fundamental.h"

#define TYPE SLAM_DATA_TYPE

unsigned long slam::compute_fundamental_8pts(const vector<pair<const Vec3*, const Vec3*>> &match_pairs,
                                             Mat3_3 &F, vector<bool> &is_outliers,
                                             TYPE sigma, TYPE confidence, unsigned int max_iters) {
    constexpr static unsigned int N = 8;
    auto sigma2 = sigma * sigma;
    auto th_e2 = TYPE(3.841) * sigma2;
    auto th_score = TYPE(5.991) * sigma2;
    TYPE ransac_k = log(max(TYPE(1) - confidence, TYPE(1e-5)));

    size_t num_points = match_pairs.size();
    if (is_outliers.size() != num_points) {
        is_outliers.resize(num_points, false);
    }

    // 归一化变换参数
    Mat3_3 Ti, Tj, Ti_inv, Tj_inv;
    TYPE meas_x_i = TYPE(0), meas_y_i = TYPE(0);
    TYPE dev_x_i = TYPE(0), dev_y_i = TYPE(0);
    TYPE meas_x_j = TYPE(0), meas_y_j = TYPE(0);
    TYPE dev_x_j = TYPE(0), dev_y_j = TYPE(0);

    // 计算均值
    for (auto &match_pair : match_pairs) {
        meas_x_i += match_pair.first->x();
        meas_y_i += match_pair.first->y();
        meas_x_j += match_pair.second->x();
        meas_y_j += match_pair.second->y();
    }
    meas_x_i /= TYPE(num_points);
    meas_y_i /= TYPE(num_points);
    meas_x_j /= TYPE(num_points);
    meas_y_j /= TYPE(num_points);

    // 计算Dev
    for (auto &match_pair : match_pairs) {
        dev_x_i += abs(match_pair.first->x() - meas_x_i);
        dev_y_i += abs(match_pair.first->y() - meas_y_i);
        dev_x_j += abs(match_pair.second->x() - meas_x_j);
        dev_y_j += abs(match_pair.second->y() - meas_y_j);
    }
    dev_x_i /= TYPE(num_points);
    dev_y_i /= TYPE(num_points);
    dev_x_j /= TYPE(num_points);
    dev_y_j /= TYPE(num_points);

    // 归一化变换
    Ti << TYPE(1) / dev_x_i, TYPE(0), -meas_x_i / dev_x_i,
            TYPE(0), TYPE(1) / dev_y_i, -meas_y_i / dev_y_i,
            TYPE(0), TYPE(0), TYPE(1);
    Tj << TYPE(1) / dev_x_j, TYPE(0), -meas_x_j / dev_x_j,
            TYPE(0), TYPE(1) / dev_y_j, -meas_y_j / dev_y_j,
            TYPE(0), TYPE(0), TYPE(1);

    Ti_inv << dev_x_i, TYPE(0), meas_x_i,
            TYPE(0), dev_y_i, meas_y_i,
            TYPE(0), TYPE(0), TYPE(1);
    Tj_inv << dev_x_j, TYPE(0), meas_x_j,
            TYPE(0), dev_y_j, meas_y_j,
            TYPE(0), TYPE(0), TYPE(1);

    // 归一化后的点
    vector<pair<Vec2, Vec2>> normal_match_pairs(num_points);
    for (unsigned long k = 0; k < num_points; ++k) {
        normal_match_pairs[k].first.x() = (match_pairs[k].first->x() - meas_x_i) / dev_x_i;
        normal_match_pairs[k].first.y() = (match_pairs[k].first->y() - meas_y_i) / dev_y_i;
        normal_match_pairs[k].second.x() = (match_pairs[k].second->x() - meas_x_j) / dev_x_j;
        normal_match_pairs[k].second.y() = (match_pairs[k].second->y() - meas_y_j) / dev_y_j;
    }

    // 构造随机index batch
    random_device rd;
    mt19937 gen(rd());
    array<unsigned long, N> local_index_map {};
    vector<unsigned long> global_index_map(num_points);
    for (unsigned long k = 0; k < num_points; ++k) {
        global_index_map[k] = k;
    }
    auto generator_indices_set = [&]() -> array<unsigned long, N> {
        array<unsigned long, N> point_indices_set{};
        for (unsigned int k = 0; k < N; ++k) {
            uniform_int_distribution<unsigned int> dist(0, global_index_map.size() - 1);
            unsigned int rand_i = dist(gen);
            auto index = global_index_map[rand_i];
            point_indices_set[k] = index;
            local_index_map[k] = index;

            global_index_map[rand_i] = global_index_map.back();
            global_index_map.pop_back();
        }

        for (unsigned int k = 0; k < N; ++k) {
            global_index_map.emplace_back(local_index_map[k]);
        }
        return point_indices_set;
    };

    // TODO: 使用多线程
    // RANSAC: 计算本质矩阵
    auto best_score = TYPE(0);
    unsigned int best_index = 0;
    unsigned int curr_index = 1;
    unsigned long num_inliers_vec[2] = {0, 0};
    vector<bool> is_outliers_vec[2] {vector<bool>(num_points, false), vector<bool>(num_points, false)};
    for (unsigned int n = 0; n < max_iters; ++n) {
        // 八点法
        Eigen::Matrix<TYPE, N, 9, Eigen::RowMajor> D;
        auto &&point_indices_set = generator_indices_set();

        // D * f = 0
        for (unsigned int k = 0; k < N; ++k) {
            unsigned int index = point_indices_set[k];
            auto u1 = normal_match_pairs[index].first.x();
            auto v1 = normal_match_pairs[index].first.y();
            auto u2 = normal_match_pairs[index].second.x();
            auto v2 = normal_match_pairs[index].second.y();

            // p1' * f * p2
            // f = [f0, f3, f6
            //      f1, f4, f7
            //      f2, f5, f8]
            D.row(k) << u1 * u2, v1 * u2, u2, u1 * v2, v1 * v2, v2, u1, v1, TYPE(1);
        }

        // SVD求解基础矩阵
        auto D_svd = D.jacobiSvd(Eigen::ComputeFullV);
        if (D_svd.rank() < N) {
            continue;
        }

        // 基础矩阵
        Vec9 f = D_svd.matrixV().col(8);
        Mat3_3 F_raw = Ti.transpose() * Eigen::Map<Mat3_3>(f.data()) * Tj;

        // 添加基础矩阵的奇异值约束
        Eigen::JacobiSVD<Mat3_3 > F_svd(F_raw, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Vec3 s = F_svd.singularValues();
        s(2) = 0.;
        Mat3_3 F12 = F_svd.matrixU() * s.asDiagonal() * F_svd.matrixV().transpose();

        // 遍历所有的点计算分数
        auto score = TYPE(0);
        num_inliers_vec[curr_index] = 0;
        for (unsigned long k = 0; k < num_points; ++k) {
            bool is_outlier = is_outliers[k];

            Vec3 Fp1 = F12 * *match_pairs[k].first;
            Vec3 Fp2 = F12 * *match_pairs[k].second;
            auto p1Fp2 = match_pairs[k].first->dot(Fp2);
            auto e2 = p1Fp2 * p1Fp2 / (Fp2.head<2>().squaredNorm() + Fp1.head<2>().squaredNorm());
            if (e2 >= th_e2) {
                is_outlier = true;
            }

            is_outliers_vec[curr_index][k] = is_outlier;
            if (!is_outlier) {
                ++num_inliers_vec[curr_index];
                score += th_score - e2;
            }
        }

        if (num_inliers_vec[curr_index] > num_inliers_vec[best_index] ||
            (num_inliers_vec[curr_index] == num_inliers_vec[best_index] && score > best_score)) {
            F = F12;
            auto inlier_ratio = TYPE(num_inliers_vec[curr_index]) / TYPE(num_points);
            auto ransac_n = ransac_k / log(TYPE(1) - pow(inlier_ratio, N));
            if (ransac_n < TYPE(max_iters)) {
                max_iters = (unsigned int)ceil(ransac_n);
            }
            swap(curr_index, best_index);
            best_score = score;
        }
    }

    is_outliers = is_outliers_vec[best_index];
    return num_inliers_vec[best_index];
}


unsigned long slam::compute_fundamental_7pts(const vector<pair<const Vec3*, const Vec3*>> &match_pairs,
                                             Mat3_3 &F, vector<bool> &is_outliers,
                                             TYPE sigma, TYPE confidence, unsigned int max_iters) {
    constexpr static unsigned int N = 7;
    auto sigma2 = sigma * sigma;
    auto th_e2 = TYPE(3.841) * sigma2;
    auto th_score = TYPE(5.991) * sigma2;
    TYPE ransac_k = log(max(TYPE(1) - confidence, TYPE(1e-5)));

    size_t num_points = match_pairs.size();
    if (is_outliers.size() != num_points) {
        is_outliers.resize(num_points, false);
    }

    // 归一化变换参数
    Mat3_3 Ti, Tj, Ti_inv, Tj_inv;
    TYPE meas_x_i = TYPE(0), meas_y_i = TYPE(0);
    TYPE dev_x_i = TYPE(0), dev_y_i = TYPE(0);
    TYPE meas_x_j = TYPE(0), meas_y_j = TYPE(0);
    TYPE dev_x_j = TYPE(0), dev_y_j = TYPE(0);

    // 计算均值
    for (auto &match_pair : match_pairs) {
        meas_x_i += match_pair.first->x();
        meas_y_i += match_pair.first->y();
        meas_x_j += match_pair.second->x();
        meas_y_j += match_pair.second->y();
    }
    meas_x_i /= TYPE(num_points);
    meas_y_i /= TYPE(num_points);
    meas_x_j /= TYPE(num_points);
    meas_y_j /= TYPE(num_points);

    // 计算Dev
    for (auto &match_pair : match_pairs) {
        dev_x_i += abs(match_pair.first->x() - meas_x_i);
        dev_y_i += abs(match_pair.first->y() - meas_y_i);
        dev_x_j += abs(match_pair.second->x() - meas_x_j);
        dev_y_j += abs(match_pair.second->y() - meas_y_j);
    }
    dev_x_i /= TYPE(num_points);
    dev_y_i /= TYPE(num_points);
    dev_x_j /= TYPE(num_points);
    dev_y_j /= TYPE(num_points);

    // 归一化变换
    Ti << TYPE(1) / dev_x_i, TYPE(0), -meas_x_i / dev_x_i,
            TYPE(0), TYPE(1) / dev_y_i, -meas_y_i / dev_y_i,
            TYPE(0), TYPE(0), TYPE(1);
    Tj << TYPE(1) / dev_x_j, TYPE(0), -meas_x_j / dev_x_j,
            TYPE(0), TYPE(1) / dev_y_j, -meas_y_j / dev_y_j,
            TYPE(0), TYPE(0), TYPE(1);

    Ti_inv << dev_x_i, TYPE(0), meas_x_i,
            TYPE(0), dev_y_i, meas_y_i,
            TYPE(0), TYPE(0), TYPE(1);
    Tj_inv << dev_x_j, TYPE(0), meas_x_j,
            TYPE(0), dev_y_j, meas_y_j,
            TYPE(0), TYPE(0), TYPE(1);

    // 归一化后的点
    vector<pair<Vec2, Vec2>> normal_match_pairs(num_points);
    for (unsigned long k = 0; k < num_points; ++k) {
        normal_match_pairs[k].first.x() = (match_pairs[k].first->x() - meas_x_i) / dev_x_i;
        normal_match_pairs[k].first.y() = (match_pairs[k].first->y() - meas_y_i) / dev_y_i;
        normal_match_pairs[k].second.x() = (match_pairs[k].second->x() - meas_x_j) / dev_x_j;
        normal_match_pairs[k].second.y() = (match_pairs[k].second->y() - meas_y_j) / dev_y_j;
    }

    // 构造随机index batch
    random_device rd;
    mt19937 gen(rd());
    array<unsigned long, N> local_index_map {};
    vector<unsigned long> global_index_map(num_points);
    for (unsigned long k = 0; k < num_points; ++k) {
        global_index_map[k] = k;
    }
    auto generator_indices_set = [&]() -> array<unsigned long, N> {
        array<unsigned long, N> point_indices_set{};
        for (unsigned int k = 0; k < N; ++k) {
            uniform_int_distribution<unsigned int> dist(0, global_index_map.size() - 1);
            unsigned int rand_i = dist(gen);
            auto index = global_index_map[rand_i];
            point_indices_set[k] = index;
            local_index_map[k] = index;

            global_index_map[rand_i] = global_index_map.back();
            global_index_map.pop_back();
        }

        for (unsigned int k = 0; k < N; ++k) {
            global_index_map.emplace_back(local_index_map[k]);
        }
        return point_indices_set;
    };

    // TODO: 使用多线程
    // RANSAC: 计算本质矩阵
    auto best_score = TYPE(0);
    unsigned int best_index = 0;
    unsigned int curr_index = 1;
    unsigned long num_inliers_vec[2] = {0, 0};
    vector<bool> is_outliers_vec[2] {vector<bool>(num_points, false), vector<bool>(num_points, false)};
    for (unsigned int n = 0; n < max_iters; ++n) {
        // 七点法
        Eigen::Matrix<TYPE, N, 9, Eigen::RowMajor> D;
        auto &&point_indices_set = generator_indices_set();

        // D * f = 0
        for (unsigned int k = 0; k < N; ++k) {
            unsigned int index = point_indices_set[k];
            auto u1 = normal_match_pairs[index].first.x();
            auto v1 = normal_match_pairs[index].first.y();
            auto u2 = normal_match_pairs[index].second.x();
            auto v2 = normal_match_pairs[index].second.y();

            // p1' * f * p2
            // f = [f0, f3, f6
            //      f1, f4, f7
            //      f2, f5, f8]
            D.row(k) << u1 * u2, v1 * u2, u2, u1 * v2, v1 * v2, v2, u1, v1, TYPE(1);
        }

        // 所有可能的基础矩阵
        vector<Mat3_3> results;
        results.reserve(4);

        // SVD求解基础矩阵, f = λ * v7 + v8
        auto D_svd = D.jacobiSvd(Eigen::ComputeFullV);
        if (D_svd.rank() < N) {
            continue;
        }

        Vec9 v7 = D_svd.matrixV().col(7);
        Vec9 v8 = D_svd.matrixV().col(8);
        auto V7 = Eigen::Map<Mat3_3>(v7.data());
        auto V8 = Eigen::Map<Mat3_3>(v8.data());

        // 如果 F = V7 满足 det(F) = 0, 那么 F 即为所求
        if (abs(V7.determinant()) < TYPE(1e-10)) {
            results.emplace_back(V7);
        }

        // 计算广义特征值和特征向量
        Eigen::GeneralizedEigenSolver<Eigen::MatrixXd> ges;
        ges.compute(V8, -V7, false);

        // 检查是否成功
        if (ges.info() == Eigen::Success) {
            // 获取特征值
            auto eigenvalues = ges.eigenvalues();
            for (int i = 0; i < eigenvalues.size(); ++i) {
                if (abs(eigenvalues[i].imag()) < 1e-10) {
                    results.emplace_back(V8 + eigenvalues[i].real() * V7);
                }
            }
        }

        // 遍历所有结果
        for (auto &result : results) {
            // 基础矩阵
            Mat3_3 F_raw = Ti.transpose() * result * Tj;

            // 添加本质矩阵的奇异值约束
            Eigen::JacobiSVD<Mat3_3 > F_svd(F_raw, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Vec3 s = F_svd.singularValues();
            s(2) = 0.;
            Mat3_3 F12 = F_svd.matrixU() * s.asDiagonal() * F_svd.matrixV().transpose();

            // 遍历所有的点计算分数
            auto score = TYPE(0);
            num_inliers_vec[curr_index] = 0;
            for (unsigned long k = 0; k < num_points; ++k) {
                bool is_outlier = is_outliers[k];

                Vec3 Fp1 = F12 * *match_pairs[k].first;
                Vec3 Fp2 = F12 * *match_pairs[k].second;
                auto p1Fp2 = match_pairs[k].first->dot(Fp2);
                auto e2 = p1Fp2 * p1Fp2 / (Fp2.head<2>().squaredNorm() + Fp1.head<2>().squaredNorm());
                if (e2 >= th_e2) {
                    is_outlier = true;
                }

                is_outliers_vec[curr_index][k] = is_outlier;
                if (!is_outlier) {
                    ++num_inliers_vec[curr_index];
                    score += th_score - e2;
                }
            }

            if (num_inliers_vec[curr_index] > num_inliers_vec[best_index] ||
                (num_inliers_vec[curr_index] == num_inliers_vec[best_index] && score > best_score)) {
                F = F12;
                auto inlier_ratio = TYPE(num_inliers_vec[curr_index]) / TYPE(num_points);
                auto ransac_n = ransac_k / log(TYPE(1) - pow(inlier_ratio, N));
                if (ransac_n < TYPE(max_iters)) {
                    max_iters = (unsigned int)ceil(ransac_n);
                }
                swap(curr_index, best_index);
                best_score = score;
            }
        }
    }

    is_outliers = is_outliers_vec[best_index];
    return num_inliers_vec[best_index];
}