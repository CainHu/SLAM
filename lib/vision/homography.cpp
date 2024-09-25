//
// Created by Cain on 2024/9/18.
//
#include "homography.h"
#include "triangulate.h"

#define TYPE SLAM_DATA_TYPE

bool slam::reconstruct_from_homography(const vector<pair<const Vec3*, const Vec3*>> &match_pairs,
                                       Mat3_3 &R, Vec3 &t, Vec3 &n, vector<Vec3> &points, vector<bool> &is_outliers,
                                       bool (*decomp)(const Mat3_3&, vector<Mat3_3>&, vector<Vec3>&, vector<Vec3>&),
                                       TYPE sigma, TYPE confidence, unsigned int max_iters) {
    constexpr static unsigned int TH_COUNT = 12;
    unsigned long num_points = match_pairs.size();

    // 匹配点太少
    if (num_points < TH_COUNT) {
        return false;
    }

    // 计算单应矩阵
    Mat3_3 H;
    auto num_inliers = compute_homography_matrix(match_pairs, H, is_outliers, sigma, confidence, max_iters);
    if (num_inliers < TH_COUNT) {
        return false;
    }

    // 单应矩阵分解
    vector<Mat3_3> R_vec;
    vector<Vec3> t_vec;
    vector<Vec3> n_vec;
    if (!decomp(H, R_vec, t_vec, n_vec)) {
        return false;
    }

    // 三角化筛选R, t
    int index = select_pose_by_triangulated(match_pairs, R_vec, t_vec, points, is_outliers, num_inliers);
    if (index < 0) {
        return false;
    }

    // 保存正确的解
    R = R_vec[index];
    t = t_vec[index];
    n = n_vec[index];

    return true;
}


unsigned long slam::compute_homography_matrix(const vector<pair<const Vec3*, const Vec3*>> &match_pairs,
                                              Mat3_3 &H, vector<bool> &is_outliers,
                                              TYPE sigma, TYPE confidence, unsigned int max_iters) {
    constexpr static unsigned int N = 4;
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
    // RANSAC: 计算单应矩阵
    auto best_score = TYPE(0);
    unsigned int best_index = 0;
    unsigned int curr_index = 1;
    unsigned long num_inliers_vec[2] = {0, 0};
    vector<bool> is_outliers_vec[2] {vector<bool>(num_points, false), vector<bool>(num_points, false)};
    for (unsigned int n = 0; n < max_iters; ++n) {
        // 四点法
        Mat8_9r D;
        auto &&point_indices_set = generator_indices_set();

        // 检测是否出现共线
        bool linear = false;
        for (unsigned int i = 0; i < N - 2; ++i) {
            unsigned int ii = point_indices_set[i];
            for (unsigned int j = i + 1; j < N - 1; ++j) {
                unsigned int jj = point_indices_set[j];
                for (unsigned int k = j + 1; k < N; ++k) {
                    unsigned int kk = point_indices_set[k];
                    Vec2 a = normal_match_pairs[jj].first - normal_match_pairs[ii].first;
                    Vec2 b = normal_match_pairs[kk].first - normal_match_pairs[ii].first;
                    TYPE aTb = a.dot(b);
                    if (aTb * aTb >= TYPE(0.9999) * a.squaredNorm() * b.squaredNorm()) {
                        linear = true;
                        goto linear_checker;
                    }

                    Vec2 c = normal_match_pairs[jj].second - normal_match_pairs[ii].second;
                    Vec2 d = normal_match_pairs[kk].second - normal_match_pairs[ii].second;
                    TYPE cTd = c.dot(d);
                    if (cTd * cTd >= TYPE(0.9999) * c.squaredNorm() * d.squaredNorm()) {
                        linear = true;
                        goto linear_checker;
                    }
                }
            }
        }
linear_checker:
        if (linear) {
            continue;
        }

        // D * h = 0
        for (unsigned int k = 0; k < N; ++k) {
            unsigned int index = point_indices_set[k];
            auto u1 = normal_match_pairs[index].first.x();
            auto v1 = normal_match_pairs[index].first.y();
            auto u2 = normal_match_pairs[index].second.x();
            auto v2 = normal_match_pairs[index].second.y();

            // p1 = a * H*p2
            D.row(2 * k) << u2, TYPE(0), -u1 * u2, v2, TYPE(0),  -u1 * v2, TYPE(1), TYPE(0), -u1;
            D.row(2 * k + 1) << TYPE(0), u2, -v1 * u2, TYPE(0),  v2, -v1 * v2, TYPE(0), TYPE(1), -v1;
        }

        // SVD求解单应矩阵
        Eigen::JacobiSVD<Mat8_9r> D_svd(D, Eigen::ComputeFullV);
        Vec9 h = D_svd.matrixV().col(8);
        Eigen::Map<Mat3_3> H_raw(h.data());
        Mat3_3 H12 = Ti_inv * H_raw * Tj;
        Mat3_3 H21 = H12.inverse();

        // 检查H的符号, 必须所有 p1' * H * p2 同号
        vector<bool> sign_vec(N);
        for (unsigned int k = 0; k < N; ++k) {
            unsigned int index = point_indices_set[k];
            sign_vec[k] = match_pairs[index].first->dot(H12 * (*match_pairs[index].second)) > TYPE(0);
        }
        unsigned int p_count = std::count(sign_vec.begin(), sign_vec.end(), true);
        if (p_count == 0) {
            H12 = -H12;
            H21 = -H21;
        } else if (p_count != N) {
            continue;
        }

        // 遍历所有的点计算分数
        auto score = TYPE(0);
        num_inliers_vec[curr_index] = 0;
        for (unsigned long k = 0; k < num_points; ++k) {
            bool is_outlier = is_outliers[k];

            Vec3 Hp2 = H12 * (*match_pairs[k].second);
            Hp2 /= Hp2.z();
            auto e12 = (*match_pairs[k].first - Hp2).squaredNorm();
            if (e12 >= th_e2) {
                is_outlier = true;
            }

            Vec3 H_invp1 = H21 * (*match_pairs[k].first);
            H_invp1 /= H_invp1.z();
            auto e21 = (*match_pairs[k].second - H_invp1).squaredNorm();
            if (e21 >= th_e2) {
                is_outlier = true;
            }

            is_outliers_vec[curr_index][k] = is_outlier;
            if (!is_outlier) {
                ++num_inliers_vec[curr_index];
                score += th_score - e12 + th_score - e21;
            }
        }

        if (num_inliers_vec[curr_index] > num_inliers_vec[best_index] ||
            (num_inliers_vec[curr_index] == num_inliers_vec[best_index] && score > best_score)) {
            H = H12;
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


bool slam::faugeras_decomposition(const Mat3_3 &H, vector<Mat3_3> &R_vec, vector<Vec3> &t_vec, vector<Vec3> &n_vec) {
    constexpr static TYPE GAP = TYPE(1.00001);

    // 奇异值分解
    Eigen::JacobiSVD<Mat3_3> H_svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Vec3 sigma = H_svd.singularValues();

    // σ0 = σ1 = σ2
    if(sigma(0) / sigma(1) < GAP && sigma(1) / sigma(2) < GAP) {
        if (H.determinant() <= TYPE(0)) {
            return false;
        }
        R_vec.resize(1);
        t_vec.resize(1);
        n_vec.resize(1);
        R_vec[0] = H / sigma(1);
        t_vec[0].setZero();
        n_vec[0].setZero();
        return true;
    }

    // σ0 ≠ σ2
    sigma /= sigma(1);
    const Mat3_3& V = H_svd.matrixV();
    const Mat3_3& U = H_svd.matrixU();
    auto s = U.determinant() * V.determinant();

    // 4组解
    R_vec.resize(4);
    t_vec.resize(4);
    n_vec.resize(4);
    auto nx = sqrt((sigma(0) * sigma(0) - TYPE(1)) / (sigma(0) * sigma(0) - sigma(2) * sigma(2)));
    auto nz = sqrt((TYPE(1) - sigma(2) * sigma(2)) / (sigma(0) * sigma(0) - sigma(2) * sigma(2)));
    n_vec[0] << nx, 0, nz;
    n_vec[1] << nx, 0, -nz;
    n_vec[2] << -nx, 0, nz;
    n_vec[3] << -nx, 0, -nz;
    if (s > 0) {
        auto cos_theta = (sigma(0) * sigma(2) + 1) / (sigma(0) + sigma(2));
        for (unsigned i = 0; i < 4; ++i) {
            auto sin_theta = (sigma(2) - sigma(0)) * n_vec[i].x() * n_vec[i].z();
            R_vec[i] << cos_theta, TYPE(0), sin_theta, TYPE(0), TYPE(1), TYPE(0), -sin_theta, TYPE(0), cos_theta;
            t_vec[i] << (sigma(0) - sigma(2)) * n_vec[i].x(), TYPE(0), (sigma(2) - sigma(0)) * n_vec[i].z();
        }
    } else {
        auto cos_theta = (sigma(0) * sigma(2) - 1) / (sigma(0) - sigma(2));
        for (unsigned i = 0; i < 4; ++i) {
            auto sin_theta = (sigma(2) + sigma(0)) * n_vec[i].x() * n_vec[i].z();
            R_vec[i] << cos_theta, TYPE(0), sin_theta, TYPE(0), TYPE(-1) ,TYPE(0), sin_theta, TYPE(0), -cos_theta;
            t_vec[i] << (sigma(0) + sigma(2)) * n_vec[i].x(), TYPE(0), (sigma(2) + sigma(0)) * n_vec[i].z();
        }
    }

    for (unsigned i = 0; i < 4; ++i) {
        R_vec[i] = s * U * R_vec[i] * V.transpose();
        t_vec[i] = U * t_vec[i];
        n_vec[i] = V * n_vec[i];
    }

    return true;
}


bool slam::zhang_decomposition(const Mat3_3 &H, vector<Mat3_3> &R_vec, vector<Vec3> &t_vec, vector<Vec3> &n_vec) {
    constexpr static TYPE GAP = TYPE(1.00001);

    // 奇异值分解
    Eigen::JacobiSVD<Mat3_3> H_svd(H, Eigen::ComputeFullV);
    Vec3 sigma = H_svd.singularValues();
    Mat3_3 H_regular = H / sigma(1);

    // σ0 = σ1 = σ2
    if(sigma(0) / sigma(1) < GAP && sigma(1) / sigma(2) < GAP) {
        if (H_regular.determinant() <= TYPE(0)) {
            return false;
        }
        R_vec.resize(1);
        t_vec.resize(1);
        n_vec.resize(1);
        R_vec[0] = H_regular;
        t_vec[0].setZero();
        n_vec[0].setZero();
        return true;
    }

    // σ0 ≠ σ2
    sigma /= sigma(1);
    const Mat3_3& V = H_svd.matrixV();

    // 4组解
    R_vec.resize(4);
    t_vec.resize(4);
    n_vec.resize(4);

    // p ⊥ v1, p1 = ±c0 * v0 + c2 * v2
    TYPE c0 = sqrt((TYPE(1) - sigma(2) * sigma(2)) / (sigma(0) * sigma(0) - sigma(2) * sigma(2)));
    TYPE c2 = sqrt((sigma(0) * sigma(0) - TYPE(1)) / (sigma(0) * sigma(0) - sigma(2) * sigma(2)));
    vector<Vec3> p(2);
    p[0] = c0 * V.col(0) + c2 * V.col(2);
    p[1] = -c0 * V.col(0) + c2 * V.col(2);

    // R = A * B^-1, t = (H - R) * n, n = ±v1 x p
    Mat3_3 A, B;
    A.col(0) = H_regular * V.col(1);
    B.col(0) = V.col(1);

    // case 0
    A.col(1) = H_regular * p[0];
    A.col(2) = A.col(0).cross(A.col(1));
    B.col(1) = p[0];
    B.col(2) = B.col(0).cross(B.col(1));
    R_vec[0].transpose() = B.transpose().partialPivLu().solve(A.transpose());
    n_vec[0] = B.col(2);
    t_vec[0] = (H_regular - R_vec[0]) * n_vec[0];

    // case 1
    A.col(1) = H_regular * p[1];
    A.col(2) = A.col(0).cross(A.col(1));
    B.col(1) = p[1];
    B.col(2) = B.col(0).cross(B.col(1));
    R_vec[1].transpose() = B.transpose().partialPivLu().solve(A.transpose());
    n_vec[1] = B.col(2);
    t_vec[1] = (H_regular - R_vec[1]) * n_vec[1];

    // case 2
    R_vec[2] = R_vec[0];
    n_vec[2] = -n_vec[0];
    t_vec[2] = -t_vec[0];

    // case 3
    R_vec[3] = R_vec[1];
    n_vec[3] = -n_vec[1];
    t_vec[3] = -t_vec[1];

    return true;
}
