//
// Created by Cain on 2024/9/24.
//

#include "essential.h"
#include "triangulate.h"

#define TYPE SLAM_DATA_TYPE

unsigned long slam::compute_essential_8pts(const vector<pair<const Vec3*, const Vec3*>> &match_pairs,
                                           Mat3_3 &E, vector<bool> &is_outliers,
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

        Vec9 f = D_svd.matrixV().col(8);
        Eigen::Map<Mat3_3> F(f.data());

        // 由基础矩阵计算本质矩阵
        Mat3_3 E_raw = Ti.transpose() * F * Tj;

        // 添加本质矩阵的奇异值约束
        Eigen::JacobiSVD<Mat3_3 > E_svd(E_raw, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Vec3 s = E_svd.singularValues();
        s(0) = 0.5 * (s(0) + s(1));
        s(1) = s(0);
        s(2) = 0.;
        Mat3_3 E12 = E_svd.matrixU() * s.asDiagonal() * E_svd.matrixV().transpose();

        // 遍历所有的点计算分数
        auto score = TYPE(0);
        num_inliers_vec[curr_index] = 0;
        for (unsigned long k = 0; k < num_points; ++k) {
            bool is_outlier = is_outliers[k];

            Vec3 Ep1 = E12 * *match_pairs[k].first;
            Vec3 Ep2 = E12 * *match_pairs[k].second;
            auto p1Ep2 = match_pairs[k].first->dot(Ep2);
            auto num = p1Ep2 * p1Ep2;

            auto e12 = num / Ep2.head<2>().squaredNorm();
            if (e12 >= th_e2) {
                is_outlier = true;
            }

            auto e21 = num / Ep1.head<2>().squaredNorm();
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
            E = E12;
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


unsigned long slam::compute_essential_7pts(const vector<pair<const Vec3*, const Vec3*>> &match_pairs,
                                           Mat3_3 &E, vector<bool> &is_outliers,
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
        for (auto &F : results) {
            // 由基础矩阵计算本质矩阵
            Mat3_3 E_raw = Ti.transpose() * F * Tj;

            // 添加本质矩阵的奇异值约束
            Eigen::JacobiSVD<Mat3_3 > E_svd(E_raw, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Vec3 s = E_svd.singularValues();
            s(0) = 0.5 * (s(0) + s(1));
            s(1) = s(0);
            s(2) = 0.;
            Mat3_3 E12 = E_svd.matrixU() * s.asDiagonal() * E_svd.matrixV().transpose();

            // 遍历所有的点计算分数
            auto score = TYPE(0);
            num_inliers_vec[curr_index] = 0;
            for (unsigned long k = 0; k < num_points; ++k) {
                bool is_outlier = is_outliers[k];

                Vec3 Ep1 = E12 * *match_pairs[k].first;
                Vec3 Ep2 = E12 * *match_pairs[k].second;
                auto p1Ep2 = match_pairs[k].first->dot(Ep2);
                auto num = p1Ep2 * p1Ep2;

                auto e12 = num / Ep2.head<2>().squaredNorm();
                if (e12 >= th_e2) {
                    is_outlier = true;
                }

                auto e21 = num / Ep1.head<2>().squaredNorm();
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
                E = E12;
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


class Polynomial {
public:
    enum GRevLexMonomials {
        XXX = 0, XXY = 1, XXZ = 2, XYY = 3, XYZ = 4, XZZ = 5, YYY = 6, YYZ = 7, YZZ = 8, ZZZ = 9,
        XX = 10, XY = 11, XZ = 12, YY = 13, YZ = 14, ZZ = 15, X = 16, Y = 17, Z = 18, I = 19
    };

    unsigned int ordering = 1;
    Eigen::Matrix<TYPE, 20, 1> v;

    explicit Polynomial(const Eigen::Matrix<TYPE, 20, 1> &coefficients_, unsigned int ordering_=1) :
            ordering(ordering_), v(coefficients_) {
    }

public:
    Polynomial() :
            Polynomial(Eigen::Matrix<TYPE, 20, 1>::Zero()) {
    }

    explicit Polynomial(TYPE w) {
        v.setZero();
        v[I] = w;
    }

    Polynomial(TYPE x, TYPE y, TYPE z, TYPE w) {
        v.setZero();
        v[X] = x;
        v[Y] = y;
        v[Z] = z;
        v[I] = w;
    }

    void set_xyzw(TYPE x, TYPE y, TYPE z, TYPE w) {
        v.setZero();
        v[X] = x;
        v[Y] = y;
        v[Z] = z;
        v[I] = w;
    }

    Polynomial operator-() const {
        return Polynomial(-v, ordering);
    }

    Polynomial operator+(const Polynomial &b) const {
        return Polynomial(v + b.v, std::max(ordering, b.ordering));
    }

    Polynomial operator-(const Polynomial &b) const {
        return Polynomial(v - b.v, std::max(ordering, b.ordering));
    }

    Polynomial operator*(const Polynomial &b) const {
        Polynomial r;

        if (ordering == 1 && b.ordering == 1) {
            r.v[I] = v[I] * b.v[I];

            r.v[Z] = v[I] * b.v[Z] + v[Z] * b.v[I];
            r.v[Y] = v[I] * b.v[Y] + v[Y] * b.v[I];
            r.v[X] = v[I] * b.v[X] + v[X] * b.v[I];

            r.v[ZZ] = v[Z] * b.v[Z];
            r.v[YZ] = v[Z] * b.v[Y] + v[Y] * b.v[Z];
            r.v[XZ] = v[Z] * b.v[X] + v[X] * b.v[Z];
            r.v[YY] = v[Y] * b.v[Y] + v[YY] * b.v[I];
            r.v[XY] = v[Y] * b.v[X] + v[X] * b.v[Y];
            r.v[XX] = v[X] * b.v[X];
        } else if (ordering == 2 && b.ordering == 1) {
            r.v[I] = v[I] * b.v[I];

            r.v[Z] = v[I] * b.v[Z] + v[Z] * b.v[I];
            r.v[Y] = v[I] * b.v[Y] + v[Y] * b.v[I];
            r.v[X] = v[I] * b.v[X] + v[X] * b.v[I];

            r.v[ZZ] = v[Z] * b.v[Z] + v[ZZ] * b.v[I];
            r.v[YZ] = v[Z] * b.v[Y] + v[Y] * b.v[Z] + v[YZ] * b.v[I];
            r.v[XZ] = v[Z] * b.v[X] + v[X] * b.v[Z] + v[XZ] * b.v[I];
            r.v[YY] = v[Y] * b.v[Y] + v[YY] * b.v[I];
            r.v[XY] = v[Y] * b.v[X] + v[X] * b.v[Y] + v[XY] * b.v[I];
            r.v[XX] = v[X] * b.v[X] + v[XX] * b.v[I];

            r.v[ZZZ] = v[ZZ] * b.v[Z];
            r.v[YZZ] = v[ZZ] * b.v[Y] + v[YZ] * b.v[Z];
            r.v[XZZ] = v[ZZ] * b.v[X] + v[XZ] * b.v[Z];
            r.v[YYZ] = v[YZ] * b.v[Y] + v[YY] * b.v[Z];
            r.v[XYZ] = v[YZ] * b.v[X] + v[XZ] * b.v[Y] + v[XY] * b.v[Z];
            r.v[XXZ] = v[XZ] * b.v[X] + v[XX] * b.v[Z];
            r.v[YYY] = v[YY] * b.v[Y];
            r.v[XYY] = v[YY] * b.v[X] + v[XY] * b.v[Y];
            r.v[XXY] = v[XY] * b.v[X] + v[XX] * b.v[Y];
            r.v[XXX] = v[XX] * b.v[X];
        } else if (ordering == 1 && b.ordering == 2) {
            r.v[I] = v[I] * b.v[I];

            r.v[Z] = v[I] * b.v[Z] + v[Z] * b.v[I];
            r.v[Y] = v[I] * b.v[Y] + v[Y] * b.v[I];
            r.v[X] = v[I] * b.v[X] + v[X] * b.v[I];

            r.v[ZZ] = v[I] * b.v[ZZ] + v[Z] * b.v[Z];
            r.v[YZ] = v[I] * b.v[YZ] + v[Z] * b.v[Y] + v[Y] * b.v[Z];
            r.v[XZ] = v[I] * b.v[XZ] + v[Z] * b.v[X] + v[X] * b.v[Z];
            r.v[YY] = v[I] * b.v[YY] + v[Y] * b.v[Y];
            r.v[XY] = v[I] * b.v[XY] + v[Y] * b.v[X] + v[X] * b.v[Y];
            r.v[XX] = v[I] * b.v[XX] + v[X] * b.v[X];

            r.v[ZZZ] = v[Z] * b.v[ZZ];
            r.v[YZZ] = v[Z] * b.v[YZ] + v[Y] * b.v[ZZ];
            r.v[XZZ] = v[Z] * b.v[XZ] + v[X] * b.v[ZZ];
            r.v[YYZ] = v[Z] * b.v[YY] + v[Y] * b.v[YZ];
            r.v[XYZ] = v[Z] * b.v[XY] + v[Y] * b.v[XZ] + v[X] * b.v[YZ];
            r.v[XXZ] = v[Z] * b.v[XX] + v[X] * b.v[XZ];
            r.v[YYY] = v[Y] * b.v[YY] + v[YY] * b.v[Y];
            r.v[XYY] = v[Y] * b.v[XY] + v[X] * b.v[YY];
            r.v[XXY] = v[Y] * b.v[XX] + v[X] * b.v[XY];
            r.v[XXX] = v[X] * b.v[XX];
        } else {
            r.v[I] = v[I] * b.v[I];

            r.v[Z] = v[I] * b.v[Z] + v[Z] * b.v[I];
            r.v[Y] = v[I] * b.v[Y] + v[Y] * b.v[I];
            r.v[X] = v[I] * b.v[X] + v[X] * b.v[I];

            r.v[ZZ] = v[I] * b.v[ZZ] + v[Z] * b.v[Z] + v[ZZ] * b.v[I];
            r.v[YZ] = v[I] * b.v[YZ] + v[Z] * b.v[Y] + v[Y] * b.v[Z] + v[YZ] * b.v[I];
            r.v[XZ] = v[I] * b.v[XZ] + v[Z] * b.v[X] + v[X] * b.v[Z] + v[XZ] * b.v[I];
            r.v[YY] = v[I] * b.v[YY] + v[Y] * b.v[Y] + v[YY] * b.v[I];
            r.v[XY] = v[I] * b.v[XY] + v[Y] * b.v[X] + v[X] * b.v[Y] + v[XY] * b.v[I];
            r.v[XX] = v[I] * b.v[XX] + v[X] * b.v[X] + v[XX] * b.v[I];

            r.v[ZZZ] = v[I] * b.v[ZZZ] + v[Z] * b.v[ZZ] + v[ZZ] * b.v[Z] + v[ZZZ] * b.v[I];
            r.v[YZZ] = v[I] * b.v[YZZ] + v[Z] * b.v[YZ] + v[Y] * b.v[ZZ] + v[ZZ] * b.v[Y] + v[YZ] * b.v[Z] + v[YZZ] * b.v[I];
            r.v[XZZ] = v[I] * b.v[XZZ] + v[Z] * b.v[XZ] + v[X] * b.v[ZZ] + v[ZZ] * b.v[X] + v[XZ] * b.v[Z] + v[XZZ] * b.v[I];
            r.v[YYZ] = v[I] * b.v[YYZ] + v[Z] * b.v[YY] + v[Y] * b.v[YZ] + v[YZ] * b.v[Y] + v[YY] * b.v[Z] + v[YYZ] * b.v[I];
            r.v[XYZ] = v[I] * b.v[XYZ] + v[Z] * b.v[XY] + v[Y] * b.v[XZ] + v[X] * b.v[YZ] + v[YZ] * b.v[X] + v[XZ] * b.v[Y] + v[XY] * b.v[Z] + v[XYZ] * b.v[I];
            r.v[XXZ] = v[I] * b.v[XXZ] + v[Z] * b.v[XX] + v[X] * b.v[XZ] + v[XZ] * b.v[X] + v[XX] * b.v[Z] + v[XXZ] * b.v[I];
            r.v[YYY] = v[I] * b.v[YYY] + v[Y] * b.v[YY] + v[YY] * b.v[Y] + v[YYY] * b.v[I];
            r.v[XYY] = v[I] * b.v[XYY] + v[Y] * b.v[XY] + v[X] * b.v[YY] + v[YY] * b.v[X] + v[XY] * b.v[Y] + v[XYY] * b.v[I];
            r.v[XXY] = v[I] * b.v[XXY] + v[Y] * b.v[XX] + v[X] * b.v[XY] + v[XY] * b.v[X] + v[XX] * b.v[Y] + v[XXY] * b.v[I];
            r.v[XXX] = v[I] * b.v[XXX] + v[X] * b.v[XX] + v[XX] * b.v[X] + v[XXX] * b.v[I];
        }

        r.ordering = ordering + b.ordering;

        return r;
    }

    [[nodiscard]] const Eigen::Matrix<TYPE, 20, 1> &coefficients() const {
        return v;
    }
};

Polynomial operator*(const TYPE &scale, const Polynomial &poly) {
    return Polynomial(scale * poly.coefficients(), poly.ordering);
}

unsigned long slam::compute_essential_5pts(const vector<pair<const Vec3*, const Vec3*>> &match_pairs,
                                           Mat3_3 &E, vector<bool> &is_outliers,
                                           TYPE sigma, TYPE confidence, unsigned int max_iters) {
    constexpr static unsigned int N = 5;
    auto sigma2 = sigma * sigma;
    auto th_e2 = TYPE(3.841) * sigma2;
    auto th_score = TYPE(5.991) * sigma2;
    TYPE ransac_k = log(max(TYPE(1) - confidence, TYPE(1e-5)));

    size_t num_points = match_pairs.size();
    if (is_outliers.size() != num_points) {
        is_outliers.resize(num_points, false);
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
        // 五点法
        Eigen::Matrix<TYPE, N, 9, Eigen::RowMajor> D;
        auto &&point_indices_set = generator_indices_set();

        // D * e = 0
        for (unsigned int k = 0; k < N; ++k) {
            unsigned int index = point_indices_set[k];
            auto u1 = match_pairs[index].first->x();
            auto v1 = match_pairs[index].first->y();
            auto u2 = match_pairs[index].second->x();
            auto v2 = match_pairs[index].second->y();

            // p1' * e * p2
            // e = [e0, e3, e6
            //      e1, e4, e7
            //      e2, e5, e8]
            D.row(k) << u1 * u2, v1 * u2, u2, u1 * v2, v1 * v2, v2, u1, v1, TYPE(1);
        }

        // 所有可能的本质矩阵
        vector<Mat3_3> results;
        results.reserve(10);

        // 计算 D*e = 0 的 nullspace
        auto D_svd = D.jacobiSvd(Eigen::ComputeFullV);
        if (D_svd.rank() < N) {
            continue;
        }

        Mat9_4 nullspace = D_svd.matrixV().rightCols<4>();
        auto Ex = Eigen::Map<Mat3_3>(nullspace.data());
        auto Ey = Eigen::Map<Mat3_3>(nullspace.data() + 9);
        auto Ez = Eigen::Map<Mat3_3>(nullspace.data() + 18);
        auto Ew = Eigen::Map<Mat3_3>(nullspace.data() + 27);

        // E = x*Ex + y*Ey + z*Ez + w*Ew, w = 1
        Eigen::Matrix<Polynomial, 3, 3> Epoly;
        Epoly << Polynomial(Ex(0, 0), Ey(0, 0), Ez(0, 0), Ew(0, 0)), Polynomial(Ex(0, 1), Ey(0, 1), Ez(0, 1), Ew(0, 1)), Polynomial(Ex(0, 2), Ey(0, 2), Ez(0, 2), Ew(0, 2)),
                 Polynomial(Ex(1, 0), Ey(1, 0), Ez(1, 0), Ew(1, 0)), Polynomial(Ex(1, 1), Ey(1, 1), Ez(1, 1), Ew(1, 1)), Polynomial(Ex(1, 2), Ey(1, 2), Ez(1, 2), Ew(1, 2)),
                 Polynomial(Ex(2, 0), Ey(2, 0), Ez(2, 0), Ew(2, 0)), Polynomial(Ex(2, 1), Ey(2, 1), Ez(2, 1), Ew(2, 1)), Polynomial(Ex(2, 2), Ey(2, 2), Ez(2, 2), Ew(2, 2));

        // 约束矩阵(行优先, 加速赋值)
        Eigen::Matrix<TYPE, 10, 20, Eigen::RowMajor> polynomials;

        // E*E^T*E - 0.5*trace(E*E^T)*E = 0
        Eigen::Matrix<Polynomial, 3, 3> EEt = Epoly * Epoly.transpose();
        Eigen::Matrix<Polynomial, 3, 3> singular_value_constraints = (EEt * Epoly) - (0.5 * EEt.trace()) * Epoly;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                polynomials.row(i * 3 + j) = singular_value_constraints(i, j).coefficients();
            }
        }

        // det(E) = 0
        Polynomial detE = Epoly.determinant();
        polynomials.row(9) = detE.coefficients();

        // 求广义逆 C = [A, B]  ->  [I A^-1 * B]
//        auto p_lu = polynomials.topLeftCorner<10, 10>().partialPivLu();
//        Mat10_10 p_inv = p_lu.inverse();
//        Mat10_10 B;
//        B.topLeftCorner<6, 10>() = -p_inv.topLeftCorner<6, 10>() * polynomials.bottomRightCorner<10, 10>();
        auto p_lu = polynomials.topLeftCorner<10, 10>().fullPivLu();
        Mat10_10 B = p_lu.solve(-polynomials.bottomRightCorner<10, 10>());
        B.row(6) = Vec10::Unit(Polynomial::XX - Polynomial::XX).transpose();
        B.row(7) = Vec10::Unit(Polynomial::XY - Polynomial::XX).transpose();
        B.row(8) = Vec10::Unit(Polynomial::XZ - Polynomial::XX).transpose();
        B.row(9) = Vec10::Unit(Polynomial::X - Polynomial::XX).transpose();

        // 特征分解
        Eigen::EigenSolver<Mat10_10> eigen_solver(B, true);
        Eigen::Matrix<std::complex<TYPE>, 10, 1> xs = eigen_solver.eigenvalues();

        if (eigen_solver.info() == Eigen::Success) {
            // Essential矩阵计算
            for (int i = 0; i < 10; ++i) {
                if (abs(xs[i].imag()) < TYPE(1e-10)) {
                    Vec10 h = eigen_solver.eigenvectors().col(i).real();
                    TYPE xw = h(Polynomial::X - Polynomial::XX);
                    TYPE yw = h(Polynomial::Y - Polynomial::XX);
                    TYPE zw = h(Polynomial::Z - Polynomial::XX);
                    TYPE w = h(Polynomial::I - Polynomial::XX);
                    if(w > TYPE(1e-10)) {
                        Vec9 e = nullspace * Eigen::Vector3d(xw / w, yw / w, zw / w).homogeneous();
                        results.emplace_back(e.data());
                    }
                }
            }
        }

        // 遍历所有可能的结果
        for (auto &E12 : results) {
            // 遍历所有的点计算分数
            auto score = TYPE(0);
            num_inliers_vec[curr_index] = 0;
            for (unsigned long k = 0; k < num_points; ++k) {
                bool is_outlier = is_outliers[k];

                Vec3 Ep1 = E12 * *match_pairs[k].first;
                Vec3 Ep2 = E12 * *match_pairs[k].second;
                auto p1Ep2 = match_pairs[k].first->dot(Ep2);
                auto num = p1Ep2 * p1Ep2;

                auto e12 = num / Ep2.head<2>().squaredNorm();
                if (e12 >= th_e2) {
                    is_outlier = true;
                }

                auto e21 = num / Ep1.head<2>().squaredNorm();
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
                E = E12;
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

bool slam::essential_decomposition(const Mat3_3 &E, vector<Mat3_3> &R_vec, vector<Vec3> &t_vec) {
    // 从E中还原出4组可能的R, t
    R_vec.resize(4);
    t_vec.resize(4);
    Eigen::JacobiSVD<Mat3_3> E_svd(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Mat3_3 V = E_svd.matrixV();
    Mat3_3 U1 = E_svd.matrixU();

    if (V.determinant() < TYPE(0)) {
        V *= TYPE(-1);
    }
    if (U1.determinant() < TYPE(0)) {
        U1 *= TYPE(-1);
    }

    t_vec[0] = U1.col(2);
    t_vec[1] = -t_vec[0];
    t_vec[2] = t_vec[0];
    t_vec[3] = t_vec[1];

    U1.col(0).swap(U1.col(1));
    Mat3_3 U2 = U1;
    U1.col(1) *= TYPE(-1);
    U2.col(0) *= TYPE(-1);

    R_vec[0] = U1 * V.transpose();
    R_vec[1] = R_vec[0];
    R_vec[2] = U2 * V.transpose();
    R_vec[3] = R_vec[2];

    return true;
}

bool slam::reconstruct_from_essential(const vector<pair<const Vec3*, const Vec3*>> &match_pairs,
                                      Mat3_3 &R, Vec3 &t, vector<Vec3> &points, vector<bool> &is_outliers,
                                      unsigned long (*solve)(const vector<pair<const Vec3*, const Vec3*>>&,
                                                             Mat3_3&, vector<bool>&, TYPE, TYPE, unsigned int),
                                      TYPE sigma, TYPE confidence, unsigned int max_iters) {
    constexpr static unsigned int TH_COUNT = 12;
    unsigned long num_points = match_pairs.size();

    // 匹配点太少
    if (num_points < TH_COUNT) {
        return false;
    }

    // 计算本质矩阵
    Mat3_3 E;
    auto num_inliers = solve(match_pairs, E, is_outliers, sigma, confidence, max_iters);
    if (num_inliers < TH_COUNT) {
        return false;
    }

    // 单应矩阵分解
    vector<Mat3_3> R_vec;
    vector<Vec3> t_vec;
    if (!essential_decomposition(E, R_vec, t_vec)) {
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

    return true;
}