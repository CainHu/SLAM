//
// Created by Cain on 2024/9/24.
//

#include <iostream>
#include <random>
#include "common.h"
#include "lib/vision/essential.h"

#define TYPE SLAM_DATA_TYPE
using namespace slam;

int main() {
    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<TYPE> dist(-TYPE(0.5), TYPE(0.5));
    static vector<Vec3> points;
    for (unsigned i = 0; i < 10; ++i) {
        for (unsigned j = 0; j < 10; ++j) {
            points.emplace_back(TYPE(i) - TYPE(4.5) + dist(gen), TYPE(j) - TYPE(4.5) + dist(gen), TYPE(10) + TYPE(10) * dist(gen));
        }
    }

    static Mat3_3 R0 = slam::AngAxis(TYPE(-0.5), Vec3(0, 0, 1)).toRotationMatrix();
    static Vec3 t0(1, 0, 0);
    static Mat3_3 R = slam::AngAxis(TYPE(0.1), Vec3(1, 0, 0)).toRotationMatrix();
    static Vec3 t(0, 1, -0.5);

    static vector<Vec3> points_i(points.size()), points_j(points.size());
    static vector<pair<const Vec3*, const Vec3*>> match_pairs(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        points_i[i] = R0.transpose() * (points[i] - t0);
        points_j[i] = R.transpose() * (points_i[i] - t);
        points_i[i] /= points_i[i].z();
        points_j[i] /= points_j[i].z();

        match_pairs[i].first = &points_i[i];
        match_pairs[i].second = &points_j[i];
    }

    Mat3_3 R_est;
    Vec3 t_est;
    vector<Vec3> points_est;
    vector<bool> is_outliers;
    auto flag = reconstruct_from_essential(match_pairs, R_est, t_est, points, is_outliers, compute_essential_5pts);

    cout << "flag = " << flag << '\n';
    cout << "R = " << R << '\n';
    cout << "R_est = " << R_est << '\n';
    cout << "t = " << t.transpose() << '\n';
    cout << "t_est = " << t_est.transpose() << '\n';

    return 0;
}
