//
// Created by Cain on 2024/9/18.
//

#ifndef SLAM_COMMON_H
#define SLAM_COMMON_H

#include <iostream>
#include <fstream>
#include <string>
#include <memory>
#include <ctime>
#include <cmath>
#include <algorithm>
#include <random>
#include <array>
#include <vector>
#include <list>
#include <queue>
#include <deque>
#include <map>
#include <unordered_map>
#include <set>
#include <unordered_set>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#define SLAM_DATA_TYPE double

namespace slam {
    using namespace std;
    using namespace Eigen;

    // 列优先矩阵
    typedef Eigen::Matrix<SLAM_DATA_TYPE, Eigen::Dynamic, Eigen::Dynamic> MatXX;

    typedef Eigen::Matrix<SLAM_DATA_TYPE, 2, 2> Mat2_2;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 3, 2> Mat3_2;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 4, 2> Mat4_2;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 5, 2> Mat5_2;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 6, 2> Mat6_2;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 7, 2> Mat7_2;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 8, 2> Mat8_2;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 9, 2> Mat9_2;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 10, 2> Mat10_2;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 11, 2> Mat11_2;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 12, 2> Mat12_2;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 13, 2> Mat13_2;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 14, 2> Mat14_2;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 15, 2> Mat15_2;

    typedef Eigen::Matrix<SLAM_DATA_TYPE, 2, 3> Mat2_3;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 3, 3> Mat3_3;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 4, 3> Mat4_3;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 5, 3> Mat5_3;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 6, 3> Mat6_3;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 7, 3> Mat7_3;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 8, 3> Mat8_3;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 9, 3> Mat9_3;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 10, 3> Mat10_3;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 11, 3> Mat11_3;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 12, 3> Mat12_3;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 13, 3> Mat13_3;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 14, 3> Mat14_3;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 15, 3> Mat15_3;

    typedef Eigen::Matrix<SLAM_DATA_TYPE, 2, 4> Mat2_4;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 3, 4> Mat3_4;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 4, 4> Mat4_4;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 5, 4> Mat5_4;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 6, 4> Mat6_4;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 7, 4> Mat7_4;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 8, 4> Mat8_4;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 9, 4> Mat9_4;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 10, 4> Mat10_4;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 11, 4> Mat11_4;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 12, 4> Mat12_4;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 13, 4> Mat13_4;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 14, 4> Mat14_4;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 15, 4> Mat15_4;

    typedef Eigen::Matrix<SLAM_DATA_TYPE, 2, 5> Mat2_5;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 3, 5> Mat3_5;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 4, 5> Mat4_5;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 5, 5> Mat5_5;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 6, 5> Mat6_5;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 7, 5> Mat7_5;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 8, 5> Mat8_5;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 9, 5> Mat9_5;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 10, 5> Mat10_5;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 11, 5> Mat11_5;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 12, 5> Mat12_5;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 13, 5> Mat13_5;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 14, 5> Mat14_5;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 15, 5> Mat15_5;

    typedef Eigen::Matrix<SLAM_DATA_TYPE, 2, 6> Mat2_6;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 3, 6> Mat3_6;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 4, 6> Mat4_6;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 5, 6> Mat5_6;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 6, 6> Mat6_6;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 7, 6> Mat7_6;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 8, 6> Mat8_6;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 9, 6> Mat9_6;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 10, 6> Mat10_6;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 11, 6> Mat11_6;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 12, 6> Mat12_6;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 13, 6> Mat13_6;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 14, 6> Mat14_6;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 15, 6> Mat15_6;

    typedef Eigen::Matrix<SLAM_DATA_TYPE, 2, 7> Mat2_7;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 3, 7> Mat3_7;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 4, 7> Mat4_7;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 5, 7> Mat5_7;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 6, 7> Mat6_7;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 7, 7> Mat7_7;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 8, 7> Mat8_7;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 9, 7> Mat9_7;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 10, 7> Mat10_7;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 11, 7> Mat11_7;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 12, 7> Mat12_7;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 13, 7> Mat13_7;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 14, 7> Mat14_7;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 15, 7> Mat15_7;

    typedef Eigen::Matrix<SLAM_DATA_TYPE, 2, 8> Mat2_8;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 3, 8> Mat3_8;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 4, 8> Mat4_8;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 5, 8> Mat5_8;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 6, 8> Mat6_8;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 7, 8> Mat7_8;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 8, 8> Mat8_8;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 9, 8> Mat9_8;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 10, 8> Mat10_8;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 11, 8> Mat11_8;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 12, 8> Mat12_8;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 13, 8> Mat13_8;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 14, 8> Mat14_8;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 15, 8> Mat15_8;

    typedef Eigen::Matrix<SLAM_DATA_TYPE, 2, 9> Mat2_9;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 3, 9> Mat3_9;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 4, 9> Mat4_9;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 5, 9> Mat5_9;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 6, 9> Mat6_9;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 7, 9> Mat7_9;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 8, 9> Mat8_9;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 9, 9> Mat9_9;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 10, 9> Mat10_9;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 11, 9> Mat11_9;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 12, 9> Mat12_9;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 13, 9> Mat13_9;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 14, 9> Mat14_9;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 15, 9> Mat15_9;

    typedef Eigen::Matrix<SLAM_DATA_TYPE, 2, 10> Mat2_10;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 3, 10> Mat3_10;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 4, 10> Mat4_10;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 5, 10> Mat5_10;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 6, 10> Mat6_10;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 7, 10> Mat7_10;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 8, 10> Mat8_10;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 9, 10> Mat9_10;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 10, 10> Mat10_10;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 11, 10> Mat11_10;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 12, 10> Mat12_10;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 13, 10> Mat13_10;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 14, 10> Mat14_10;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 15, 10> Mat15_10;

    typedef Eigen::Matrix<SLAM_DATA_TYPE, 2, 11> Mat2_11;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 3, 11> Mat3_11;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 4, 11> Mat4_11;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 5, 11> Mat5_11;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 6, 11> Mat6_11;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 7, 11> Mat7_11;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 8, 11> Mat8_11;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 9, 11> Mat9_11;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 10, 11> Mat10_11;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 11, 11> Mat11_11;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 12, 11> Mat12_11;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 13, 11> Mat13_11;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 14, 11> Mat14_11;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 15, 11> Mat15_11;

    typedef Eigen::Matrix<SLAM_DATA_TYPE, 2, 12> Mat2_12;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 3, 12> Mat3_12;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 4, 12> Mat4_12;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 5, 12> Mat5_12;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 6, 12> Mat6_12;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 7, 12> Mat7_12;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 8, 12> Mat8_12;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 9, 12> Mat9_12;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 10, 12> Mat10_12;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 11, 12> Mat11_12;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 12, 12> Mat12_12;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 13, 12> Mat13_12;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 14, 12> Mat14_12;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 15, 12> Mat15_12;

    typedef Eigen::Matrix<SLAM_DATA_TYPE, 2, 13> Mat2_13;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 3, 13> Mat3_13;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 4, 13> Mat4_13;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 5, 13> Mat5_13;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 6, 13> Mat6_13;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 7, 13> Mat7_13;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 8, 13> Mat8_13;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 9, 13> Mat9_13;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 10, 13> Mat10_13;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 11, 13> Mat11_13;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 12, 13> Mat12_13;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 13, 13> Mat13_13;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 14, 13> Mat14_13;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 15, 13> Mat15_13;

    typedef Eigen::Matrix<SLAM_DATA_TYPE, 2, 14> Mat2_14;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 3, 14> Mat3_14;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 4, 14> Mat4_14;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 5, 14> Mat5_14;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 6, 14> Mat6_14;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 7, 14> Mat7_14;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 8, 14> Mat8_14;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 9, 14> Mat9_14;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 10, 14> Mat10_14;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 11, 14> Mat11_14;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 12, 14> Mat12_14;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 13, 14> Mat13_14;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 14, 14> Mat14_14;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 15, 14> Mat15_14;

    typedef Eigen::Matrix<SLAM_DATA_TYPE, 2, 15> Mat2_15;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 3, 15> Mat3_15;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 4, 15> Mat4_15;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 5, 15> Mat5_15;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 6, 15> Mat6_15;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 7, 15> Mat7_15;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 8, 15> Mat8_15;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 9, 15> Mat9_15;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 10, 15> Mat10_15;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 11, 15> Mat11_15;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 12, 15> Mat12_15;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 13, 15> Mat13_15;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 14, 15> Mat14_15;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 15, 15> Mat15_15;

    // 行优先矩阵
    typedef Eigen::Matrix<SLAM_DATA_TYPE, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatXXr;

    typedef Eigen::Matrix<SLAM_DATA_TYPE, 2, 2, Eigen::RowMajor> Mat2_2r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 3, 2, Eigen::RowMajor> Mat3_2r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 4, 2, Eigen::RowMajor> Mat4_2r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 5, 2, Eigen::RowMajor> Mat5_2r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 6, 2, Eigen::RowMajor> Mat6_2r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 7, 2, Eigen::RowMajor> Mat7_2r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 8, 2, Eigen::RowMajor> Mat8_2r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 9, 2, Eigen::RowMajor> Mat9_2r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 10, 2, Eigen::RowMajor> Mat10_2r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 11, 2, Eigen::RowMajor> Mat11_2r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 12, 2, Eigen::RowMajor> Mat12_2r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 13, 2, Eigen::RowMajor> Mat13_2r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 14, 2, Eigen::RowMajor> Mat14_2r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 15, 2, Eigen::RowMajor> Mat15_2r;

    typedef Eigen::Matrix<SLAM_DATA_TYPE, 2, 3, Eigen::RowMajor> Mat2_3r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 3, 3, Eigen::RowMajor> Mat3_3r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 4, 3, Eigen::RowMajor> Mat4_3r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 5, 3, Eigen::RowMajor> Mat5_3r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 6, 3, Eigen::RowMajor> Mat6_3r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 7, 3, Eigen::RowMajor> Mat7_3r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 8, 3, Eigen::RowMajor> Mat8_3r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 9, 3, Eigen::RowMajor> Mat9_3r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 10, 3, Eigen::RowMajor> Mat10_3r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 11, 3, Eigen::RowMajor> Mat11_3r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 12, 3, Eigen::RowMajor> Mat12_3r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 13, 3, Eigen::RowMajor> Mat13_3r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 14, 3, Eigen::RowMajor> Mat14_3r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 15, 3, Eigen::RowMajor> Mat15_3r;

    typedef Eigen::Matrix<SLAM_DATA_TYPE, 2, 4, Eigen::RowMajor> Mat2_4r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 3, 4, Eigen::RowMajor> Mat3_4r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 4, 4, Eigen::RowMajor> Mat4_4r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 5, 4, Eigen::RowMajor> Mat5_4r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 6, 4, Eigen::RowMajor> Mat6_4r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 7, 4, Eigen::RowMajor> Mat7_4r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 8, 4, Eigen::RowMajor> Mat8_4r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 9, 4, Eigen::RowMajor> Mat9_4r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 10, 4, Eigen::RowMajor> Mat10_4r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 11, 4, Eigen::RowMajor> Mat11_4r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 12, 4, Eigen::RowMajor> Mat12_4r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 13, 4, Eigen::RowMajor> Mat13_4r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 14, 4, Eigen::RowMajor> Mat14_4r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 15, 4, Eigen::RowMajor> Mat15_4r;

    typedef Eigen::Matrix<SLAM_DATA_TYPE, 2, 5, Eigen::RowMajor> Mat2_5r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 3, 5, Eigen::RowMajor> Mat3_5r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 4, 5, Eigen::RowMajor> Mat4_5r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 5, 5, Eigen::RowMajor> Mat5_5r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 6, 5, Eigen::RowMajor> Mat6_5r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 7, 5, Eigen::RowMajor> Mat7_5r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 8, 5, Eigen::RowMajor> Mat8_5r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 9, 5, Eigen::RowMajor> Mat9_5r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 10, 5, Eigen::RowMajor> Mat10_5r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 11, 5, Eigen::RowMajor> Mat11_5r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 12, 5, Eigen::RowMajor> Mat12_5r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 13, 5, Eigen::RowMajor> Mat13_5r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 14, 5, Eigen::RowMajor> Mat14_5r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 15, 5, Eigen::RowMajor> Mat15_5r;

    typedef Eigen::Matrix<SLAM_DATA_TYPE, 2, 6, Eigen::RowMajor> Mat2_6r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 3, 6, Eigen::RowMajor> Mat3_6r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 4, 6, Eigen::RowMajor> Mat4_6r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 5, 6, Eigen::RowMajor> Mat5_6r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 6, 6, Eigen::RowMajor> Mat6_6r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 7, 6, Eigen::RowMajor> Mat7_6r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 8, 6, Eigen::RowMajor> Mat8_6r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 9, 6, Eigen::RowMajor> Mat9_6r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 10, 6, Eigen::RowMajor> Mat10_6r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 11, 6, Eigen::RowMajor> Mat11_6r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 12, 6, Eigen::RowMajor> Mat12_6r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 13, 6, Eigen::RowMajor> Mat13_6r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 14, 6, Eigen::RowMajor> Mat14_6r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 15, 6, Eigen::RowMajor> Mat15_6r;

    typedef Eigen::Matrix<SLAM_DATA_TYPE, 2, 7, Eigen::RowMajor> Mat2_7r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 3, 7, Eigen::RowMajor> Mat3_7r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 4, 7, Eigen::RowMajor> Mat4_7r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 5, 7, Eigen::RowMajor> Mat5_7r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 6, 7, Eigen::RowMajor> Mat6_7r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 7, 7, Eigen::RowMajor> Mat7_7r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 8, 7, Eigen::RowMajor> Mat8_7r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 9, 7, Eigen::RowMajor> Mat9_7r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 10, 7, Eigen::RowMajor> Mat10_7r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 11, 7, Eigen::RowMajor> Mat11_7r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 12, 7, Eigen::RowMajor> Mat12_7r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 13, 7, Eigen::RowMajor> Mat13_7r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 14, 7, Eigen::RowMajor> Mat14_7r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 15, 7, Eigen::RowMajor> Mat15_7r;

    typedef Eigen::Matrix<SLAM_DATA_TYPE, 2, 8, Eigen::RowMajor> Mat2_8r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 3, 8, Eigen::RowMajor> Mat3_8r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 4, 8, Eigen::RowMajor> Mat4_8r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 5, 8, Eigen::RowMajor> Mat5_8r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 6, 8, Eigen::RowMajor> Mat6_8r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 7, 8, Eigen::RowMajor> Mat7_8r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 8, 8, Eigen::RowMajor> Mat8_8r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 9, 8, Eigen::RowMajor> Mat9_8r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 10, 8, Eigen::RowMajor> Mat10_8r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 11, 8, Eigen::RowMajor> Mat11_8r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 12, 8, Eigen::RowMajor> Mat12_8r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 13, 8, Eigen::RowMajor> Mat13_8r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 14, 8, Eigen::RowMajor> Mat14_8r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 15, 8, Eigen::RowMajor> Mat15_8r;

    typedef Eigen::Matrix<SLAM_DATA_TYPE, 2, 9, Eigen::RowMajor> Mat2_9r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 3, 9, Eigen::RowMajor> Mat3_9r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 4, 9, Eigen::RowMajor> Mat4_9r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 5, 9, Eigen::RowMajor> Mat5_9r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 6, 9, Eigen::RowMajor> Mat6_9r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 7, 9, Eigen::RowMajor> Mat7_9r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 8, 9, Eigen::RowMajor> Mat8_9r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 9, 9, Eigen::RowMajor> Mat9_9r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 10, 9, Eigen::RowMajor> Mat10_9r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 11, 9, Eigen::RowMajor> Mat11_9r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 12, 9, Eigen::RowMajor> Mat12_9r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 13, 9, Eigen::RowMajor> Mat13_9r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 14, 9, Eigen::RowMajor> Mat14_9r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 15, 9, Eigen::RowMajor> Mat15_9r;

    typedef Eigen::Matrix<SLAM_DATA_TYPE, 2, 10, Eigen::RowMajor> Mat2_10r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 3, 10, Eigen::RowMajor> Mat3_10r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 4, 10, Eigen::RowMajor> Mat4_10r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 5, 10, Eigen::RowMajor> Mat5_10r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 6, 10, Eigen::RowMajor> Mat6_10r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 7, 10, Eigen::RowMajor> Mat7_10r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 8, 10, Eigen::RowMajor> Mat8_10r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 9, 10, Eigen::RowMajor> Mat9_10r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 10, 10, Eigen::RowMajor> Mat10_10r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 11, 10, Eigen::RowMajor> Mat11_10r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 12, 10, Eigen::RowMajor> Mat12_10r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 13, 10, Eigen::RowMajor> Mat13_10r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 14, 10, Eigen::RowMajor> Mat14_10r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 15, 10, Eigen::RowMajor> Mat15_10r;

    typedef Eigen::Matrix<SLAM_DATA_TYPE, 2, 11, Eigen::RowMajor> Mat2_11r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 3, 11, Eigen::RowMajor> Mat3_11r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 4, 11, Eigen::RowMajor> Mat4_11r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 5, 11, Eigen::RowMajor> Mat5_11r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 6, 11, Eigen::RowMajor> Mat6_11r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 7, 11, Eigen::RowMajor> Mat7_11r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 8, 11, Eigen::RowMajor> Mat8_11r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 9, 11, Eigen::RowMajor> Mat9_11r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 10, 11, Eigen::RowMajor> Mat10_11r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 11, 11, Eigen::RowMajor> Mat11_11r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 12, 11, Eigen::RowMajor> Mat12_11r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 13, 11, Eigen::RowMajor> Mat13_11r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 14, 11, Eigen::RowMajor> Mat14_11r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 15, 11, Eigen::RowMajor> Mat15_11r;

    typedef Eigen::Matrix<SLAM_DATA_TYPE, 2, 12, Eigen::RowMajor> Mat2_12r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 3, 12, Eigen::RowMajor> Mat3_12r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 4, 12, Eigen::RowMajor> Mat4_12r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 5, 12, Eigen::RowMajor> Mat5_12r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 6, 12, Eigen::RowMajor> Mat6_12r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 7, 12, Eigen::RowMajor> Mat7_12r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 8, 12, Eigen::RowMajor> Mat8_12r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 9, 12, Eigen::RowMajor> Mat9_12r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 10, 12, Eigen::RowMajor> Mat10_12r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 11, 12, Eigen::RowMajor> Mat11_12r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 12, 12, Eigen::RowMajor> Mat12_12r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 13, 12, Eigen::RowMajor> Mat13_12r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 14, 12, Eigen::RowMajor> Mat14_12r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 15, 12, Eigen::RowMajor> Mat15_12r;

    typedef Eigen::Matrix<SLAM_DATA_TYPE, 2, 13, Eigen::RowMajor> Mat2_13r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 3, 13, Eigen::RowMajor> Mat3_13r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 4, 13, Eigen::RowMajor> Mat4_13r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 5, 13, Eigen::RowMajor> Mat5_13r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 6, 13, Eigen::RowMajor> Mat6_13r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 7, 13, Eigen::RowMajor> Mat7_13r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 8, 13, Eigen::RowMajor> Mat8_13r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 9, 13, Eigen::RowMajor> Mat9_13r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 10, 13, Eigen::RowMajor> Mat10_13r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 11, 13, Eigen::RowMajor> Mat11_13r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 12, 13, Eigen::RowMajor> Mat12_13r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 13, 13, Eigen::RowMajor> Mat13_13r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 14, 13, Eigen::RowMajor> Mat14_13r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 15, 13, Eigen::RowMajor> Mat15_13r;

    typedef Eigen::Matrix<SLAM_DATA_TYPE, 2, 14, Eigen::RowMajor> Mat2_14r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 3, 14, Eigen::RowMajor> Mat3_14r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 4, 14, Eigen::RowMajor> Mat4_14r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 5, 14, Eigen::RowMajor> Mat5_14r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 6, 14, Eigen::RowMajor> Mat6_14r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 7, 14, Eigen::RowMajor> Mat7_14r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 8, 14, Eigen::RowMajor> Mat8_14r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 9, 14, Eigen::RowMajor> Mat9_14r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 10, 14, Eigen::RowMajor> Mat10_14r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 11, 14, Eigen::RowMajor> Mat11_14r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 12, 14, Eigen::RowMajor> Mat12_14r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 13, 14, Eigen::RowMajor> Mat13_14r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 14, 14, Eigen::RowMajor> Mat14_14r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 15, 14, Eigen::RowMajor> Mat15_14r;

    typedef Eigen::Matrix<SLAM_DATA_TYPE, 2, 15, Eigen::RowMajor> Mat2_15r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 3, 15, Eigen::RowMajor> Mat3_15r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 4, 15, Eigen::RowMajor> Mat4_15r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 5, 15, Eigen::RowMajor> Mat5_15r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 6, 15, Eigen::RowMajor> Mat6_15r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 7, 15, Eigen::RowMajor> Mat7_15r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 8, 15, Eigen::RowMajor> Mat8_15r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 9, 15, Eigen::RowMajor> Mat9_15r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 10, 15, Eigen::RowMajor> Mat10_15r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 11, 15, Eigen::RowMajor> Mat11_15r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 12, 15, Eigen::RowMajor> Mat12_15r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 13, 15, Eigen::RowMajor> Mat13_15r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 14, 15, Eigen::RowMajor> Mat14_15r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 15, 15, Eigen::RowMajor> Mat15_15r;

    // 列向量
    typedef Eigen::Matrix<SLAM_DATA_TYPE, Eigen::Dynamic, 1> VecX;

    typedef Eigen::Matrix<SLAM_DATA_TYPE, 1, 1> Vec1;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 2, 1> Vec2;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 3, 1> Vec3;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 4, 1> Vec4;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 5, 1> Vec5;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 6, 1> Vec6;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 7, 1> Vec7;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 8, 1> Vec8;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 9, 1> Vec9;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 10, 1> Vec10;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 11, 1> Vec11;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 12, 1> Vec12;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 13, 1> Vec13;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 14, 1> Vec14;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 15, 1> Vec15;

    // 行向量
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 1, Eigen::Dynamic, Eigen::RowMajor> VecXr;

    typedef Eigen::Matrix<SLAM_DATA_TYPE, 1, 1, Eigen::RowMajor> Vec1r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 1, 2, Eigen::RowMajor> Vec2r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 1, 3, Eigen::RowMajor> Vec3r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 1, 4, Eigen::RowMajor> Vec4r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 1, 5, Eigen::RowMajor> Vec5r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 1, 6, Eigen::RowMajor> Vec6r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 1, 7, Eigen::RowMajor> Vec7r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 1, 8, Eigen::RowMajor> Vec8r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 1, 9, Eigen::RowMajor> Vec9r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 1, 10, Eigen::RowMajor> Vec10r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 1, 11, Eigen::RowMajor> Vec11r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 1, 12, Eigen::RowMajor> Vec12r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 1, 13, Eigen::RowMajor> Vec13r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 1, 14, Eigen::RowMajor> Vec14r;
    typedef Eigen::Matrix<SLAM_DATA_TYPE, 1, 15, Eigen::RowMajor> Vec15r;

    // 四元数
    typedef Eigen::Quaternion<SLAM_DATA_TYPE> Quat;
    typedef Eigen::AngleAxis<SLAM_DATA_TYPE> AngAxis;

    // 向量数组
    typedef std::vector<Vec2, Eigen::aligned_allocator<Vec2>> VecVec2;
    typedef std::vector<Vec3, Eigen::aligned_allocator<Vec3>> VecVec3;
    typedef std::vector<Vec2, Eigen::aligned_allocator<Vec2r>> VecVec2r;
    typedef std::vector<Vec3, Eigen::aligned_allocator<Vec3r>> VecVec3r;

    // Map of Eigen matrix
    typedef std::map<unsigned long, MatXX, std::less<>, Eigen::aligned_allocator<MatXX>> MapMatXX;
}

#endif //SLAM_COMMON_H
