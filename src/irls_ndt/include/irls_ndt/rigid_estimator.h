/*
 * @Descripttion: 
 * @version: 
 * @Author: Yuhang Li
 * @Date: 2022-09-17 20:00:34
 * @LastEditors: Yuhang Li
 * @LastEditTime: 2022-09-17 20:14:09
 */
// This file contains different implementations of the se(3) estimator.
#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <vector>

namespace irls_ndt {

inline Eigen::Matrix3d skew(Eigen::Vector3d& p) {
    Eigen::Matrix3d Skew;
    Skew << 0, -p(2), p(1), p(2), 0, -p(0), -p(1), p(0), 0;
    return Skew;
}
//// odinary algorithms.
void computeHessianOrdinary(const Eigen::Matrix3Xd& V,
                            const Eigen::Matrix3Xd& Aft,
                            const Eigen::VectorXd& W,
                            const Eigen::MatrixXd& MArray,
                            Eigen::MatrixXd& H,
                            Eigen::VectorXd& b);
// vectorized algorithm.
void computeHessianVectorized(const Eigen::Matrix3Xd& V,
                              const Eigen::Matrix3Xd& Aft,
                              const Eigen::VectorXd& W,
                              const Eigen::MatrixXd& MArray,
                              Eigen::MatrixXd& H,
                              Eigen::VectorXd& b);

}  // namespace irls_ndt
