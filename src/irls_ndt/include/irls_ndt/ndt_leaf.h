#pragma once

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <unordered_map>

namespace irls_ndt {
/**
 * @class NDTLeaf
 * @brief simple structure storing a centroid and inverse covariance matrix
 */
struct NDTLeaf {
    typedef NDTLeaf* Ptr;
    typedef const NDTLeaf* ConstPtr;
    NDTLeaf() {}
    /**
     * @brief getters
     */
    Eigen::Vector3d getMean() const { return _mean; }
    int getPointCount() const { return _nr_points; }
    const Eigen::Matrix3d& getEvecs() const { return _evecs; }
    const Eigen::Vector3d& getEvals() const { return _evals; }
    const Eigen::Matrix3d& getCov() const { return _cov; }
    const Eigen::Matrix3d& getInverseCov() const { return _icov; }
    const Eigen::Matrix<double, 6, 1>& getInvCovVec() const { return _icov_vec; }

    int _nr_points{0};
    Eigen::Vector3d _mean = Eigen::Vector3d::Zero();
    Eigen::Matrix3d _cov = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d _icov = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d _evecs = Eigen::Matrix3d::Zero();
    Eigen::Vector3d _evals = Eigen::Vector3d::Zero();
    /**
     * @brief inverse covariance matrix. It contains 6 elements because covariance
     * is symetric. Notice NDT algorithm only needs inverse covariance instead of
     * covariance
     */
    Eigen::Matrix<double, 6, 1> _icov_vec = Eigen::Matrix<double, 6, 1>::Zero();
};

}  // namespace irls_ndt
