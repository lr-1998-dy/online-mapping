#include "irls_ndt/voxel_grid_covariance.h"

namespace irls_ndt {

bool VoxelGridCovariance::init(const VoxelGridCovariance::PointCloudPtr& cloud_in) {
    if (nullptr == cloud_in) {
        std::cout << "input is empty!";
        return false;
    }
    if (std::isnan(_ndt_grid_resolution)) {
        std::cout << "ndt_grid_resolution is nan";
        return false;
    }
    _leaf_size << _ndt_grid_resolution, _ndt_grid_resolution, _ndt_grid_resolution, 1.0;
    Eigen::Vector4f min_p, max_p;
    pcl::getMinMax3D<VoxelGridCovariance::PointT>(*cloud_in, min_p, max_p);
    // Check that the leaf size is not too small, given the size of the data
    int dx = FloorDiv(max_p[0] - min_p[0], _leaf_size[0]) + 1;
    int dy = FloorDiv(max_p[1] - min_p[1], _leaf_size[1]) + 1;
    int dz = FloorDiv(max_p[2] - min_p[2], _leaf_size[2]) + 1;
    if ((dx * dy * dz) > std::numeric_limits<int>::max()) {
        std::cout << "Leaf size is too small for the input! indices would overflow!";
        return false;
    }
    // Compute the minimum and maximum bounding box values
    _min_b[0] = FloorDiv(min_p[0], _leaf_size[0]);
    _max_b[0] = FloorDiv(max_p[0], _leaf_size[0]);
    _min_b[1] = FloorDiv(min_p[1], _leaf_size[1]);
    _max_b[1] = FloorDiv(max_p[1], _leaf_size[1]);
    _min_b[2] = FloorDiv(min_p[2], _leaf_size[2]);
    _max_b[2] = FloorDiv(max_p[2], _leaf_size[2]);
    // Compute the number of divisions needed along all axis
    _div_b = _max_b - _min_b + Eigen::Vector4i::Ones();
    _div_b[3] = 0;
    _divb_mul = Eigen::Vector4i(1, _div_b[0], _div_b[0] * _div_b[1], 0);
    return true;
}

bool VoxelGridCovariance::run(const VoxelGridCovariance::PointCloudPtr& cloud_in) {
    if (!init(cloud_in)) {
        std::cout << "cannot init voxel_grid_covariance!";
        return false;
    }
    _leaves.clear();
    // First pass: go over all points and insert them into the right leaf
    for (const auto& pt : cloud_in->points) {
        if (std::isinf(pt.x) || std::isinf(pt.y) || std::isinf(pt.z)) {
            continue;
        }
        int ijk0 = static_cast<int>(floor(pt.x / _leaf_size[0]) - static_cast<float>(_min_b[0]));
        int ijk1 = static_cast<int>(floor(pt.y / _leaf_size[1]) - static_cast<float>(_min_b[1]));
        int ijk2 = static_cast<int>(floor(pt.z / _leaf_size[2]) - static_cast<float>(_min_b[2]));
        // insert into leaves
        int idx = ijk0 * _divb_mul[0] + ijk1 * _divb_mul[1] + ijk2 * _divb_mul[2];
        auto& leaf = _leaves[idx];
        Eigen::Vector3d pt3d(pt.x, pt.y, pt.z);
        leaf._mean += pt3d;
        leaf._cov += pt3d * pt3d.transpose();
        ++leaf._nr_points;
    }
    // delete invalid cells
    auto itr = _leaves.begin();
    while (itr != _leaves.end()) {
        if (itr->second._nr_points < _min_points_per_voxel) {
            itr = _leaves.erase(itr);
        } else {
            ++itr;
        }
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver;
    Eigen::Matrix3d eigen_val;
    Eigen::Vector3d pt_sum;
    // Second pass: go over all leaves and compute mean and cov
    for (auto& elem : _leaves) {
        auto& leaf = elem.second;
        pt_sum = leaf._mean;
        leaf._mean /= leaf._nr_points;
        // Single pass covariance calculation
        leaf._cov = (leaf._cov - 2 * (pt_sum * leaf._mean.transpose())) / leaf._nr_points +
                    leaf._mean * leaf._mean.transpose();
        leaf._cov *= (leaf._nr_points - 1.0) / leaf._nr_points;
        // Normalize Eigen Val such that max no more than 100x min.
        eigensolver.compute(leaf._cov);
        eigen_val = eigensolver.eigenvalues().asDiagonal();
        leaf._evecs = eigensolver.eigenvectors();
        if (eigen_val(0, 0) < 0 || eigen_val(1, 1) < 0 || eigen_val(2, 2) <= 0) {
            leaf._nr_points = -1;
            continue;
        }
        // Avoids matrices near singularities (eq 6.11)[Magnusson 2009]. Eigen
        // values less than a threshold of max eigen value are inflated to a set
        // fraction of the max eigen value.
        double min_covar_eigvalue = _min_covar_eigvalue_mult * eigen_val(2, 2);
        if (eigen_val(0, 0) < min_covar_eigvalue) {
            eigen_val(0, 0) = min_covar_eigvalue;

            if (eigen_val(1, 1) < min_covar_eigvalue) {
                eigen_val(1, 1) = min_covar_eigvalue;
            }
            leaf._cov = leaf._evecs * eigen_val * leaf._evecs.inverse();
        }
        leaf._evals = eigen_val.diagonal();
        auto m = leaf._cov.inverse();
        leaf._icov = m;
        leaf._icov_vec << m(0, 0), m(0, 1), m(0, 2), m(1, 1), m(1, 2), m(2, 2);
        if (leaf._icov.maxCoeff() == std::numeric_limits<float>::infinity() ||
            leaf._icov.minCoeff() == -std::numeric_limits<float>::infinity()) {
            leaf._nr_points = -1;
        }
    }
    return true;
}

}  // namespace irls_ndt
