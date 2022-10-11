/*
 * @Descripttion: 
 * @version: 
 * @Author: Yuhang Li
 * @Date: 2022-09-17 20:00:34
 * @LastEditors: Yuhang Li
 * @LastEditTime: 2022-09-17 21:01:38
 */
#pragma once

#include <pcl/common/common.h>

#include "ndt_leaf.h"

namespace irls_ndt {

inline int FloorDiv(double a, double b) {
    return static_cast<int>(std::floor(a / b));
}

typedef std::unordered_map<size_t, NDTLeaf> NDTLidarMap;

class VoxelGridCovariance {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef std::unique_ptr<VoxelGridCovariance> Ptr;

    typedef pcl::PointXYZI PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;

    /**
     * @brief Users can only get instance from this function
     * @return instance pointer
     */
    static Ptr create(double ndt_resolution) {
        return std::unique_ptr<VoxelGridCovariance>(new VoxelGridCovariance(ndt_resolution));
    }
    virtual ~VoxelGridCovariance() = default;

    // getters.
    int getMinPointPerVoxel() const { return _min_points_per_voxel; }
    const Eigen::Vector4f& getLeafSize() const { return _leaf_size; }
    const Eigen::Vector4i& getMinb() const { return _min_b; }
    const Eigen::Vector4i& getMaxb() const { return _max_b; }
    const Eigen::Vector4i& getDivbMul() const { return _divb_mul; }
    const NDTLidarMap& getLeaves() const { return _leaves; }
    // main process
    bool run(const PointCloudPtr& cloud_in);

  private:
    VoxelGridCovariance() = delete;
    explicit VoxelGridCovariance(double ndt_grid_resolution)
        : _ndt_grid_resolution(ndt_grid_resolution) {}
    bool init(const PointCloudPtr& cloud_in);

    double _ndt_grid_resolution{NAN};
    const int _min_points_per_voxel{6};
    const double _min_covar_eigvalue_mult{0.01};
    Eigen::Vector4f _leaf_size = Eigen::Vector4f::Zero();
    Eigen::Vector4i _min_b = Eigen::Vector4i::Zero();
    Eigen::Vector4i _max_b = Eigen::Vector4i::Zero();
    Eigen::Vector4i _divb_mul = Eigen::Vector4i::Zero();
    Eigen::Vector4i _div_b = Eigen::Vector4i::Zero();
    NDTLidarMap _leaves;
};

}  // namespace irls_ndt

