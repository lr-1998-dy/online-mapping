/*
 * @Descripttion: 
 * @version: 
 * @Author: Yuhang Li
 * @Date: 2022-09-17 20:00:34
 * @LastEditors: Yuhang Li
 * @LastEditTime: 2022-09-17 21:02:00
 */
#pragma once

#include "voxel_grid_covariance.h"

namespace irls_ndt {

/**
 * @class NeighborFinder
 * @brief NDT voxel map
 */
class NeighborFinder {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef pcl::PointXYZI PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef std::unique_ptr<NeighborFinder> Ptr;
    /**
     * @brief Users can only get instance from this function
     * @return instance pointer
     */
    static Ptr create(double ndt_resolution) {
        return std::unique_ptr<NeighborFinder>(new NeighborFinder(ndt_resolution));
    }
    /**
     * @brief destructors
     */
    virtual ~NeighborFinder() = default;
    bool init(const PointCloudPtr& cloud);
    /**
     * @brief get neighbors of voxel cells
     * @param reference_point the point to querry
     * @param neighbor_num number of neighbors, can be 1 or 7. 1 can be fast,
     * while 7 is accurate with high latency
     * @param neighbors the output voxel cells
     */
    void getNeighborhood(const PointT& reference_point,
                         int neighbor_num,
                         std::vector<NDTLeaf::ConstPtr>& neighbors) const;
    /**
     * @brief NDT resolution is useful when computing outliers and inliers
     * @return NDT resolution
     */
    double getResolution() const { return _leaf_size(0); }
    /**
     * @brief NDT voxel size
     * @return NAN if NDT lidar map is not loaded, voxel size otherwise
     */
    double getLeafSize() const { return _leaves.empty() ? NAN : _leaves.size(); }

  private:
    NeighborFinder() = delete;
    explicit NeighborFinder(double ndt_resolution) : _ndt_resolution(ndt_resolution){};

    double _ndt_resolution;
    Eigen::Vector4i _min_b, _max_b, _divb_mul;
    Eigen::Vector4f _leaf_size;
    NDTLidarMap _leaves;
    /**
     * @brief backbone function to load ndt_lidar_map. This function can be called
     * multiple times without considering the file IO burden, because file IO is
     * only called once at the first time
     * @param file protobuf file of NDT lidar map
     * @return true if load is success
     */
    bool load(const std::string& file);
    /**
     * @brief nearest neighbor search
     * @param relative_coordinates relative coordinates support 26/8/1 neighbors
     * @param reference_point the point to query
     * @param neighbors resulting neigbors
     * @return neighbor size
     */
    int getNeighborhoodAtPoint(const Eigen::MatrixXi& relative_coordinates,
                               const PointT& reference_point,
                               std::vector<NDTLeaf::ConstPtr>& neighbors) const;
    /**
     * @brief the Euclidean distance between a 3D point and NDT lidar map. Useful
     * when computing inliers and outliers
     * @param pt the point to query
     * @return NAN if the querry point has no neighbors in NDT lidar map,
     * Euclidean distance otherwise
     */
    double getDistance(const PointT& pt) const;
};

}  // namespace irls_ndt

