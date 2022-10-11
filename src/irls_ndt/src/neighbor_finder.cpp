/*
 * @Descripttion: 
 * @version: 
 * @Author: sueRimn
 * @Date: 2022-09-17 20:00:34
 * @LastEditors: sueRimn
 * @LastEditTime: 2022-09-18 15:38:28
 */
#include "irls_ndt/neighbor_finder.h"

namespace irls_ndt {

bool NeighborFinder::init(const NeighborFinder::PointCloudPtr& cloud) {
    auto voxel_grid = VoxelGridCovariance::create(_ndt_resolution);
    if (!voxel_grid->run(cloud)) {
        std::cout << "cannot compute voxel grid covariance!";
        return false;
    }
    _leaves = voxel_grid->getLeaves();
    _min_b = voxel_grid->getMinb();
    _max_b = voxel_grid->getMaxb();
    _divb_mul = voxel_grid->getDivbMul();
    _leaf_size = voxel_grid->getLeafSize();
    return true;
}

double NeighborFinder::getDistance(const PointT& pt) const {
    std::vector<NDTLeaf::ConstPtr> neighborhood;
    getNeighborhood(pt, 1, neighborhood);
    if (neighborhood.empty()) {
        return NAN;
    }
    Eigen::Vector3d x_trans{pt.x, pt.y, pt.z};
    return (x_trans - neighborhood.back()->getMean()).norm();
}

int NeighborFinder::getNeighborhoodAtPoint(const Eigen::MatrixXi& relative_coordinates,
                                           const PointT& reference_point,
                                           std::vector<NDTLeaf::ConstPtr>& neighbors) const {
    neighbors.clear();
    if (_leaves.empty()) {
        std::cout << "leaves are empty!";
    }
    // Find displacement coordinates
    Eigen::Vector4i ijk(FloorDiv(reference_point.x, _leaf_size[0]),
                        FloorDiv(reference_point.y, _leaf_size[1]),
                        FloorDiv(reference_point.z, _leaf_size[2]),
                        0);
    Eigen::Array4i diff2min = _min_b - ijk;
    Eigen::Array4i diff2max = _max_b - ijk;
    neighbors.reserve(relative_coordinates.cols());
    // Check each neighbor to see if it is occupied
    for (int ni = 0; ni < relative_coordinates.cols(); ni++) {
        Eigen::Vector4i displacement =
                (Eigen::Vector4i() << relative_coordinates.col(ni), 0).finished();
        // Checking if the specified cell is in the grid
        if ((diff2min <= displacement.array()).all() && (diff2max >= displacement.array()).all()) {
            auto leaf_iter = _leaves.find(((ijk + displacement - _min_b).dot(_divb_mul)));
            if (leaf_iter != _leaves.end()) {
                neighbors.push_back(&leaf_iter->second);
            }
        }
    }
    return (static_cast<int>(neighbors.size()));
}

void NeighborFinder::getNeighborhood(const PointT& reference_point,
                                     int neighbor_num,
                                     std::vector<NDTLeaf::ConstPtr>& neighbors) const {
    neighbors.clear();
    Eigen::MatrixXi relative_coordinates;
    if (1 == neighbor_num) {
        relative_coordinates = Eigen::MatrixXi::Zero(3, 1);
    } else if (7 == neighbor_num) {
        relative_coordinates = Eigen::MatrixXi::Zero(3, 7);
        relative_coordinates.setZero();
        relative_coordinates(0, 1) = 1;
        relative_coordinates(0, 2) = -1;
        relative_coordinates(1, 3) = 1;
        relative_coordinates(1, 4) = -1;
        relative_coordinates(2, 5) = 1;
        relative_coordinates(2, 6) = -1;
    } else {
        std::cout << "neighbor_num is erroneous! neighbor_num = " << neighbor_num;
    }
    getNeighborhoodAtPoint(relative_coordinates, reference_point, neighbors);
}

}  // namespace irls_ndt
