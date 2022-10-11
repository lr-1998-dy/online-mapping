#include "irls_ndt/ndt_scan_matcher.h"

namespace irls_ndt {

void NDTScanMatcher::computeCoeff(float resolution) {
    double gauss_c1 = 10.0 * (1 - _param.outlier_ratio);
    double gauss_c2 = _param.outlier_ratio / pow(resolution, 3);
    double gauss_d3 = -log(gauss_c2);
    double gauss_d1 = -log(gauss_c1 + gauss_c2) - gauss_d3;
    _gauss_d2 = -2 * log((-log(gauss_c1 * exp(-0.5) + gauss_c2) - gauss_d3) / gauss_d1);
}

bool NDTScanMatcher::setInputTarget(const NDTScanMatcher::PointCloudPtr& target_cloud) {
    _neighbor_finder = NeighborFinder::create(_param.ndt_resolution);
    if (nullptr == _neighbor_finder) {
        std::cout << "cannot create neighbor finder!";
        return false;
    }
    if (!_neighbor_finder->init(target_cloud)) {
        std::cout << "cannot init neighbor finder!";
        return false;
    }
    // std::isnan() will guarantee that computeCoeff() is only called once
    if (std::isnan(_gauss_d2)) {
        computeCoeff(_neighbor_finder->getResolution());
    }
    return true;
}

bool NDTScanMatcher::align(const PointCloudConstPtr& cloud_source,
                           PointCloudPtr& output,
                           const Eigen::Matrix4d& guess) {
    // avoid segment fault
    if (!output) {
        output.reset(new pcl::PointCloud<PointT>());
    }
    // Apply guessed transformation prior to search for neighbours
    pcl::transformPointCloud(*cloud_source, *output, guess.cast<float>());
    Eigen::Matrix4d dM = Eigen::Matrix4d::Identity();
    _final_transformation = guess;
    for (int nr_iterations = 0; nr_iterations < _param.max_iterations; nr_iterations++) {
        // refine output cloud and transformation simultaneously
        if (!optimization(*output, dM)) {
            std::cout << "optimization fails";
            return false;
        }
        _final_transformation = dM * _final_transformation;
        // terminate if translation is small enough
        if (dM.block<3, 1>(0, 3).norm() < _param.transformation_epsilon) {
            break;
        }
    }
    if (_score > _param.min_score) {
        // std::cout<<"fitness_score : "<<_score;
        return true;
    }
    std::cout << "ndt fails! score = " << _score << ", min_score = " << _param.min_score;
    return false;
}

double NDTScanMatcher::computeLikelihood(const PointCloudConstPtr& cloud_source,
                                         const Eigen::Matrix4d& guess) {
    PointCloud::Ptr cloud_after(new PointCloud());
    pcl::transformPointCloud(*cloud_source, *cloud_after, guess.cast<float>());
    if (!prepareItem(*cloud_after)) {
        return 0.0;
    }
    return _score;
}

bool NDTScanMatcher::prepareItem(const PointCloud& trans_cloud) {
    _v_W.clear();
    _v_A.clear();
    _v_M.clear();
    _v_V.clear();
    size_t reserve_len = trans_cloud.size();
    _v_W.reserve(reserve_len);
    _v_A.reserve(reserve_len);
    _v_M.reserve(reserve_len);
    _v_V.reserve(reserve_len);
    std::vector<double> v_distance_sqr;
    v_distance_sqr.reserve(reserve_len);
    Eigen::Matrix3d icov = Eigen::Matrix3d::Identity();
    std::vector<float> distances;
    std::vector<NDTLeaf::ConstPtr> neighbors;
    Eigen::Vector3d transform_point = Eigen::Vector3d::Zero();
    Eigen::Vector3d residual = Eigen::Vector3d::Zero();
    for (const auto& point : trans_cloud.points) {
        _neighbor_finder->getNeighborhood(point, _param.search_neighbor_num, neighbors);
        int count = 0;
        for (const auto& cell : neighbors) {
            transform_point << point.x, point.y, point.z;
            _v_A.push_back(transform_point);
            residual = transform_point - cell->getMean();
            _v_V.push_back(residual);
            // Use precomputed covariance for speed.
            auto m = cell->getInvCovVec();
            _v_M.push_back(m);
            icov << m(0), m(1), m(2), m(1), m(3), m(4), m(2), m(4), m(5);
            double val = -0.5 * _gauss_d2 * residual.transpose() * icov * residual;
            _v_W.push_back(val);
            // only record first neighbor
            if (!count) {
                v_distance_sqr.push_back(residual.squaredNorm());
            }
            count++;
        }
    }
    // Compute the score. When the distance is less than half resolution of NDT map, this point-pair
    // is considered to be reliable.
    _score = 0.0;
    double distance_thr = std::pow(_neighbor_finder->getResolution() / 2, 2.0);
    if (v_distance_sqr.size() >= static_cast<size_t>(_param.min_neighbor_num)) {
        int count = std::count_if(v_distance_sqr.begin(),
                                  v_distance_sqr.end(),
                                  [&distance_thr](double val) { return val < distance_thr; });
        _score = static_cast<double>(count) / v_distance_sqr.size();
        return true;
    }
    // neighbors are not reliable.
    std::cout << "neighbors are not reliable";
    return false;
}

void NDTScanMatcher::assign2Mat() {
    // assign
    size_t len = _v_W.size();
    _W_vec.resize(len, 1);
    _V_mat.resize(3, len);
    _A_mat.resize(3, len);
    _M_mat.resize(6, len);
    for (size_t i = 0; i < _v_W.size(); i++) {
        _W_vec(i) = _v_W.at(i);
        _V_mat.col(i) = _v_V.at(i);
        _A_mat.col(i) = _v_A.at(i);
        _M_mat.col(i) = _v_M.at(i);
    }
    // remember to exp() for the weight.
    double maxW = _W_vec.maxCoeff();
    _W_vec = (_W_vec.array() - maxW).exp();
}

NDTScanMatcher::SafeMatrix4d NDTScanMatcher::computeRT() {
    Eigen::MatrixXd H;
    Eigen::VectorXd b;
    computeHessianVectorized(_V_mat, _A_mat, _W_vec, _M_mat, H, b);
    // solve the equation: Hx = -b
    Eigen::LDLT<Eigen::MatrixXd> ldlt(H);
    Eigen::VectorXd x = ldlt.solve(-b);
    Eigen::Vector3d translation = x.tail<3>();
    // compute rotation
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d u = x.head(3);
    // AngleAxisd() is problematic when angle is small enough, and thus skip it directly
    double angle = u.norm();
    if (abs(angle) > 1e-12) {
        // compute rotation when u is in normal case.
        Eigen::Vector3d n = u / angle;
        rotation = Eigen::AngleAxisd(angle, n).matrix();
    }
    SafeMatrix4d dM = SafeMatrix4d::Identity();
    dM.block<3, 3>(0, 0) = rotation;
    dM.block<3, 1>(0, 3) = translation;
    return dM;
}

bool NDTScanMatcher::optimization(PointCloud& trans_cloud,
                                  Eigen::Matrix4d& optimal_transformation) {
    optimal_transformation.setIdentity();
    for (int iter_irls = 0; iter_irls < _param.max_irls; iter_irls++) {
        if (!prepareItem(trans_cloud)) {
            optimal_transformation.setIdentity();
            std::cout << "cannot prepareItem";
            return false;
        }
        assign2Mat();
        Eigen::Matrix4d dM = computeRT();
        // terminate when translation is small enough
        if (dM.block<3, 1>(0, 3).norm() < _param.transformation_epsilon) {
            break;
        }
        pcl::transformPointCloud(trans_cloud, trans_cloud, dM.cast<float>());
        optimal_transformation = dM * optimal_transformation;
    }
    return true;
}

}  // namespace irls_ndt

