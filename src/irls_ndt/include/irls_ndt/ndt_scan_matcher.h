#pragma once

#include <pcl/common/transforms.h>

#include "neighbor_finder.h"
#include "rigid_estimator.h"

template <typename T>
using EigenVector = std::vector<T, Eigen::aligned_allocator<T>>;

namespace irls_ndt {

struct NDTLocParam {
    int max_iterations = 5;
    int min_neighbor_num = 200;
    int max_irls = 10;
    int search_neighbor_num = 1;  // 7 or 1
    double ndt_resolution = 1.0;
    double transformation_epsilon = 0.01;
    double outlier_ratio = 0.55;
    double min_score = 0.50;
};

/**
 * @class NDTScanMatcher
 * @brief Algorithms for Normal Distribution Transform (NDT). Compared with original NDT, the
 * Gaussian method is replaced with iteratively reweighted least squares (IRLS), in order to avoid
 * the computation of Hessian matrix
 */
class NDTScanMatcher {
  public:
    typedef pcl::PointXYZI PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;
    typedef std::unique_ptr<NDTScanMatcher> Ptr;
    /**
     * @brief Using Eigen::Matrix4d introduces memory aligned problems: Even if macro
     * EIGEN_MAKE_ALIGNED_OPERATOR_NEW is added, it can still cause segfault when using
     * std::make_shared<T> https://eigen.tuxfamily.org/bz/show_bug.cgi?id=1049. Therefore, just not
     * use fixed size vectorizable Eigen object to make life easier
     */
    typedef Eigen::Matrix<double, 4, 4, Eigen::DontAlign> SafeMatrix4d;
    /**
     * @brief Users can only get instance from this function
     * @param param NDT localization param
     * @return instance pointer
     */
    static Ptr create(const NDTLocParam& param) {
        return std::unique_ptr<NDTScanMatcher>(new NDTScanMatcher(param));
    }
    /**
     * @brief destructor
     */
    virtual ~NDTScanMatcher() {}
    /**
     * @brief set NDT lidar map
     * @param target_cloud
     * @return true if target cloud is successfully loaded
     */
    bool setInputTarget(const PointCloudPtr& target_cloud);
    /**
     * @brief backbone function for NDT matching
     * @param cloud_source source cloud to be aligned with NDT lidar map
     * @param output aligned point cloud
     * @param guess init transformation from GPS or IMU dead reckon. It is important because NDT is
     * a local method and can be easily to be trapped into local minima
     * @return true if NDT matching is success
     */
    bool align(const PointCloudConstPtr& cloud_source,
               PointCloudPtr& output,
               const Eigen::Matrix4d& guess = Eigen::Matrix4d::Identity());
    /**
     * @brief for robust lidar localization initialization
     * @param cloud_source source cloud to be aligned with NDT lidar map
     * @param guess init transformation from xyz sampling generated from robust init
     * @return NDT matching score
     */
    double computeLikelihood(const PointCloudConstPtr& cloud_source, const Eigen::Matrix4d& guess);
    /**
     * @brief getter of final aligned transformation computed by NDT
     */
    const SafeMatrix4d& getFinalTransformation() const { return _final_transformation; }
    /**
     * @brief getter of NDT matching score
     */
    double getScore() const { return _score; }
    /**
     * @brief getter of NDT neighbor number
     */
    int getNeighNum() const { return static_cast<int>(_v_V.size()); };

  private:
    /**
     * @brief constructor
     */
    NDTScanMatcher() = delete;
    /**
     * @brief constructor
     */
    explicit NDTScanMatcher(const NDTLocParam& param) : _param(param) {}
    /**
     * @brief compute the guassian fitting parameters (eq. 6.8) [Magnusson 2009]
     */
    void computeCoeff(float resolution);
    /**
     * @brief backbone function for iteratively reweighted least-squares (IRLS) to compute the rigid
     * transformation
     * @param trans_cloud cloud to be transformed
     * @param M rigid transformation to be optimized. It will be overwrote during optimization
     * @return true if success
     */
    bool optimization(PointCloud& trans_cloud, Eigen::Matrix4d& M);
    /**
     * @brief prepare various matrices for quadratic program
     */
    bool prepareItem(const PointCloud& trans_cloud);
    /**
     * @brief converting Eigen data structure
     */
    void assign2Mat();
    /**
     * @brief compute rotation and translation by quadratic program
     */
    SafeMatrix4d computeRT();

    NDTLocParam _param;
    // The normalization constants used fit the point distribution to a normal distribution,
    // Equation 6.8 [Magnusson 2009].
    double _gauss_d2{NAN};
    double _score{0.0};
    SafeMatrix4d _final_transformation;
    // the following will be used in IRLS.
    std::vector<double> _v_W;
    EigenVector<Eigen::Matrix<double, 6, 1>> _v_M;  // inverse of covariance matrix.
    EigenVector<Eigen::Vector3d> _v_A;              // A means after.
    EigenVector<Eigen::Vector3d> _v_V;              // V means residual vector.
    Eigen::VectorXd _W_vec;
    Eigen::Matrix3Xd _V_mat;
    Eigen::Matrix3Xd _A_mat;
    Eigen::MatrixXd _M_mat;
    NeighborFinder::Ptr _neighbor_finder;
};

}  // namespace irls_ndt
