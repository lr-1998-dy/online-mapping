#include "irls_ndt/rigid_estimator.h"

namespace irls_ndt {

//// odinary algorithms.
void computeHessianOrdinary(const Eigen::Matrix3Xd& V,
                            const Eigen::Matrix3Xd& Aft,
                            const Eigen::VectorXd& W,
                            const Eigen::MatrixXd& MArray,
                            Eigen::MatrixXd& H,
                            Eigen::VectorXd& b) {
    H.resize(6, 6);
    H.setZero();
    b.resize(6);
    b.setZero();
    for (int i = 0; i < V.cols(); i++) {
        Eigen::Vector3d p = Aft.col(i);
        Eigen::MatrixXd tmpH = Eigen::MatrixXd::Zero(3, 6);
        tmpH.topLeftCorner(3, 3) = -skew(p);
        tmpH.topRightCorner(3, 3) = Eigen::Matrix3d::Identity();
        Eigen::VectorXd m = MArray.col(i);
        Eigen::Matrix3d omega;
        omega << m(0), m(1), m(2), m(1), m(3), m(4), m(2), m(4), m(5);
        H = H + W(i) * tmpH.transpose() * omega * tmpH;
        b = b + W(i) * tmpH.transpose() * omega * V.col(i);
    }
}

// vectorized algorithm. The complex equations are generated by symvar tool of MATLAB.
void computeHessianVectorized(const Eigen::Matrix3Xd& V,
                              const Eigen::Matrix3Xd& Aft,
                              const Eigen::VectorXd& W,
                              const Eigen::MatrixXd& MArray,
                              Eigen::MatrixXd& H,
                              Eigen::VectorXd& b) {
    H.resize(6, 6);
    H.setZero();
    b.resize(6);
    b.setZero();

    Eigen::ArrayXd p1(V.cols()), p2(V.cols()), p3(V.cols()), v1(V.cols()), v2(V.cols()),
            v3(V.cols()), m1(V.cols()), m2(V.cols()), m3(V.cols()), m4(V.cols()), m5(V.cols()),
            m6(V.cols()), w(V.cols());
    p1 = Aft.row(0).transpose().array(), p2 = Aft.row(1).transpose().array(),
    p3 = Aft.row(2).transpose().array(), v1 = V.row(0).transpose().array(),
    v2 = V.row(1).transpose().array(), v3 = V.row(2).transpose().array(),
    m1 = MArray.row(0).transpose().array(), m2 = MArray.row(1).transpose().array(),
    m3 = MArray.row(2).transpose().array(), m4 = MArray.row(3).transpose().array(),
    m5 = MArray.row(4).transpose().array(), m6 = MArray.row(5).transpose().array();
    w = W.array();
    //// Hrr.
    Eigen::ArrayXXd arrayHrr(V.cols(), 9);
    arrayHrr.col(0) = w * (m4 * (p3 * p3) + m6 * (p2 * p2) - m5 * p2 * p3 * 2.0);
    arrayHrr.col(1) = -p3 * (m2 * p3 * w - m3 * p2 * w) + p1 * (m5 * p3 * w - m6 * p2 * w);
    arrayHrr.col(2) = p2 * (m2 * p3 * w - m3 * p2 * w) - p1 * (m4 * p3 * w - m5 * p2 * w);
    arrayHrr.col(3) = -p3 * (m2 * p3 * w - m5 * p1 * w) + p2 * (m3 * p3 * w - m6 * p1 * w);
    arrayHrr.col(4) = w * (m1 * (p3 * p3) + m6 * (p1 * p1) - m3 * p1 * p3 * 2.0);
    arrayHrr.col(5) = -p2 * (m1 * p3 * w - m3 * p1 * w) + p1 * (m2 * p3 * w - m5 * p1 * w);
    arrayHrr.col(6) = p3 * (m2 * p2 * w - m4 * p1 * w) - p2 * (m3 * p2 * w - m5 * p1 * w);
    arrayHrr.col(7) = -p3 * (m1 * p2 * w - m2 * p1 * w) + p1 * (m3 * p2 * w - m5 * p1 * w);
    arrayHrr.col(8) = w * (m1 * (p2 * p2) + m4 * (p1 * p1) - m2 * p1 * p2 * 2.0);
    Eigen::Matrix3d Hrr = Eigen::Matrix3d::Identity();
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            Hrr(i, j) = arrayHrr.col(3 * i + j).sum();
        }
    }
    //// Hrt.
    Eigen::ArrayXXd arrayHrt(V.cols(), 9);
    arrayHrt.col(0) = -w * (m2 * p3 - m3 * p2);
    arrayHrt.col(1) = -w * (m4 * p3 - m5 * p2);
    arrayHrt.col(2) = -w * (m5 * p3 - m6 * p2);
    arrayHrt.col(3) = w * (m1 * p3 - m3 * p1);
    arrayHrt.col(4) = w * (m2 * p3 - m5 * p1);
    arrayHrt.col(5) = w * (m3 * p3 - m6 * p1);
    arrayHrt.col(6) = -w * (m1 * p2 - m2 * p1);
    arrayHrt.col(7) = -w * (m2 * p2 - m4 * p1);
    arrayHrt.col(8) = -w * (m3 * p2 - m5 * p1);
    Eigen::Matrix3d Hrt = Eigen::Matrix3d::Identity();
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            Hrt(i, j) = (arrayHrt.col(3 * i + j)).sum();
        }
    }
    //// Htt.
    Eigen::Matrix3d Htt = Eigen::Matrix3d::Identity();
    Htt << (w * m1).sum(), (w * m2).sum(), (w * m3).sum(), 0.0, (w * m4).sum(), (w * m5).sum(), 0.0,
            0.0, (w * m6).sum();
    for (int i = 0; i < 3; i++) {
        for (int j = i + 1; j < 3; j++) {
            Htt(j, i) = Htt(i, j);
        }
    }
    //// reduce to H.
    H.block(0, 0, 3, 3) = Hrr;
    H.block(0, 3, 3, 3) = Hrt;
    H.block(3, 0, 3, 3) = Hrt.transpose();
    H.block(3, 3, 3, 3) = Htt;
    //// br
    Eigen::ArrayXXd arrayBr(V.cols(), 3);
    arrayBr.col(0) = -v1 * (m2 * p3 * w - m3 * p2 * w) - v2 * (m4 * p3 * w - m5 * p2 * w) -
                     v3 * (m5 * p3 * w - m6 * p2 * w);
    arrayBr.col(1) = v1 * (m1 * p3 * w - m3 * p1 * w) + v2 * (m2 * p3 * w - m5 * p1 * w) +
                     v3 * (m3 * p3 * w - m6 * p1 * w);
    arrayBr.col(2) = -v1 * (m1 * p2 * w - m2 * p1 * w) - v2 * (m2 * p2 * w - m4 * p1 * w) -
                     v3 * (m3 * p2 * w - m5 * p1 * w);
    Eigen::Vector3d br = Eigen::Vector3d::Zero();
    for (int i = 0; i < 3; i++) {
        br(i) = arrayBr.col(i).sum();
    }

    Eigen::ArrayXXd arrayBt(V.cols(), 3);
    arrayBt.col(0) = w * (m1 * v1 + m2 * v2 + m3 * v3);
    arrayBt.col(1) = w * (m2 * v1 + m4 * v2 + m5 * v3);
    arrayBt.col(2) = w * (m3 * v1 + m5 * v2 + m6 * v3);
    Eigen::Vector3d bt = Eigen::Vector3d::Zero();
    for (int i = 0; i < 3; i++) {
        bt(i) = arrayBt.col(i).sum();
    }
    //// reduce to b.
    b << br, bt;
}

}  // namespace irls_ndt