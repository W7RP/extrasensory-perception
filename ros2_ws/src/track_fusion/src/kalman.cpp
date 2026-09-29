#include "track_fusion/kalman.hpp"

#include <cmath>

#include <Eigen/Cholesky>

namespace track_fusion
{

void cv_model(double dt, double q, Mat6 & F, Mat6 & Q) noexcept
{
  F.setIdentity();
  F.block<3, 3>(0, 3) = dt * Mat3::Identity();
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  Q.setZero();
  Q.block<3, 3>(0, 0) = (q * dt3 / 3.0) * Mat3::Identity();
  Q.block<3, 3>(0, 3) = (q * dt2 / 2.0) * Mat3::Identity();
  Q.block<3, 3>(3, 0) = (q * dt2 / 2.0) * Mat3::Identity();
  Q.block<3, 3>(3, 3) = (q * dt) * Mat3::Identity();
}

void predict(Vec6 & x, Mat6 & P, double dt, double q) noexcept
{
  if (dt <= 0.0) {
    return;
  }
  Mat6 F;
  Mat6 Q;
  cv_model(dt, q, F, Q);
  x = F * x;
  P = F * P * F.transpose() + Q;
}

Innovation innovation(const Vec6 & x, const Mat6 & P, const Vec3 & z, const Mat3 & R) noexcept
{
  Innovation in;
  in.nu = z - x.head<3>();
  in.S = P.block<3, 3>(0, 0) + R;
  const Eigen::LLT<Mat3> llt(in.S);
  in.d2 = in.nu.dot(llt.solve(in.nu));
  const Mat3 L = llt.matrixL();
  in.log_det_S = 2.0 * (std::log(L(0, 0)) + std::log(L(1, 1)) + std::log(L(2, 2)));
  return in;
}

double update(Vec6 & x, Mat6 & P, const Vec3 & z, const Mat3 & R) noexcept
{
  const Innovation in = innovation(x, P, z, R);
  // K = P H' S^-1, with H = [I 0].
  const Eigen::Matrix<double, 6, 3> PHt = P.block<6, 3>(0, 0);
  const Eigen::Matrix<double, 6, 3> K = in.S.llt().solve(PHt.transpose()).transpose();
  x += K * in.nu;
  Eigen::Matrix<double, 6, 6> I_KH = Mat6::Identity();
  I_KH.block<6, 3>(0, 0) -= K;
  P = I_KH * P * I_KH.transpose() + K * R * K.transpose();
  P = 0.5 * (P + P.transpose());
  return in.d2;
}

}  // namespace track_fusion
