// Constant-velocity Kalman filter for one track, position-only measurements.
//
// State x = [p v] (world frame), 6x6 covariance. Process model: white
// acceleration noise with spectral density q [m^2/s^3] on each axis (the
// standard discretised "nearly constant velocity" model, Bar-Shalom et al.,
// Estimation with Applications to Tracking and Navigation, 6.3.2).
//
// All functions are noexcept, fixed-size, O(1) and allocation-free.
#pragma once

#include "track_fusion/types.hpp"

namespace track_fusion
{

// Transition matrix F and process noise Q for a step of dt seconds.
void cv_model(double dt, double q, Mat6 & F, Mat6 & Q) noexcept;

// Propagate (x, P) forward by dt >= 0. With no measurements this is what makes
// the covariance grow while a track coasts.
void predict(Vec6 & x, Mat6 & P, double dt, double q) noexcept;

struct Innovation
{
  Vec3 nu{Vec3::Zero()};      // z - H x
  Mat3 S{Mat3::Identity()};   // H P H' + R
  double d2{0.0};             // nu' S^-1 nu: squared Mahalanobis distance
  double log_det_S{0.0};
};

[[nodiscard]] Innovation innovation(
  const Vec6 & x, const Mat6 & P, const Vec3 & z, const Mat3 & R) noexcept;

// Kalman update with a position measurement (Joseph form, so P stays
// symmetric positive definite). Returns the normalised innovation squared.
double update(Vec6 & x, Mat6 & P, const Vec3 & z, const Mat3 & R) noexcept;

}  // namespace track_fusion
