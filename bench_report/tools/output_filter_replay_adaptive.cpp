// output_filter_replay_adaptive (chapter 36): replay rko_lio's output position filter from a run's logged inputs
// (output_filter_log, with the registration's Gauss-Newton Hessian on each update) with a measurement noise that
// follows each scan's registration. The prediction is OutputPositionFilter's (same state, same matrices); the update
// takes a full 3x3 measurement covariance R instead of sigma^2 I:
//   fixed     R = sigma^2 I (OutputPositionFilter's update; reproduces it)
//   scalar    R = sigma^2 clip((s / s_ref)^alpha) I
//   shape     R = sigma^2 V clip((lambda / mean(lambda))^alpha) V^T
//   marginal  R = sigma^2 V clip((lambda / s_ref)^alpha) V^T
// where C = V diag(lambda) V^T is the scan's marginal translation covariance, the translation block of H^-1 (H the
// final Hessian of the registration, averaged over its residuals, [t_world; r_world]), s = trace(C) / 3, s_ref the
// geometric mean of s over this and all earlier registered scans (causal), and clip limits each factor to [lo, hi].
// A scan without a Hessian (not registered by ICP) gets sigma^2 I and does not enter s_ref.
// Prints one filtered position per output pose ('U' and 'R' events), in order.
// Build: g++ -O2 -std=c++17 -I/usr/include/eigen3 output_filter_replay_adaptive.cpp
// Usage: output_filter_replay_adaptive <events> <mode> <sigma> <acceleration_noise> <bias_noise> <alpha> <lo> <hi>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/LU>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

using Matrix9d = Eigen::Matrix<double, 9, 9>;
using Vector9d = Eigen::Matrix<double, 9, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

struct Filter {
  double sigma = 0.02, q = 0.001, qb = 1e-4;
  Vector9d x = Vector9d::Zero();
  Matrix9d P = Matrix9d::Zero();
  bool initialized = false;
  double time = 0.0;
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();

  void reset(double t, const Eigen::Vector3d& p, const Eigen::Vector3d& v) {
    x.setZero();
    x.segment<3>(0) = p;
    x.segment<3>(3) = v;
    P.setZero();
    P.block<3, 3>(0, 0).diagonal().setConstant(sigma * sigma);
    P.block<3, 3>(3, 3).diagonal().setConstant(0.5 * 0.5);
    P.block<3, 3>(6, 6).diagonal().setConstant(0.05 * 0.05);
    time = t;
    initialized = true;
    acceleration.setZero();
  }
  void advance(double t) {
    if (!initialized) return;
    const double dt = t - time;
    if (!(dt > 0.0)) return;
    const Eigen::Vector3d a = acceleration - x.segment<3>(6);
    x.segment<3>(0) += x.segment<3>(3) * dt + 0.5 * a * dt * dt;
    x.segment<3>(3) += a * dt;
    Matrix9d F = Matrix9d::Identity();
    F.block<3, 3>(0, 3).diagonal().setConstant(dt);
    F.block<3, 3>(0, 6).diagonal().setConstant(-0.5 * dt * dt);
    F.block<3, 3>(3, 6).diagonal().setConstant(-dt);
    Matrix9d Q = Matrix9d::Zero();
    Q.block<3, 3>(0, 0).diagonal().setConstant(q * dt * dt * dt / 3.0);
    Q.block<3, 3>(0, 3).diagonal().setConstant(q * dt * dt / 2.0);
    Q.block<3, 3>(3, 0).diagonal().setConstant(q * dt * dt / 2.0);
    Q.block<3, 3>(3, 3).diagonal().setConstant(q * dt);
    Q.block<3, 3>(6, 6).diagonal().setConstant(qb * dt);
    P = F * P * F.transpose() + Q;
    time = t;
  }
  void propagate(double t, const Eigen::Vector3d& a) {
    acceleration = a;
    advance(t);
  }
  Eigen::Vector3d update(double t, const Eigen::Vector3d& p, const Eigen::Matrix3d& R) {
    advance(t);
    const Eigen::Matrix3d S = P.block<3, 3>(0, 0) + R;
    const Eigen::Matrix<double, 9, 3> K = P.block<9, 3>(0, 0) * S.inverse();
    x += K * (p - x.segment<3>(0));
    P -= K * P.block<3, 9>(0, 0);
    P = 0.5 * (P + P.transpose()).eval();
    return x.segment<3>(0);
  }
};

int main(int argc, char** argv) {
  if (argc != 9) {
    std::cerr << "usage: output_filter_replay_adaptive <events> <mode> <sigma> <q_a> <q_b> <alpha> <lo> <hi>\n";
    return 2;
  }
  const std::string mode = argv[2];
  Filter filter;
  filter.sigma = std::atof(argv[3]);
  filter.q = std::atof(argv[4]);
  filter.qb = std::atof(argv[5]);
  const double alpha = std::atof(argv[6]), lo = std::atof(argv[7]), hi = std::atof(argv[8]);
  if (mode != "fixed" && mode != "scalar" && mode != "shape" && mode != "marginal") {
    std::cerr << "unknown mode " << mode << "\n";
    return 2;
  }
  const double s2 = filter.sigma * filter.sigma;
  const auto clip = [&](double f) { return std::clamp(std::pow(f, alpha), lo, hi); };
  double log_sum = 0.0;
  long count = 0;
  std::ifstream file(argv[1]);
  std::string line;
  while (std::getline(file, line)) {
    std::istringstream in(line);
    char kind;
    double t;
    Eigen::Vector3d a, b;
    in >> kind >> t >> a.x() >> a.y() >> a.z();
    Eigen::Vector3d out;
    if (kind == 'P') {
      if (filter.initialized) filter.propagate(t, a);
      continue;
    }
    if (kind == 'R') {
      in >> b.x() >> b.y() >> b.z();
      filter.reset(t, a, b);
      out = a;
    } else {
      Matrix6d H = Matrix6d::Zero();
      for (int i = 0; i < 6 && in; ++i) {
        for (int j = 0; j < 6 && in; ++j) in >> H(i, j);
      }
      if (!in) H.setZero();  // a log without the Hessian
      Eigen::Matrix3d R = s2 * Eigen::Matrix3d::Identity();
      if (mode != "fixed" && H.norm() > 0.0) {
        const Eigen::FullPivLU<Matrix6d> lu(H);
        if (lu.isInvertible()) {
          Eigen::Matrix3d C = lu.inverse().block<3, 3>(0, 0);
          C = 0.5 * (C + C.transpose()).eval();
          const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(C);
          const Eigen::Vector3d lambda = eig.eigenvalues().cwiseMax(1e-30);
          const double s = lambda.sum() / 3.0;
          log_sum += std::log(s);
          ++count;
          const double s_ref = std::exp(log_sum / static_cast<double>(count));
          Eigen::Vector3d f;
          for (int k = 0; k < 3; ++k) {
            f(k) = mode == "scalar" ? clip(s / s_ref) : mode == "shape" ? clip(lambda(k) / s) : clip(lambda(k) / s_ref);
          }
          R = s2 * eig.eigenvectors() * f.asDiagonal() * eig.eigenvectors().transpose();
        }
      }
      out = filter.update(t, a, R);
    }
    std::printf("%.17g %.17g %.17g %.17g\n", t, out.x(), out.y(), out.z());
  }
  return 0;
}
