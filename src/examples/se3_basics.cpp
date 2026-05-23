/// \file se3_basics.cpp
/// \brief A tiny worked example: compose two SE(3) transforms and print the
/// result. Run it after building: `./build/debug/examples/se3_basics`.
///
/// This is referenced by course chapter 04 (Lie groups) as the "hello world"
/// of rigid transforms.
#include <cmath>
#include <iostream>

#include "tinyspatial/liegroup/se3.hpp"

int main() {
  using tinyspatial::SE3;
  using tinyspatial::SO3;
  using tinyspatial::Vector3;
  using tinyspatial::Vector6;

  const Eigen::IOFormat fmt(4, 0, ", ", "\n", "    [", "]");

  // T_a: rotate +90° about the world z-axis, then sit at (1, 0, 0).
  const SE3 t_a(SO3::exp(Vector3(0.0, 0.0, M_PI / 2.0)), Vector3(1.0, 0.0, 0.0));
  // T_b: rotate +90° about its own x-axis, offset (0, 2, 0).
  const SE3 t_b(SO3::exp(Vector3(M_PI / 2.0, 0.0, 0.0)), Vector3(0.0, 2.0, 0.0));

  const SE3 t_ab = t_a * t_b;  // first apply T_b, then T_a

  std::cout << "T_a translation:\n" << t_a.translation().transpose().format(fmt) << "\n\n";
  std::cout << "T_b translation:\n" << t_b.translation().transpose().format(fmt) << "\n\n";
  std::cout << "Composed  T_a * T_b\n";
  std::cout << "  rotation:\n" << t_ab.rotation().matrix().format(fmt) << "\n";
  std::cout << "  translation:\n" << t_ab.translation().transpose().format(fmt) << "\n\n";

  // Where does the point at T_b's origin land in the world frame?
  const Vector3 origin_of_b_in_world = t_a.act(t_b.translation());
  std::cout << "Origin of B, expressed in world (== T_a * T_b applied to 0):\n"
            << origin_of_b_in_world.transpose().format(fmt) << "\n\n";

  // exp/log are inverses: recover the twist that generates the composed motion.
  const Vector6 xi = t_ab.log();
  const SE3 recovered = SE3::exp(xi);
  const double round_trip_error = (recovered.matrix() - t_ab.matrix()).cwiseAbs().maxCoeff();
  std::cout << "log(T_a*T_b) twist (omega; v):\n" << xi.transpose().format(fmt) << "\n";
  std::cout << "exp(log(.)) round-trip error: " << round_trip_error << "\n";

  return 0;
}
