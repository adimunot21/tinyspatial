/// \file differentiable_dynamics.cpp
/// \brief The differentiable-first capability in ~30 lines: lift a model to the
/// autodiff scalar `Jet`, seed the configuration, and read exact derivatives of
/// forward kinematics AND inverse dynamics straight out of the same algorithms
/// — no finite differences, no external autodiff library.
///
/// Run it: `./build/debug/examples/differentiable_dynamics`. Referenced by
/// course chapter 13 (differentiable dynamics).
#include <iostream>

#include "tinyspatial/algo/forward_kinematics.hpp"
#include "tinyspatial/algo/rnea.hpp"
#include "tinyspatial/core/jet.hpp"
#include "tinyspatial/model/model.hpp"

int main() {
  using namespace tinyspatial;

  // A planar 2-link arm, built by hand (no URDF needed). Each link: 1 kg, COM
  // 0.5 m out along its x-axis, modest rotational inertia.
  Model model;
  const SpatialInertia link(1.0, Vector3(0.5, 0.0, 0.0), Matrix3::Identity() * 0.1);
  model.add_joint("j0", "l0", -1, JointRevolute{Vector3::UnitZ()}, SE3::identity(), link);
  model.add_joint("j1", "l1", 0, JointRevolute{Vector3::UnitZ()},
                  SE3(SO3::identity(), Vector3(1.0, 0.0, 0.0)), link);

  constexpr int kNv = 2;  // = model.nv()
  const Eigen::Vector2d q_value(0.3, -0.7);

  // Lift the (constant) model to the autodiff scalar and seed q as the
  // independent variables: slot k of the derivative tracks ∂/∂q_k.
  using J = Jet<kNv>;
  const ModelT<J> ad_model = model_cast<J>(model);
  DataT<J> data(ad_model);

  typename Types<J>::VectorX q(kNv), v(kNv), a(kNv), tau(kNv);
  for (int k = 0; k < kNv; ++k) {
    q(k) = J(q_value(k), k);  // seeded: ∂q_k/∂q_k = 1
    v(k) = J(0.0);
    a(k) = J(0.0);
  }

  // Forward kinematics — the end-effector position carries its own Jacobian.
  forward_kinematics(ad_model, data, q);
  const auto ee = data.pose_in_world.back().translation();
  std::cout << "end-effector position: (" << ee.x().a << ", " << ee.y().a << ")\n";
  std::cout << "  d(ee_x)/dq = [" << ee.x().v[0] << ", " << ee.x().v[1] << "]\n";
  std::cout << "  d(ee_y)/dq = [" << ee.y().v[0] << ", " << ee.y().v[1] << "]\n\n";

  // Inverse dynamics at v = a = 0 gives the gravity torque g(q); its autodiff
  // partials are ∂g/∂q (the configuration-dependent gravity stiffness). Gravity
  // points along −y so it acts in the arm's plane of motion (the z-axis joints).
  const typename Types<J>::Vector3 gravity(J(0.0), J(-9.81), J(0.0));
  rnea(ad_model, data, q, v, a, tau, gravity);
  std::cout << "gravity torque g(q) = [" << tau(0).a << ", " << tau(1).a << "]\n";
  std::cout << "dg/dq =\n";
  for (int r = 0; r < kNv; ++r) {
    std::cout << "  [" << tau(r).v[0] << ", " << tau(r).v[1] << "]\n";
  }
  return 0;
}
