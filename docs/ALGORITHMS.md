# tinyspatial — algorithm reference

Mathematical reference for the algorithms in `include/tinyspatial/`. Audience:
senior C++ / robotics reviewers who already know what RNEA is and want to check
*our* conventions and derivations against theirs.

For a from-scratch introduction, see [`/course`](../course).

---

## 1. Spatial-vector conventions

These conventions are baked into every algorithm. **They are not optional knobs
to flip per call** — flipping them would invalidate the validation oracle, and
they are checked at the type level by the `Motion` / `Force` distinction.

### 1.1 Ordering: **angular-first**

A spatial 6-vector is laid out as

```
[ ω_x  ω_y  ω_z   v_x  v_y  v_z ]
   └── angular ──┘   └── linear ──┘
```

Indices `0..2` are angular, `3..5` are linear. This follows Featherstone
(*Rigid Body Dynamics Algorithms*, 2008), and **diverges from Pinocchio**, which
orders Motion / Force linear-first. The validation suite applies a 6×6
permutation converter at the Pinocchio boundary; the library
itself never sees Pinocchio's order.

### 1.2 Reference frame: **body-fixed**

Every per-link velocity / acceleration / force / inertia is expressed in the
**body's own frame** (sometimes called "local" or "trivialised"). To compare
quantities living in different links' frames, transport them with a Plücker
transform.

### 1.3 The motion Plücker (SE(3) adjoint)

For `T = (R, t)`,

```
Ad_T  =  ⎡   R     0 ⎤      (acts on motions, i.e. twists)
         ⎣ [t]× R   R ⎦
```

The off-diagonal block `[t]×R` is what couples angular into linear under a
frame translation. See [`liegroup/se3.hpp`](../include/tinyspatial/liegroup/se3.hpp)
`SE3::adjoint()`, and the equivalent named [`plucker_motion()`](../include/tinyspatial/spatial/plucker.hpp).

### 1.4 The force (dual) Plücker

Wrenches transform with the inverse-transpose of the motion Plücker:

```
Ad_T^{-T}  =  ⎡  R     [t]× R ⎤      (acts on forces, i.e. wrenches)
              ⎣  0      R     ⎦
```

See [`force_plucker()`](../include/tinyspatial/spatial/plucker.hpp). The two
transports are *adjoint* (in the linear-algebra sense): the dot product
`f · m` between a wrench and a twist is invariant under simultaneous transport
of both. The test `MotionForceTest.DualityPower` checks this identity for
random configurations.

### 1.5 Lie bracket (spatial cross product)

The `ad` operator of `se(3)` — Featherstone's spatial cross — is

```
m ×  ≡  ⎡  [ω]×    0  ⎤      m = (ω; v)
        ⎣  [v]×  [ω]× ⎦
```

with `m ×* = -(m ×)^T` for forces. See
[`spatial/cross.hpp`](../include/tinyspatial/spatial/cross.hpp). The
homomorphism `d/dt Ad(exp(t m))|₀ = m ×` is asserted in
`CrossTest.CrossMotionIsAdjointDerivative`.

### 1.6 Spatial inertia

In the body frame, with `c = COM`, `Ī = inertia about COM`, mass `m`:

```
I_s  =  ⎡  Ī + m·([t]×^T [t]×)|c=COM         m·[c]×  ⎤
        ⎣          m·[c]× ^T                  m·I_3  ⎦
```

(Equivalently `Ī_O = Ī − m·[c]×^2`.) The off-diagonal block-pair has opposite
signs because `[c]× ^T = -[c]×`, which is what keeps the matrix symmetric.

We store the *separable* parameters (`mass`, `com`, `inertia_com`) rather than
the 6×6, so that an SE(3) transform moves each piece cleanly:

- mass invariant,
- COM as a point: `c' = R c + t`,
- inertia tensor: `Ī' = R Ī R^T`.

Equivalent to the congruence `I' = X^{-T} I X^{-1}` on the 6×6 form — see
[`inertia.hpp`](../include/tinyspatial/spatial/inertia.hpp) and the test
`InertiaTest.Se3TransformMatchesCongruence`.

---

## 2. Lie-group calculus (SO(3), SE(3))

Filled in by Phase 1; see [`/course/04_lie_groups`](../course/04_lie_groups) for
prose and the headers themselves for the closed forms. The numerical decisions
worth flagging here:

- `SO3::log` is computed from the quaternion as `2·asin‖q.vec‖/‖q.vec‖ · q.vec`,
  stable through θ → π.
- All Jacobian and exp/log coefficients have explicit Taylor fallbacks below
  `kSmallAngle = 1e-3` rad. Without these, the entire family returns `NaN` near
  the identity.
- The SE(3) group Jacobian uses Barfoot's Q matrix (State Estimation for
  Robotics, eq. 7.86), reordered for our angular-first convention.

---

## 3. Future sections

The remaining algorithm sections will be filled in phase by phase:

- §4 Forward kinematics — Phase 4
- §5 Geometric and analytical Jacobians — Phase 4
- §6 RNEA / ABA / CRBA — Phase 5
- §7 Analytical derivatives — Phase 6
- §8 Inverse kinematics — Phase 7

Until each lands, see `course/` for the tutorial introductions and the relevant
header for the contract.
