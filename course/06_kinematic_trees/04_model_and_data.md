# Model and Data

`tinyspatial` splits robot information into two objects:

- **`Model`** holds everything *constant* about a robot — the topology, the
  joint placements, the link inertias, the names. It is built once (usually
  from a URDF) and read from thereafter.
- **`Data`** holds everything *per-configuration* — the link poses, the
  velocities, the accelerations, the wrenches. It is the scratchpad an
  algorithm fills in given a `(q, v, a)`.

This mirrors Pinocchio exactly and is one of the most important shapes in
the library.

## Why split them?

Two reasons.

### 1. Const-correctness across threads

The Model is `const` once built. Two threads can call forward kinematics on
the same Model in parallel, as long as each has its own Data. Sharing the
Data is the bug; sharing the Model is the design.

### 2. Allocation budget

Algorithms run a lot — RNEA might be evaluated thousands of times a second.
Allocating the storage every call would dominate the runtime. Handing the
caller a Data to reuse moves the allocation to construction time, and the
hot path becomes pure arithmetic on pre-sized buffers.

```cpp
Model m = build_model_from_urdf_file("robot.urdf");
Data d(m);
for (...) {
  forward_kinematics(m, d, q);   // no heap allocation here
  // ... use d.pose_in_world, d.v, etc. ...
}
```

## What Data stores

```cpp
class Data {
 public:
  explicit Data(const Model& model);     // sized to model.njoints()

  std::vector<SE3>    pose_in_parent;     // joint i's frame in parent[i]'s frame
  std::vector<SE3>    pose_in_world;      // joint i's frame in the world
  std::vector<Motion> v;                  // body i's spatial velocity
  std::vector<Motion> a;                  // body i's spatial acceleration
  std::vector<Force>  f;                  // wrench on body i
};
```

These are the *one* set of buffers used by every kinematic and dynamic
algorithm. RNEA writes into `v`, `a`, and `f`. Forward kinematics writes into
`pose_in_parent` and `pose_in_world`. CRBA uses `pose_in_parent` and writes
elsewhere. Algorithms are pure functions over `(Model, q, Data&)`.

> For readers familiar with Pinocchio: these are exactly its `liMi`, `oMi`,
> `v`, `a`, `f`. The names are docstring-equivalent.

## What goes where, the short list

| Question | Where to look |
| -------- | ------------- |
| "How many DOF does this robot have?" | `Model::nv()` |
| "What's joint 3's parent?" | `Model::parent[3]` |
| "Where is link 5 in the world right now?" | `Data::pose_in_world[5]` (after FK) |
| "What's body 2's inertia?" | `Model::inertia[2]` |
| "What torque do the motors need?" | filled into `Data::f` by RNEA (chapter 10) |

## How later chapters build on this

The Model/Data split sets the structure for all the chapters that follow.
Forward kinematics (chapter 08) is a one-line loop that fills
`Data::pose_in_world` from `Model::placement` and `Model::joints`. RNEA
(chapter 10) is a two-line loop (forward then backward sweep) filling
`Data::v`, `a`, and `f`. Every algorithm becomes a short, readable recipe
because the data layout already did the hard work.

## Where this lives in the library

| Concept | File · symbol |
| ------- | ------------- |
| Model | [`model.hpp`](../../include/tinyspatial/model/model.hpp) · `class Model` |
| Data | [`model.hpp`](../../include/tinyspatial/model/model.hpp) · `class Data` |
| `Data` is sized to a `Model` | `Data::Data(const Model&)` |
| Sizing test | [`test_model.cpp`](../../tests/unit/model/test_model.cpp) · `Model.DataIsSizedToMatch` |

Next: the [exercises](exercises.md), then [chapter 07 — URDF](../07_urdf_robot_models/README.md).
