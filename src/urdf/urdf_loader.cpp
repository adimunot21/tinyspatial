/// \file urdf_loader.cpp
/// \brief URDF → `Model` adapter. See urdf_loader.hpp for the contract.
///
/// The parser is a single forward pass over the XML, then a BFS from the
/// detected root link(s) to assemble joints in topological order. We
/// deliberately keep the surface area small: anything the kinematic / dynamic
/// algorithms can't use (`<visual>`, `<collision>`, `<transmission>`,
/// `<gazebo>`, `<safety_controller>`, mimic / planar / screw joints, xacro
/// substitution) is either silently ignored or surfaced as a `UrdfParseError`.
#include "tinyspatial/urdf/urdf_loader.hpp"

#include <cstdlib>
#include <fstream>
#include <queue>
#include <sstream>
#include <string>
#include <tinyxml2.h>
#include <unordered_map>
#include <utility>
#include <vector>

#include "tinyspatial/liegroup/se3.hpp"
#include "tinyspatial/liegroup/so3.hpp"
#include "tinyspatial/model/joint.hpp"
#include "tinyspatial/spatial/inertia.hpp"

#include <Eigen/Geometry>

namespace tinyspatial {

namespace {

using tinyxml2::XML_SUCCESS;
using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;

Vector3 parse_vec3(const char* str) {
  if (str == nullptr) {
    throw UrdfParseError("missing vector3 attribute");
  }
  std::istringstream iss(str);
  Vector3 v;
  if (!(iss >> v.x() >> v.y() >> v.z())) {
    throw UrdfParseError(std::string("could not parse vector3 from: ") + str);
  }
  return v;
}

Vector3 attr_xyz(const XMLElement* el, const Vector3& fallback = Vector3::Zero()) {
  const char* attr = (el == nullptr) ? nullptr : el->Attribute("xyz");
  if (attr == nullptr) {
    return fallback;
  }
  return parse_vec3(attr);
}

Vector3 attr_rpy(const XMLElement* el) {
  const char* attr = (el == nullptr) ? nullptr : el->Attribute("rpy");
  if (attr == nullptr) {
    return Vector3::Zero();
  }
  return parse_vec3(attr);
}

/// URDF rpy ordering: R = Rz(yaw) · Ry(pitch) · Rx(roll), extrinsic.
SO3 rpy_to_so3(const Vector3& rpy) {
  const SO3 rx(Matrix3(Eigen::AngleAxis<Scalar>(rpy.x(), Vector3::UnitX()).toRotationMatrix()));
  const SO3 ry(Matrix3(Eigen::AngleAxis<Scalar>(rpy.y(), Vector3::UnitY()).toRotationMatrix()));
  const SO3 rz(Matrix3(Eigen::AngleAxis<Scalar>(rpy.z(), Vector3::UnitZ()).toRotationMatrix()));
  return rz * ry * rx;
}

SE3 parse_origin(const XMLElement* origin) {
  if (origin == nullptr) {
    return SE3::identity();
  }
  return SE3(rpy_to_so3(attr_rpy(origin)), attr_xyz(origin));
}

Scalar parse_double_attr(const XMLElement* el, const char* attr, Scalar fallback) {
  if (el == nullptr) {
    return fallback;
  }
  const char* str = el->Attribute(attr);
  if (str == nullptr) {
    return fallback;
  }
  char* end = nullptr;
  const double value = std::strtod(str, &end);
  if (end == str) {
    throw UrdfParseError(std::string("could not parse double attribute '") + attr + "': " + str);
  }
  return static_cast<Scalar>(value);
}

SpatialInertia parse_inertial(const XMLElement* link) {
  const XMLElement* in = link->FirstChildElement("inertial");
  if (in == nullptr) {
    return SpatialInertia::zero();
  }
  const Scalar mass = parse_double_attr(in->FirstChildElement("mass"), "value", Scalar(0));
  const SE3 com_frame = parse_origin(in->FirstChildElement("origin"));
  const Vector3& com = com_frame.translation();

  const XMLElement* in_el = in->FirstChildElement("inertia");
  Matrix3 i_in_inertial = Matrix3::Zero();
  if (in_el != nullptr) {
    const Scalar ixx = parse_double_attr(in_el, "ixx", Scalar(0));
    const Scalar iyy = parse_double_attr(in_el, "iyy", Scalar(0));
    const Scalar izz = parse_double_attr(in_el, "izz", Scalar(0));
    const Scalar ixy = parse_double_attr(in_el, "ixy", Scalar(0));
    const Scalar ixz = parse_double_attr(in_el, "ixz", Scalar(0));
    const Scalar iyz = parse_double_attr(in_el, "iyz", Scalar(0));
    i_in_inertial << ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz;
  }
  // The inertial origin's rotation maps the inertia tensor from its own frame
  // into the link frame: I_link = R · I_inertial · Rᵀ.
  const Matrix3 r = com_frame.rotation().matrix();
  const Matrix3 inertia_com = r * i_in_inertial * r.transpose();
  return SpatialInertia(mass, com, inertia_com);
}

}  // namespace

Model build_model_from_urdf(std::string_view xml) {
  XMLDocument doc;
  const auto err = doc.Parse(xml.data(), xml.size());
  if (err != XML_SUCCESS) {
    throw UrdfParseError(std::string("XML parse error: ") + doc.ErrorStr());
  }

  const XMLElement* root = doc.FirstChildElement("robot");
  if (root == nullptr) {
    throw UrdfParseError("URDF must have a <robot> root element");
  }

  Model model;
  if (const char* name = root->Attribute("name"); name != nullptr) {
    model.name = name;
  }

  // ---- Pass 1: collect link inertias by name. ----
  std::unordered_map<std::string, SpatialInertia> links;
  for (const XMLElement* el = root->FirstChildElement("link"); el != nullptr;
       el = el->NextSiblingElement("link")) {
    const char* link_name = el->Attribute("name");
    if (link_name == nullptr) {
      throw UrdfParseError("<link> missing required 'name' attribute");
    }
    if (!links.emplace(link_name, parse_inertial(el)).second) {
      throw UrdfParseError(std::string("duplicate <link> name: ") + link_name);
    }
  }

  // ---- Pass 2: collect joints. ----
  struct JointInfo {
    std::string name;
    std::string parent_link;
    std::string child_link;
    Joint joint;
    SE3 placement;
  };
  std::vector<JointInfo> all_joints;
  for (const XMLElement* el = root->FirstChildElement("joint"); el != nullptr;
       el = el->NextSiblingElement("joint")) {
    const char* name = el->Attribute("name");
    if (name == nullptr) {
      throw UrdfParseError("<joint> missing required 'name' attribute");
    }
    const char* type = el->Attribute("type");
    if (type == nullptr) {
      throw UrdfParseError(std::string("<joint name=\"") + name + "\"> missing required 'type'");
    }
    const XMLElement* parent_el = el->FirstChildElement("parent");
    const XMLElement* child_el = el->FirstChildElement("child");
    if (parent_el == nullptr || child_el == nullptr) {
      throw UrdfParseError(std::string("<joint name=\"") + name +
                           "\"> missing <parent> or <child>");
    }
    const char* parent_link = parent_el->Attribute("link");
    const char* child_link = child_el->Attribute("link");
    if (parent_link == nullptr || child_link == nullptr) {
      throw UrdfParseError(std::string("<joint name=\"") + name +
                           "\"> parent/child missing 'link'");
    }

    const Vector3 axis = attr_xyz(el->FirstChildElement("axis"), Vector3::UnitX());

    JointInfo info;
    info.name = name;
    info.parent_link = parent_link;
    info.child_link = child_link;
    info.placement = parse_origin(el->FirstChildElement("origin"));

    const std::string t(type);
    if (t == "fixed") {
      info.joint = JointFixed{};
    } else if (t == "revolute" || t == "continuous") {
      info.joint = JointRevolute{axis.normalized()};
    } else if (t == "prismatic") {
      info.joint = JointPrismatic{axis.normalized()};
    } else if (t == "floating") {
      info.joint = JointFloating{};
    } else {
      throw UrdfParseError(std::string("<joint name=\"") + name + "\"> unsupported type: " + type);
    }
    all_joints.push_back(std::move(info));
  }

  // ---- Pass 3: topological assembly. ----
  std::unordered_map<std::string, std::vector<int>> joints_by_parent;
  for (int i = 0; i < static_cast<int>(all_joints.size()); ++i) {
    joints_by_parent[all_joints[i].parent_link].push_back(i);
  }
  std::unordered_map<std::string, bool> is_child;
  for (const auto& j : all_joints) {
    is_child[j.child_link] = true;
  }
  // Root links: link names that never appear as a joint's child.
  std::vector<std::string> roots;
  for (const auto& [link_name, _inertia] : links) {
    if (!is_child[link_name]) {
      roots.push_back(link_name);
    }
  }
  if (roots.empty()) {
    throw UrdfParseError("URDF has no root link (every link is a child)");
  }

  // BFS from the roots, adding joints in topological order. parent_idx in the
  // Model is the index of the joint whose child_link is *this* link's parent
  // — or -1 if the parent is itself a root link in URDF.
  std::unordered_map<std::string, int> link_to_joint_idx;
  for (const auto& r : roots) {
    link_to_joint_idx[r] = -1;
  }
  std::queue<std::string> queue;
  for (const auto& r : roots) {
    queue.push(r);
  }
  int processed = 0;
  while (!queue.empty()) {
    const std::string parent_link_name = std::move(queue.front());
    queue.pop();
    const int parent_joint = link_to_joint_idx.at(parent_link_name);
    const auto it = joints_by_parent.find(parent_link_name);
    if (it == joints_by_parent.end()) {
      continue;
    }
    for (int joint_index : it->second) {
      const JointInfo& info = all_joints[joint_index];
      const auto link_it = links.find(info.child_link);
      if (link_it == links.end()) {
        throw UrdfParseError(std::string("joint '") + info.name +
                             "' references unknown child link '" + info.child_link + "'");
      }
      const int idx = model.add_joint(info.name, info.child_link, parent_joint, info.joint,
                                      info.placement, link_it->second);
      link_to_joint_idx[info.child_link] = idx;
      queue.push(info.child_link);
      ++processed;
    }
  }

  if (processed != static_cast<int>(all_joints.size())) {
    throw UrdfParseError("URDF has a cycle or disconnected joint(s): only " +
                         std::to_string(processed) + " of " + std::to_string(all_joints.size()) +
                         " joints reachable from roots");
  }
  return model;
}

Model build_model_from_urdf_file(const std::string& path) {
  std::ifstream stream(path);
  if (!stream) {
    throw UrdfParseError("could not open URDF file: " + path);
  }
  std::ostringstream buf;
  buf << stream.rdbuf();
  return build_model_from_urdf(buf.str());
}

}  // namespace tinyspatial
