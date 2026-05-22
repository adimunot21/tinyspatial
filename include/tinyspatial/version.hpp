/// \file version.hpp
/// \brief Compile-time version information for tinyspatial.
///
/// This is the Phase 0 placeholder header: it carries no algebra, only the
/// library version, and exists so the build, test, and CI plumbing have
/// something real to compile and assert against.
#ifndef TINYSPATIAL_VERSION_HPP
#define TINYSPATIAL_VERSION_HPP

#include <string_view>

namespace tinyspatial {

/// Major version component (incremented on incompatible API changes).
inline constexpr int kVersionMajor = 0;
/// Minor version component (incremented on backwards-compatible additions).
inline constexpr int kVersionMinor = 1;
/// Patch version component (incremented on backwards-compatible fixes).
inline constexpr int kVersionPatch = 0;

/// Semantic version string, e.g. "0.1.0". Kept in sync with the CMake
/// project() version and CHANGELOG.md.
inline constexpr std::string_view kVersion = "0.1.0";

}  // namespace tinyspatial

#endif  // TINYSPATIAL_VERSION_HPP
