# tinyspatial — top-level build.
#
# The library itself is header-mostly (CLAUDE.md §7), so the public target is an
# INTERFACE library. Non-header impl (bindings, URDF parsing) gets added as it
# lands in later phases. Keep this file lean; per-component logic lives in the
# subdirectory CMakeLists.
cmake_minimum_required(VERSION 3.20)

project(tinyspatial
  VERSION 0.1.0
  DESCRIPTION "Header-mostly C++20 spatial-algebra and differentiable-IK library"
  HOMEPAGE_URL "https://github.com/adimunot21/tinyspatial"
  LANGUAGES CXX)

# -- Are we the top-level project, or vendored into someone else's build? --
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(TINYSPATIAL_STANDALONE ON)
else()
  set(TINYSPATIAL_STANDALONE OFF)
endif()

# -- Options --
option(TINYSPATIAL_BUILD_TESTS "Build unit tests" ${TINYSPATIAL_STANDALONE})
option(TINYSPATIAL_BUILD_EXAMPLES "Build the standalone C++ examples" ${TINYSPATIAL_STANDALONE})
option(TINYSPATIAL_BUILD_BENCHMARKS "Build Google Benchmark perf tests" OFF)
option(TINYSPATIAL_BUILD_VALIDATION "Build the Pinocchio parity validation suite" OFF)
option(TINYSPATIAL_BUILD_PYTHON "Build the nanobind Python bindings" OFF)

# -- Language standard (CLAUDE.md §7: C++20, no compiler extensions) --
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# compile_commands.json for clang-tidy and editor tooling.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# -- The public, header-only interface library --
add_library(tinyspatial INTERFACE)
add_library(tinyspatial::tinyspatial ALIAS tinyspatial)
target_compile_features(tinyspatial INTERFACE cxx_std_20)
target_include_directories(tinyspatial INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Eigen is vendored under third_party/ (pinned to 3.4.0). version.hpp does not
# need it yet, so the Phase 0 build works even before submodules are fetched;
# we wire it in as soon as the submodule is present. SYSTEM include keeps
# Eigen's own headers out of our -Werror surface.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen/Eigen/Core")
  target_include_directories(tinyspatial SYSTEM INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen>)
  message(STATUS "tinyspatial: using vendored Eigen at third_party/eigen")
else()
  message(STATUS "tinyspatial: third_party/eigen not present yet "
                 "(run: git submodule update --init --recursive)")
endif()

# Shared warning flags for OUR translation units (tests, examples, benchmarks).
# Never applied to third_party. CLAUDE.md §7: -Wall -Wextra -Wpedantic, with
# -Werror reserved for the canonical CI build to avoid breaking local dev on
# newer compilers.
add_library(tinyspatial_warnings INTERFACE)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(tinyspatial_warnings INTERFACE
    -Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------------------------
# Vendored tinyxml2 — single .cpp, treated as a SYSTEM include so its warnings
# don't pollute our -Wall surface.
# ---------------------------------------------------------------------------
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/tinyxml2/tinyxml2.cpp")
  add_library(tinyxml2_vendor STATIC third_party/tinyxml2/tinyxml2.cpp)
  target_include_directories(tinyxml2_vendor SYSTEM PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/tinyxml2>)
  set_target_properties(tinyxml2_vendor PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

# ---------------------------------------------------------------------------
# tinyspatial_urdf — non-template URDF parsing (heavy enough to deserve a .cpp
# per CLAUDE.md §7). Linked in by anything that needs to read URDF; the
# header-only `tinyspatial` target stays untouched.
# ---------------------------------------------------------------------------
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/tinyxml2/tinyxml2.cpp")
  add_library(tinyspatial_urdf STATIC src/urdf/urdf_loader.cpp)
  target_link_libraries(tinyspatial_urdf
    PUBLIC tinyspatial
    PRIVATE tinyxml2_vendor tinyspatial_warnings)
  set_target_properties(tinyspatial_urdf PROPERTIES POSITION_INDEPENDENT_CODE ON)
  add_library(tinyspatial::urdf ALIAS tinyspatial_urdf)
endif()

if(TINYSPATIAL_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

if(TINYSPATIAL_BUILD_EXAMPLES)
  add_subdirectory(src/examples)
endif()

# ---------------------------------------------------------------------------
# Validation-only Python binding (Phase 4 Pinocchio cross-check). Activated by
# TINYSPATIAL_BUILD_VALIDATION=ON. Builds a single nanobind extension module
# `_tinyspatial` and places it at `python/tinyspatial/_tinyspatial*.so` so the
# validation tests can `import tinyspatial`. Full Python bindings (the public
# package, examples) are Phase 8 work and live elsewhere.
# ---------------------------------------------------------------------------
if(TINYSPATIAL_BUILD_VALIDATION)
  find_package(Python 3.10 COMPONENTS Interpreter Development.Module REQUIRED)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/nanobind/CMakeLists.txt")
    # nanobind picks up the Python interpreter from the find_package call above.
    add_subdirectory(third_party/nanobind EXCLUDE_FROM_ALL)
    nanobind_add_module(_tinyspatial STABLE_ABI src/bindings/main.cpp)
    target_link_libraries(_tinyspatial PRIVATE tinyspatial tinyspatial_urdf)
    set_target_properties(_tinyspatial PROPERTIES
      LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/python/tinyspatial")

    # CTest entry for the Pinocchio cross-check. Prefers the project venv at
    # .venv (where `pin` is installed) over the system Python; falls back to
    # ${Python_EXECUTABLE}. Labelled so `ctest --preset=validation -L
    # pinocchio_parity` matches CLAUDE.md §9.
    if(TINYSPATIAL_BUILD_TESTS)
      set(_validation_python "${Python_EXECUTABLE}")
      if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/.venv/bin/python")
        set(_validation_python "${CMAKE_CURRENT_SOURCE_DIR}/.venv/bin/python")
      endif()
      add_test(NAME pinocchio_parity_kinematics
               COMMAND ${_validation_python}
                       "${CMAKE_CURRENT_SOURCE_DIR}/tests/validation/test_kinematics.py"
               WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
      set_tests_properties(pinocchio_parity_kinematics PROPERTIES
        LABELS "pinocchio_parity"
        ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/python")
    endif()
  endif()
endif()
