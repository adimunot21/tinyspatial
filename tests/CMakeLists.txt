# Unit tests. Mirrors the include/ layout under unit/. Validation (Pinocchio
# parity) tests are Python and live in tests/validation/, gated separately.

# GoogleTest is vendored under third_party/. Building it as part of our tree
# (rather than find_package) keeps the "compiles standalone" promise.
set(TINYSPATIAL_GTEST_DIR "${CMAKE_SOURCE_DIR}/third_party/googletest")
if(NOT EXISTS "${TINYSPATIAL_GTEST_DIR}/CMakeLists.txt")
  message(FATAL_ERROR
    "third_party/googletest is missing. Run:\n"
    "  git submodule update --init --recursive")
endif()

# Don't install gtest, and force shared CRT off for consistency.
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
add_subdirectory("${TINYSPATIAL_GTEST_DIR}"
                 "${CMAKE_BINARY_DIR}/_deps/googletest" EXCLUDE_FROM_ALL)

include(GoogleTest)

# Helper: register a unit test executable linked against the library + gtest.
function(tinyspatial_add_test name)
  add_executable(${name} ${ARGN})
  target_link_libraries(${name}
    PRIVATE tinyspatial tinyspatial_warnings GTest::gtest_main)
  gtest_discover_tests(${name})
endfunction()

tinyspatial_add_test(test_version unit/test_version.cpp)
tinyspatial_add_test(test_so3 unit/liegroup/test_so3.cpp)
tinyspatial_add_test(test_se3 unit/liegroup/test_se3.cpp)
tinyspatial_add_test(test_cross unit/spatial/test_cross.cpp)
tinyspatial_add_test(test_motion_force unit/spatial/test_motion_force.cpp)
tinyspatial_add_test(test_inertia unit/spatial/test_inertia.cpp)
tinyspatial_add_test(test_joint unit/model/test_joint.cpp)
tinyspatial_add_test(test_model unit/model/test_model.cpp)

# URDF tests need the (non-template) loader implementation + fixture path.
if(TARGET tinyspatial_urdf)
  tinyspatial_add_test(test_urdf_loader unit/urdf/test_urdf_loader.cpp)
  tinyspatial_add_test(test_urdf_placements unit/urdf/test_urdf_placements.cpp)
  tinyspatial_add_test(test_urdf_fuzz unit/urdf/test_urdf_fuzz.cpp)
  tinyspatial_add_test(test_forward_kinematics unit/algo/test_forward_kinematics.cpp)
  tinyspatial_add_test(test_jacobian unit/algo/test_jacobian.cpp)
  tinyspatial_add_test(test_rnea unit/algo/test_rnea.cpp)
  tinyspatial_add_test(test_crba unit/algo/test_crba.cpp)
  tinyspatial_add_test(test_aba unit/algo/test_aba.cpp)
  tinyspatial_add_test(test_fk_derivatives unit/diff/test_fk_derivatives.cpp)
  tinyspatial_add_test(test_rnea_derivatives unit/diff/test_rnea_derivatives.cpp)
  tinyspatial_add_test(test_dls unit/ik/test_dls.cpp)
  foreach(t test_urdf_loader test_urdf_placements test_urdf_fuzz test_forward_kinematics test_jacobian test_rnea test_crba test_aba test_fk_derivatives test_rnea_derivatives test_dls)
    target_link_libraries(${t} PRIVATE tinyspatial_urdf)
    target_compile_definitions(${t} PRIVATE
      "TINYSPATIAL_TEST_FIXTURE_DIR=\"${CMAKE_SOURCE_DIR}/data/robots\"")
  endforeach()
endif()
