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
