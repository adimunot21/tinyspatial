# Google Benchmark harness, vendored under third_party/. Built only when
# TINYSPATIAL_BUILD_BENCHMARKS=ON. Benchmarks live alongside the algorithms
# they exercise; keyed by algorithm name (`bench_<algo>.cpp`).

set(TINYSPATIAL_BENCHMARK_DIR "${CMAKE_SOURCE_DIR}/third_party/benchmark")
if(NOT EXISTS "${TINYSPATIAL_BENCHMARK_DIR}/CMakeLists.txt")
  message(FATAL_ERROR
    "third_party/benchmark is missing. Run:\n"
    "  git submodule update --init --recursive")
endif()

# Suppress Google Benchmark's own tests / install rules to keep the build lean.
set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
set(BENCHMARK_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
set(BENCHMARK_ENABLE_GTEST_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory("${TINYSPATIAL_BENCHMARK_DIR}"
                 "${CMAKE_BINARY_DIR}/_deps/benchmark" EXCLUDE_FROM_ALL)

function(tinyspatial_add_benchmark name)
  add_executable(${name} ${ARGN})
  target_link_libraries(${name}
    PRIVATE tinyspatial tinyspatial_urdf tinyspatial_warnings benchmark::benchmark)
  target_compile_definitions(${name} PRIVATE
    "TINYSPATIAL_FIXTURE_DIR=\"${CMAKE_SOURCE_DIR}/data/robots\"")
endfunction()

if(TARGET tinyspatial_urdf)
  tinyspatial_add_benchmark(bench_rnea bench_rnea.cpp)
  tinyspatial_add_benchmark(bench_kinematics bench_kinematics.cpp)
  tinyspatial_add_benchmark(bench_ik bench_ik.cpp)
  tinyspatial_add_benchmark(bench_rnea_derivatives bench_rnea_derivatives.cpp)
endif()
