# Standalone C++ examples for the README and the course. Each is a tiny
# self-contained program linking only the header-only library.

function(tinyspatial_add_example name)
  add_executable(${name} ${ARGN})
  target_link_libraries(${name} PRIVATE tinyspatial tinyspatial_warnings)
  # Keep example binaries together under build/<preset>/examples/.
  set_target_properties(${name} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/examples")
endfunction()

tinyspatial_add_example(se3_basics se3_basics.cpp)
tinyspatial_add_example(differentiable_dynamics differentiable_dynamics.cpp)
