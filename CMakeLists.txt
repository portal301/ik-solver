cmake_minimum_required(VERSION 3.10)
project(ikfast_robotics VERSION 1.0.0 LANGUAGES CXX)

# C++ Standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build options
option(BUILD_TESTS "Build test programs" ON)

# Find dependencies
find_package(PkgConfig REQUIRED)

# LAPACK and BLAS (required for IKFast)
find_package(LAPACK REQUIRED)
find_package(BLAS REQUIRED)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${LAPACK_INCLUDE_DIRS}
    ${BLAS_INCLUDE_DIRS}
)

# ============================================================================
# Library: ikfast_robotics
# ============================================================================

set(IKFAST_ROBOTICS_SOURCES
    src/ikfast_solver.cpp
    src/robots/kj125_ikfast.cpp
)

# Add more robot-specific IKFast cpp files here as they are generated
# Example:
# list(APPEND IKFAST_ROBOTICS_SOURCES src/robots/gp4_ikfast.cpp)
# list(APPEND IKFAST_ROBOTICS_SOURCES src/robots/gp7_ikfast.cpp)

# Build shared library
add_library(ikfast_robotics SHARED ${IKFAST_ROBOTICS_SOURCES})

target_include_directories(ikfast_robotics PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(ikfast_robotics
    ${LAPACK_LIBRARIES}
    ${BLAS_LIBRARIES}
)

# Compiler options
if(MSVC)
    target_compile_options(ikfast_robotics PRIVATE /W4)
else()
    target_compile_options(ikfast_robotics PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ============================================================================
# Tests
# ============================================================================

if(BUILD_TESTS)
    enable_testing()

    add_executable(test_solver tests/test_solver.cpp)
    target_link_libraries(test_solver ikfast_robotics)
    add_test(NAME test_solver COMMAND test_solver)
endif()

# ============================================================================
# Installation
# ============================================================================

install(TARGETS ikfast_robotics
    EXPORT ikfast_robotics-targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
)

install(EXPORT ikfast_robotics-targets
    FILE ikfast_robotics-config.cmake
    NAMESPACE ikfast_robotics::
    DESTINATION lib/cmake/ikfast_robotics
)
