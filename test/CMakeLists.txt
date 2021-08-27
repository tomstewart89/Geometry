cmake_minimum_required(VERSION 3.14)
project(geometry_tests)

set(CMAKE_CXX_STANDARD 11)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
)

FetchContent_MakeAvailable(googletest)

enable_testing()

include_directories("${PROJECT_SOURCE_DIR}" "${PROJECT_SOURCE_DIR}/.."
                    "${PROJECT_SOURCE_DIR}/BasicLinearAlgebra")

add_executable(test_geometry ../Geometry.cpp test_geometry.cpp)
target_link_libraries(test_geometry gtest_main)

add_executable(test_rotations ../OtherRotations.cpp test_rotations.cpp)
target_link_libraries(test_rotations gtest_main)

add_executable(
  test_examples ../OtherRotations.cpp ../Geometry.cpp
                ../examples/PoseGraph/PoseGraph.cpp test_examples.cpp)
target_link_libraries(test_examples gtest_main)
target_include_directories(test_examples
                           PUBLIC "${PROJECT_SOURCE_DIR}/../examples/PoseGraph")

include(GoogleTest)

gtest_discover_tests(test_geometry)
gtest_discover_tests(test_rotations)
gtest_discover_tests(test_examples)
