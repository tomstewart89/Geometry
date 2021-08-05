cmake_minimum_required(VERSION 3.14)
project(bla_tests)

set(CMAKE_CXX_STANDARD 11)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
)

FetchContent_MakeAvailable(googletest)

enable_testing()

include_directories("${PROJECT_SOURCE_DIR}" "${PROJECT_SOURCE_DIR}/..")

add_executable(test_geometry test_geometry.cpp)
target_link_libraries(test_geometry gtest_main)

include(GoogleTest)

gtest_discover_tests(test_geometry)