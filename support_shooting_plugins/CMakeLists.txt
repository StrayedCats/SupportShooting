cmake_minimum_required(VERSION 3.8)
project(support_shooting_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(OpenCV REQUIRED)

file(GLOB_RECURSE SRCS src/*.cpp)
ament_auto_add_library(support_shooting_plugins SHARED
  ${SRCS}
)
ament_target_dependencies(${PROJECT_NAME} OpenCV)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)

  set(TEST_TARGETS
    test_publish_1st_index_plugin)
  foreach(TARGET ${TEST_TARGETS})
    ament_add_gtest(${TARGET} test/src/${TARGET}.cpp)
    target_link_libraries(${TARGET} ${PROJECT_NAME} ${OpenCV_LIBRARIES})
  endforeach()
endif()

pluginlib_export_plugin_description_file(support_shooting_base support_shooting_plugins.xml)

include_directories(include)
ament_auto_package()
