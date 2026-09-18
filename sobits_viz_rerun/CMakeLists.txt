cmake_minimum_required(VERSION 3.16)
project(sobits_viz_rerun LANGUAGES CXX)

if(NOT DEFINED CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS imgcodecs)

# Rerun C++ SDK.
# The release zip bundles the SDK sources plus a prebuilt librerun_c static lib.
# On the first configure it also downloads and builds a minimal Apache Arrow,
# which takes several minutes; later builds reuse it from the build tree.
# Point RERUN_CPP_URL at a local zip to build without network access.
set(RERUN_CPP_URL
  "https://github.com/rerun-io/rerun/releases/download/0.37.2/rerun_cpp_sdk.zip"
  CACHE STRING "URL or path of the rerun_cpp_sdk archive")
include(FetchContent)
fetchcontent_declare(rerun_sdk URL ${RERUN_CPP_URL})
fetchcontent_makeavailable(rerun_sdk)

# Message conversion, kept free of rclcpp so it can be unit tested on its own.
add_library(${PROJECT_NAME}_codec
  src/image_codec.cpp
)
target_include_directories(${PROJECT_NAME}_codec PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_codec PUBLIC rerun_sdk ${OpenCV_LIBS})
ament_target_dependencies(${PROJECT_NAME}_codec PUBLIC sensor_msgs)

add_executable(bridge
  src/main.cpp
  src/bridge_node.cpp
)
target_include_directories(bridge PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(bridge PRIVATE ${PROJECT_NAME}_codec rerun_sdk ${YAML_CPP_LIBRARIES})
ament_target_dependencies(bridge PUBLIC
  rclcpp
  nav_msgs
  trajectory_msgs
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2_msgs
)

#############
## Install ##
#############

install(TARGETS bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}_codec
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  config
  blueprint
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_image_codec test/test_image_codec.cpp)
  target_include_directories(test_image_codec PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_image_codec ${PROJECT_NAME}_codec ${OpenCV_LIBS})
  ament_target_dependencies(test_image_codec sensor_msgs)
endif()

ament_package()
