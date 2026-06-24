cmake_minimum_required(VERSION 3.8)
project(calousel)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(discocal REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Sophus REQUIRED)

if (OpenMP_FOUND)
    message(STATUS "OpenMP found. Enabling OpenMP compilation flags.")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
else()
    message(WARNING "OpenMP not found. Compilation will proceed without OpenMP acceleration.")
endif()

add_library(calousel_core
  src/key_frame_acquisition.cpp
  src/CLieAlgebra.cpp
  src/optimize.cpp
  src/keyframe_storage.cpp
)

# Add -fPIC flag for calousel_core so it can be linked into shared libraries
set_target_properties(calousel_core PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)

target_include_directories(calousel_core PUBLIC
  "include"
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${discocal_INCLUDE_DIRS}
)

target_link_libraries(calousel_core PUBLIC
  ${discocal_LIBRARIES}
  ${OpenCV_LIBS}
  opencv_flann
  Eigen3::Eigen
  Ceres::ceres
  yaml-cpp
  Sophus::Sophus
)

add_executable(keyframe_extractor_cli
  src/keyframe_extractor_cli.cpp
  src/keyframe_extraction_impl.cpp
)
target_link_options(keyframe_extractor_cli PRIVATE -Wl,--no-as-needed)
target_link_libraries(keyframe_extractor_cli
  calousel_core
)
target_include_directories(keyframe_extractor_cli PRIVATE
  ${discocal_INCLUDE_DIRS}
)
ament_target_dependencies(keyframe_extractor_cli
  rosbag2_cpp
  rclcpp
  sensor_msgs
  cv_bridge
)

add_executable(optimize_extrinsics_cli
  src/optimize_extrinsics_cli.cpp
  src/extrinsic_optimization_impl.cpp
)
target_link_libraries(optimize_extrinsics_cli
  calousel_core
)
target_include_directories(optimize_extrinsics_cli PRIVATE
  ${discocal_INCLUDE_DIRS}
)

add_executable(calousel_pipeline
  src/calousel_pipeline.cpp
  src/keyframe_extraction_impl.cpp
  src/extrinsic_optimization_impl.cpp
)
target_link_options(calousel_pipeline PRIVATE -Wl,--no-as-needed)
target_link_libraries(calousel_pipeline
  calousel_core
)
target_include_directories(calousel_pipeline PRIVATE
  ${discocal_INCLUDE_DIRS}
)
ament_target_dependencies(calousel_pipeline
  rosbag2_cpp
  rclcpp
  sensor_msgs
  cv_bridge
)

install(TARGETS
  keyframe_extractor_cli
  optimize_extrinsics_cli
  calousel_pipeline
  DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
