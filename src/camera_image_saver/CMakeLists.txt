cmake_minimum_required(VERSION 3.8)
project(camera_image_saver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(image_saver_node src/image_saver_node.cpp)

ament_target_dependencies(image_saver_node
  rclcpp
  sensor_msgs
  cv_bridge
)

target_link_libraries(image_saver_node ${OpenCV_LIBS})

install(TARGETS
  image_saver_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
