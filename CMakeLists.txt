cmake_minimum_required(VERSION 3.5)
project(pcl2marker)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

include_directories(include)

link_libraries(
  ${PCL_LIBRARIES}
  ${pcl_conversions_LIBRARIES}
)


add_executable(pcl2copy src/copy.cpp)
ament_target_dependencies(pcl2copy vision_msgs visualization_msgs OpenCV cv_bridge PCL rclcpp std_msgs sensor_msgs pcl_conversions )

add_executable(pcl2marker src/pcl2marker.cpp)
ament_target_dependencies(pcl2marker visualization_msgs OpenCV cv_bridge PCL rclcpp std_msgs sensor_msgs pcl_conversions )

add_executable(pcltest src/test.cpp)
ament_target_dependencies(pcltest visualization_msgs OpenCV cv_bridge PCL rclcpp std_msgs sensor_msgs pcl_conversions )


install(TARGETS
 pcl2copy
 pcl2marker
 pcltest
 DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
