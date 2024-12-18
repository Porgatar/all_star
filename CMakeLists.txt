cmake_minimum_required(VERSION 3.8)
project(all_star)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Werror -Wextra -Wpedantic -Wconversion -O3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(ZBAR REQUIRED zbar)
include_directories(${ZBAR_INCLUDE_DIRS})
link_directories(${ZBAR_LIBRARY_DIRS})

add_executable(BabySharky src/main.cpp src/BabySharky.cpp src/ImageProcessorCallback.cpp src/TargetStanCallback.cpp src/MainProcessorCallback.cpp src/Search.cpp src/Rally.cpp)
ament_target_dependencies(BabySharky rclcpp std_msgs sensor_msgs ros_gz_interfaces geometry_msgs cv_bridge image_transport OpenCV)
target_link_libraries(BabySharky ${ZBAR_LIBRARIES})
target_include_directories(BabySharky PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(BabySharky PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS BabySharky
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
