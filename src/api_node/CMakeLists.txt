cmake_minimum_required(VERSION 3.8)
project(api_node)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Drogon REQUIRED)

# Find jsoncpp library
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)

# Include directories
include_directories(
    /usr/include/jsoncpp
    ${JSONCPP_INCLUDE_DIRS}  # Optional, if pkg-config finds it
)

# Add executables
add_executable(api_node_publisher src/api_node_publisher.cpp)
add_executable(api_node_subscriber src/api_node_subscriber.cpp) 


# Link dependencies (DO NOT link std_msgs::std_msgs)
ament_target_dependencies(api_node_publisher rclcpp std_msgs Drogon)
ament_target_dependencies(api_node_subscriber rclcpp std_msgs Drogon)

target_include_directories(api_node_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(api_node_subscriber PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link jsoncpp and Drogon
target_link_libraries(api_node_publisher ${JSONCPP_LIBRARIES} Drogon::Drogon)
target_link_libraries(api_node_subscriber ${JSONCPP_LIBRARIES} Drogon::Drogon)

# Install targets
install(TARGETS
    api_node_publisher
    api_node_subscriber
    DESTINATION lib/${PROJECT_NAME})

ament_package()
