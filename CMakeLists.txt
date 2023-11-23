cmake_minimum_required(VERSION 3.8)
project(basic_sim)

set(CMAKE_BUILD_TYPE "None") 
set(CMAKE_CXX_FLAGS "-std=c++20 -g -O3 ${CMAKE_CXX_FLAGS}")

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)

set(AMENT_DEPS
    rclcpp
    tf2_ros
    tf2_geometry_msgs
    nav_msgs
    sensor_msgs
    yaml_cpp_vendor
    )

foreach(Dependency IN ITEMS ${AMENT_DEPS})
  find_package(${Dependency} REQUIRED)
endforeach()




#Target
include_directories(
    ./include
    third_party/DDA/include
    )

add_executable(basic_sim 
src/BasicSim.cpp
src/Map.cpp
src/Robot.cpp
src/LaserScanner.cpp
)

ament_target_dependencies(basic_sim
${AMENT_DEPS})

target_link_libraries(basic_sim 
${OpenCV_LIBS}
OpenMP::OpenMP_CXX)


# Install
install(
  TARGETS basic_sim
  DESTINATION lib/${PROJECT_NAME}
)

install(
	DIRECTORY test
	DESTINATION share/${PROJECT_NAME}
)
ament_package()
