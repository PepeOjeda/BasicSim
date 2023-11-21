cmake_minimum_required(VERSION 3.8)
project(basic_sim)

set(CMAKE_BUILD_TYPE "None") 
set(CMAKE_CXX_FLAGS "-std=c++20 -g -O0 ${CMAKE_CXX_FLAGS}")

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
set(AMENT_DEPS
    rclcpp
    tf2_ros
    tf2_geometry_msgs
    nav_msgs
    )

foreach(Dependency IN ITEMS ${AMENT_DEPS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_subdirectory(third_party/yaml-cpp)



#Target
include_directories(
    ./include
    third_party/yaml-cpp/include
    )

add_executable(basic_sim 
src/BasicSim.cpp
src/Robot.cpp
src/Map.cpp
)

ament_target_dependencies(basic_sim
${AMENT_DEPS})
target_link_libraries(basic_sim yaml-cpp::yaml-cpp ${OpenCV_LIBS})


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
