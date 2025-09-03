cmake_minimum_required(VERSION 3.8)
project(basic_sim)

set(CMAKE_BUILD_TYPE "Release") 
set(CMAKE_CXX_FLAGS_Release "-g -O3 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_Debug "-g -O0 ${CMAKE_CXX_FLAGS}")

set(CMAKE_CXX_STANDARD 20)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)

find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
find_package(yaml-cpp)
ament_auto_find_build_dependencies()

#Target
include_directories(
    ./include
    third_party/DDA/include
    )

ament_auto_add_executable(basic_sim 
    src/BasicSim.cpp
    src/Map.cpp
    src/Robot.cpp
    src/LaserScanner.cpp
)

target_link_libraries(basic_sim 
    ${OpenCV_LIBS}
    OpenMP::OpenMP_CXX
    yaml-cpp::yaml-cpp)


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
