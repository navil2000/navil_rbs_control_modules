cmake_minimum_required(VERSION 3.5)
project(rbs_mpc)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

# Eigen3 -> https://eigen.tuxfamily.org/dox/TopicCMakeGuide.html
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# IPOPT -> https://coin-or.github.io/Ipopt/INSTALL.html
set(IPOPT_INCLUDE_DIRS /usr/include/coin)
set(IPOPT_LIBRARY_DIRS /usr/lib)
set(IPOPT_LIBRARIES "-lipopt")

# Include directories
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${IPOPT_INCLUDE_DIRS}
)

# Add subdirectories
add_subdirectory(matplotplusplus)

# Set sources
set(SOURCES
  src/mpc_example.cpp
)

# Add executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  ${IPOPT_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/matplotplusplus
)

# Link directories
target_link_directories(${PROJECT_NAME} PUBLIC
  ${IPOPT_LIBRARY_DIRS}
)

# Link libraries
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen matplot ${IPOPT_LIBRARIES})

