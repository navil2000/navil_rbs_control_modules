cmake_minimum_required(VERSION 3.5)
project(rbs_lqr)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

# Eigen3 -> https://eigen.tuxfamily.org/dox/TopicCMakeGuide.html
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Include directories
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
)

# Add subdirectories
add_subdirectory(matplotplusplus)

# Set sources
set(SOURCES
  src/rbs_lqr.cpp
  src/lqr_example.cpp
)

# Add executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Link libraries
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen matplot)

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  ${PROJECT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/matplotplusplus
)