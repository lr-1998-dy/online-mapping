cmake_minimum_required(VERSION 2.8.3)
project(irls_ndt)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED
             roscpp)

find_package(PCL REQUIRED)

find_package(Eigen3 QUIET)


if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES irls_ndt                   #The exported libraries from the project
  DEPENDS PCL
)

include_directories(
  "include"
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

set(srcs
  src/ndt_scan_matcher.cpp
  src/neighbor_finder.cpp
  src/rigid_estimator.cpp
  src/scan_matcher.cpp
  src/voxel_grid_covariance.cpp
)

set(incs
  include/irls_ndt/ndt_leaf.h
  include/irls_ndt/ndt_scan_matcher.h
  include/irls_ndt/neighbor_finder.h
  include/irls_ndt/rigid_estimator.h
  include/irls_ndt/voxel_grid_covariance.h
)

if(NOT ("${CMAKE_BUILD_TYPE}" STREQUAL "Release"))
  message(WARNING "Not building for release, performance will be slow")

  message(WARNING
  "Adding 'EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT' macro to prevent ndt_matching's runtime error in debug mode.
    The bug reasons and solutions are written in http://eigen.tuxfamily.org/dox-devel/group__TopicUnalignedArrayAssert.html .
    This workaround was discussed on https://gitlab.com/autowarefoundation/autoware.ai/core_perception/merge_requests/57 .")
  add_definitions(-DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)
endif()

add_library(irls_ndt ${incs} ${srcs})

target_link_libraries(irls_ndt
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS irls_ndt
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
