cmake_minimum_required(VERSION 2.8.3)
project(revopoint_imu_fusion)

# set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++14 -O3 -lboost_system -msse2 -msse3 -pthread -Wenum-compare") # -Wall
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# Add OpenCV if available
FIND_PACKAGE(Boost REQUIRED COMPONENTS filesystem iostreams program_options system serialization)
if(Boost_FOUND)
	INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
	LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})
endif()

find_package(catkin REQUIRED COMPONENTS
                    roscpp
                    std_msgs
                    geometry_msgs
                    nav_msgs
                    tf
                    cv_bridge
            )

# find_package(Ceres REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

### Find OpenMP #######
FIND_PACKAGE(OpenMP)
if(OPENMP_FOUND)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
	#cmake only check for separate OpenMP library on AppleClang 7+
	#https://github.com/Kitware/CMake/blob/42212f7539040139ecec092547b7d58ef12a4d72/Modules/FindOpenMP.cmake#L252
	if (CMAKE_CXX_COMPILER_ID MATCHES "AppleClang" AND (NOT CMAKE_CXX_COMPILER_VERSION VERSION_LESS "7.0"))
		SET(OpenMP_LIBS ${OpenMP_libomp_LIBRARY})
		LIST(APPEND OpenMVS_EXTRA_LIBS ${OpenMP_LIBS})
	endif()
else()
	message("-- Can't find OpenMP. Continuing without it.")
endif()

# message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")
include_directories(${catkin_INCLUDE_DIRS})

generate_messages(
 DEPENDENCIES
 geometry_msgs
)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ./inc/FOV_Checker
  ./inc/kd_tree
  ./inc/lib_sophus
  ./inc/optical_flow
  ./inc/rgb_map
  ./inc/tools
  ./inc
)


catkin_package()

message(STATUS "===== ${PROJECT_NAME}: OpenCV library status: =====")
message(STATUS "===== OpenCV ersion: ${OpenCV_VERSION}  =====")
message(STATUS "===== OpenCV libraries: ${OpenCV_LIBS} =====")
message(STATUS "===== OpenCV include path: ${OpenCV_INCLUDE_DIRS} =====")

add_executable(revopoint_pointcloud_handler src/RPAPointcloudHandler)
target_link_libraries(revopoint_pointcloud_handler ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(revopoint_mapping src/RPARosNode.cpp 
                src/RPALIOSubsystem.cpp
                src/kd_tree/ikd_Tree.cpp
                src/FOV_Checker/RPAFOVChecker.cpp 
                src/RPAIMUHandler.cpp
                # From VIO
                src/RPAVIOSubsystem.cpp
                src/optical_flow/lkpyramid.cpp
                src/rgb_map/RPAColorMapTracker.cpp
                src/rgb_map/RPAImageFrame.cpp
                src/rgb_map/RPAPointcloudRgbd.cpp
              )
target_link_libraries(revopoint_mapping 
                          ${catkin_LIBRARIES}
                          ${Boost_LIBRARIES}
                          ${Boost_FILESYSTEM_LIBRARY}
                          ${Boost_SERIALIZATION_LIBRARY} # serialization
                          ${OpenCV_LIBRARIES}
                          ${PCL_LIBRARIES}
                          pcl_common 
                          pcl_io) 