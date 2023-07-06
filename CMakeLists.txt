cmake_minimum_required(VERSION 2.8.3)
project(sc_ct_icp)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  image_transport
  cv_bridge
  tf
)

#find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

find_package(OpenMP REQUIRED)
find_package(GTSAM REQUIRED QUIET)

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL 
  INCLUDE_DIRS include
)

# add_executable(sc_ascanRegistration src/scanRegistration.cpp)
# target_link_libraries(sc_ascanRegistration ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# add_executable(sc_alaserOdometry src/laserOdometry.cpp)
# target_link_libraries(sc_alaserOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

# add_executable(sc_alaserMapping src/laserMapping.cpp)
# target_link_libraries(sc_alaserMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(pub_pcd src/pub_pcd.cpp)
target_link_libraries(pub_pcd ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

# add_executable(pub_hilti src/pub_hilti.cpp)
# target_link_libraries(pub_hilti ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(SC_PGO 
  src/laserPosegraphOptimization.cpp
  include/scancontext/Scancontext.cpp
)
target_compile_options(SC_PGO 
  PRIVATE ${OpenMP_CXX_FLAGS}
)
target_link_libraries(SC_PGO 
  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
  gtsam
)

# add_executable(kittiHelper src/kittiHelper.cpp)
# target_link_libraries(kittiHelper ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(log_sc_cticp src/log_sc_cticp.cpp)
target_link_libraries(log_sc_cticp ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})