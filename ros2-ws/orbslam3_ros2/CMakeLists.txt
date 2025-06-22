cmake_minimum_required(VERSION 3.8)
project(orbslam3_ros2)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
#endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -w -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w -O3")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

add_library(${PROJECT_NAME} SHARED
src/orbslam3_src/System.cc
src/orbslam3_src/Tracking.cc
src/orbslam3_src/LocalMapping.cc
src/orbslam3_src/LoopClosing.cc
src/orbslam3_src/ORBextractor.cc
src/orbslam3_src/ORBmatcher.cc
src/orbslam3_src/FrameDrawer.cc
src/orbslam3_src/Converter.cc
src/orbslam3_src/MapPoint.cc
src/orbslam3_src/KeyFrame.cc
src/orbslam3_src/Atlas.cc
src/orbslam3_src/Map.cc
src/orbslam3_src/MapDrawer.cc
src/orbslam3_src/Optimizer.cc
src/orbslam3_src/Frame.cc
src/orbslam3_src/KeyFrameDatabase.cc
src/orbslam3_src/Sim3Solver.cc
src/orbslam3_src/Viewer.cc
src/orbslam3_src/ImuTypes.cc
src/orbslam3_src/G2oTypes.cc
src/orbslam3_src/CameraModels/Pinhole.cpp
src/orbslam3_src/CameraModels/KannalaBrandt8.cpp
src/orbslam3_src/OptimizableTypes.cpp
src/orbslam3_src/MLPnPsolver.cpp
src/orbslam3_src/GeometricTools.cc
src/orbslam3_src/TwoViewReconstruction.cc
src/orbslam3_src/DepthModule.cc
src/orbslam3_src/Config.cc
src/orbslam3_src/Settings.cc
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/CameraModels
  ${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
)

add_subdirectory(Thirdparty/g2o)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  ${Pangolin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
  -lboost_serialization
  -lcrypto
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  OpenCV
  cv_bridge
  Eigen3
  Pangolin
)

add_executable(track_monocular_node src/track_monocular_node.cpp)
ament_target_dependencies(track_monocular_node rclcpp sensor_msgs OpenCV cv_bridge Eigen3 Pangolin)
target_link_libraries(track_monocular_node ${PROJECT_NAME})

add_executable(track_rgbl_node src/track_rgbl_node.cpp)
ament_target_dependencies(track_rgbl_node rclcpp sensor_msgs OpenCV cv_bridge Eigen3 Pangolin)
target_link_libraries(track_rgbl_node ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  realsense_subscriber
  track_rgbl_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
