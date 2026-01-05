cmake_minimum_required(VERSION 3.10)
project(ORB_SLAM2)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_CXX_FLAGS "-std=c++17 -faligned-new -Wall -Wno-deprecated-declarations -O3 -march=native")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV REQUIRED)
if(NOT OpenCV_FOUND)
   message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
else()
   if(NOT "${OpenCV_VERSION_MAJOR}" STREQUAL "4")
      message(WARNING "OpenCV 4 is required for dnn module. Object detector will not be available.")
   else()
      message("OpenCV 4 found. Make sure OpenCV is built with dnn module and CUDA.")
      add_definitions(-DUSE_DNN)
   endif()
endif()



find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Protobuf REQUIRED)

# JSON
set(JSON_BuildTests OFF CACHE INTERNAL "")
add_subdirectory(Thirdparty/json)

# Dlib
find_package(dlib REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${PROTOBUF_INCLUDE_DIR}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
src/Viewer.cc
src/Osmap.cpp
src/osmap.pb.cc
src/ImageDetections.cc
src/Ellipse.cc
src/Ellipsoid.cc
src/ObjectTrack.cc
src/Distance.cc
src/MapObject.cc
src/OptimizerObject.cc
src/LocalObjectMapping.cc
src/Utils.cc
src/ColorManager.cc
src/Reconstruction.cc
src/Camera.cc
src/Scene.cc
src/p3p.cc
src/Localization.cc
src/ARViewer.cc
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
${PROTOBUF_LIBRARIES}
nlohmann_json::nlohmann_json
stdc++fs
dlib::dlib
)



# Build Examples

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable(oa-slam
Examples/oa-slam.cc)
target_link_libraries(oa-slam ${PROJECT_NAME})

add_executable(oa-slam_localization
Examples/oa-slam_localization.cc)
target_link_libraries(oa-slam_localization ${PROJECT_NAME})

add_executable(oa-slam_rgb-d-offline Examples/oa-slam-rgb-d-offline.cc)
target_link_libraries(oa-slam_rgb-d-offline ${PROJECT_NAME})


# ... (Phần cuối file CMakeLists.txt)

# --- ROS 2 Integration ---
# Tìm các gói ROS 2 cần thiết
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)

# Thêm đường dẫn include cho ROS 2
include_directories(
    ${rclcpp_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${cv_bridge_INCLUDE_DIRS}
    /opt/ros/humble/include/cv_bridge  # [FIX] Thêm dòng này để tìm thấy <cv_bridge/cv_bridge.h>
    ${image_transport_INCLUDE_DIRS}
    ${message_filters_INCLUDE_DIRS}
)

# Tạo executable cho ROS node
add_executable(ros_oa_slam_rgbd Examples/ros_oa_slam_rgbd.cc)

# Link thư viện OA-SLAM và các thư viện ROS 2
target_link_libraries(ros_oa_slam_rgbd 
    ${PROJECT_NAME}
    rclcpp::rclcpp
    ${sensor_msgs_LIBRARIES}
    /opt/ros/humble/lib/libcv_bridge.so  # [FIX] Trỏ thẳng vào file thư viện .so
    ${message_filters_LIBRARIES}
    ${image_transport_LIBRARIES}
)
# Fix lỗi C++ standard nếu cần (ROS 2 Humble cần C++17, Foxy cần C++14)
# Nếu build bị lỗi standard, bỏ comment dòng dưới:
# target_compile_features(ros_oa_slam_rgbd PUBLIC cxx_std_17)