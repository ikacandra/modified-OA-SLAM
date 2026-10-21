cmake_minimum_required(VERSION 3.10)
project(OA_SLAM3)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-deprecated-declarations -O3")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

# OA-SLAM needs C++17 (structured bindings, ...).
# COMPILEDWITHC11 selects std::chrono::steady_clock in the ORB-SLAM3 sources.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -faligned-new")
add_definitions(-DCOMPILEDWITHC11)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 4.4)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 4.4 not found.")
   endif()

MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})
# OpenCV DNN backend of the YOLOv8-seg detector (.onnx)
add_definitions(-DUSE_DNN)

# TensorRT (optional): YOLOv8-seg detector from .engine files (e.g. FP16 on Jetson).
find_path(TENSORRT_INCLUDE_DIR NvInfer.h PATH_SUFFIXES aarch64-linux-gnu x86_64-linux-gnu)
find_library(TENSORRT_NVINFER_LIB nvinfer)
find_path(CUDA_RUNTIME_INCLUDE_DIR cuda_runtime_api.h PATHS /usr/local/cuda/include)
find_library(CUDA_CUDART_LIB cudart PATHS /usr/local/cuda/lib64)
if(TENSORRT_INCLUDE_DIR AND TENSORRT_NVINFER_LIB AND CUDA_RUNTIME_INCLUDE_DIR AND CUDA_CUDART_LIB)
   message("TensorRT found: ${TENSORRT_NVINFER_LIB}. TRTDetector (.engine) enabled.")
   add_definitions(-DUSE_TENSORRT)
   set(TENSORRT_LIBS ${TENSORRT_NVINFER_LIB} ${CUDA_CUDART_LIB})
   set(TENSORRT_INCLUDE_DIRS ${TENSORRT_INCLUDE_DIR} ${CUDA_RUNTIME_INCLUDE_DIR})
else()
   message(WARNING "TensorRT/CUDA runtime not found. TRTDetector (.engine) will not be available.")
endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(realsense2)

# JSON
set(JSON_BuildTests OFF CACHE INTERNAL "")
add_subdirectory(Thirdparty/json)

# Dlib (Hungarian algorithm for the object association)
find_package(dlib REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/include/CameraModels
${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${TENSORRT_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
# ORB-SLAM3 backbone
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
src/Atlas.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Viewer.cc
src/ImuTypes.cc
src/G2oTypes.cc
src/CameraModels/Pinhole.cpp
src/CameraModels/KannalaBrandt8.cpp
src/OptimizableTypes.cpp
src/MLPnPsolver.cpp
src/GeometricTools.cc
src/TwoViewReconstruction.cc
src/Config.cc
src/Settings.cc
# OA-SLAM object layer
src/ImageDetections.cc
src/TRTDetector.cc
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
src/p3p.cc
src/Localization.cc
)

add_subdirectory(Thirdparty/g2o)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
-lboost_serialization
-lcrypto
nlohmann_json::nlohmann_json
stdc++fs
dlib::dlib
${TENSORRT_LIBS}
)

# If RealSense SDK is found the library is added
if(realsense2_FOUND)
    include_directories(${PROJECT_NAME}
    ${realsense_INCLUDE_DIR}
    )
    target_link_libraries(${PROJECT_NAME}
    ${realsense2_LIBRARY}
    )
endif()


# Build examples

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable(oa-slam
Examples/oa-slam.cc)
target_link_libraries(oa-slam ${PROJECT_NAME})

add_executable(oa-slam_rgb-d-offline
Examples/oa-slam-rgb-d-offline.cc)
target_link_libraries(oa-slam_rgb-d-offline ${PROJECT_NAME})
