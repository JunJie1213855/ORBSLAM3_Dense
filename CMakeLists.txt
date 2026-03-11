cmake_minimum_required(VERSION 3.15)
project(ORB_SLAM3)
SET(CMAKE_BUILD_TYPE Release)
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wall   -O3")

set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")


# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
   add_definitions(-DCOMPILEDWITHC14)
   message(STATUS "Using flag -std=c++14.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV)
if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 4.4 not found.")
endif()

MESSAGE("set CUDA and TensorRT")
set(CUDA_ROOT_DIR "/usr/local/cuda")
set(TensorRT_ROOT_DIR "/root/TensorRT")


MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options)


include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/include/CameraModels
    ${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
    ${PROJECT_SOURCE_DIR}/Thirdparty/TensorRTTemplate/TRTInfer
    ${TensorRT_ROOT_DIR}/include
    ${CUDA_ROOT_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

# FOR PCL
add_definitions( ${PCL_DEFINITIONS} )
link_directories( ${PCL_LIBRARY_DIRS} )

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
   src/PointCloudMapping.cc
   src/StereoMatch.cc
   include/System.h
   include/Tracking.h
   include/LocalMapping.h
   include/LoopClosing.h
   include/ORBextractor.h
   include/ORBmatcher.h
   include/FrameDrawer.h
   include/Converter.h
   include/MapPoint.h
   include/KeyFrame.h
   include/Atlas.h
   include/Map.h
   include/MapDrawer.h
   include/Optimizer.h
   include/Frame.h
   include/KeyFrameDatabase.h
   include/Sim3Solver.h
   include/Viewer.h
   include/ImuTypes.h
   include/G2oTypes.h
   include/CameraModels/GeometricCamera.h
   include/CameraModels/Pinhole.h
   include/CameraModels/KannalaBrandt8.h
   include/OptimizableTypes.h
   include/MLPnPsolver.h
   include/GeometricTools.h
   include/TwoViewReconstruction.h
   include/SerializationUtils.h
   include/Config.h
   include/Settings.h
   include/PointCloudMapping.h
   include/StereoMatch.h
)

# add_subdirectory(Thirdparty/g2o)

target_link_libraries(${PROJECT_NAME}
   ${OpenCV_LIBS}
   ${EIGEN3_LIBS}
   ${Pangolin_LIBRARIES}
   ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
   ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
   ${PROJECT_SOURCE_DIR}/Thirdparty/elas/lib/libelas.so
   ${PROJECT_SOURCE_DIR}/Thirdparty/TensorRTTemplate/lib/libtrtemplate.so
   ${PCL_LIBRARIES}
   -lboost_serialization
   -lcrypto
)


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/MyExample/)
add_executable(rgbd_slam
            MyExample/rgbd_slam.cc)
target_link_libraries(rgbd_slam 
   ${PROJECT_NAME}
   ${Boost_LIBRARIES}
)


add_executable(stereoslam_disp
            MyExample/stereoslam_disp.cc)
target_link_libraries(stereoslam_disp 
   ${PROJECT_NAME}
   ${Boost_LIBRARIES}
)


option(WITH_FILTER "WITH FILTER ?" OFF)
if(WITH_FILTER)
   message("stereo with the wls filter")
   add_definitions(-DWITH_FILTER)
endif()
add_executable(stereoslam
            MyExample/stereoslam.cc)
target_link_libraries(stereoslam 
   ${PROJECT_NAME}
   ${Boost_LIBRARIES}
)


add_executable(zedslam
            MyExample/zedslam.cc)
target_link_libraries(zedslam ${PROJECT_NAME})

# orbbec code
add_executable(orbbec_slam MyExample/orbbec_slam.cc ${PROJECT_SOURCE_DIR}/OrbbecThirdparty/utility/register.hpp)
# header
target_include_directories(orbbec_slam PUBLIC
   ${PROJECT_SOURCE_DIR}/OrbbecThirdparty/Common
   ${PROJECT_SOURCE_DIR}/OrbbecThirdparty/d2c/include
   ${PROJECT_SOURCE_DIR}/OrbbecThirdparty/OpenNI2/include
   ${PROJECT_SOURCE_DIR}/OrbbecThirdparty/utility
)
# lib
target_link_libraries(orbbec_slam 
   ${PROJECT_NAME}
# openni
   ${PROJECT_SOURCE_DIR}/OrbbecThirdparty/OpenNI2/lib/libOpenNI2.so
   ${PROJECT_SOURCE_DIR}/OrbbecThirdparty/OpenNI2/lib/libDepthUtils.a
   ${PROJECT_SOURCE_DIR}/OrbbecThirdparty/OpenNI2/lib/OpenNI2/Drivers/libOniFile.so
   ${PROJECT_SOURCE_DIR}/OrbbecThirdparty/OpenNI2/lib/OpenNI2/Drivers/liborbbec.so
# d2c
   ${PROJECT_SOURCE_DIR}/OrbbecThirdparty/d2c/lib/libd2c.so
)