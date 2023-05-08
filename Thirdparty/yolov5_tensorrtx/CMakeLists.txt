cmake_minimum_required(VERSION 3.16)

project(yolov5)

add_definitions(-std=c++14)
add_definitions(-DAPI_EXPORTS)
option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Debug)
add_definitions(-w)
find_package(CUDA REQUIRED)

if(WIN32)
enable_language(CUDA)
endif(WIN32)
add_definitions(-w)
include_directories(${PROJECT_SOURCE_DIR}/include)
# include and link dirs of cuda and tensorrt, you need adapt them if yours are different
# cuda
include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)
# tensorrt
include_directories(/media/ubuntu-slam/1T/Thirdparty/TensorRT-7.2.3.4.Ubuntu-18.04.x86_64-gnu.cuda-11.1.cudnn8.1/TensorRT-7.2.3.4/include)
link_directories(/media/ubuntu-slam/1T/Thirdparty/TensorRT-7.2.3.4.Ubuntu-18.04.x86_64-gnu.cuda-11.1.cudnn8.1/TensorRT-7.2.3.4/lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Ofast -g -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDED")
#cuda_add_library(myplugins SHARED yololayer.cu)
#target_link_libraries(myplugins nvinfer cudart)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

#cuda_add_executable(yolov5 calibrator.cpp yolov5.cpp preprocess.cu yololayer.cu)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
cuda_add_library(yolov5 SHARED calibrator.cpp yolov5.cpp preprocess.cu yololayer.cu)

#target_link_libraries(yolov5 nvinfer)
#target_link_libraries(yolov5 cudart)
#target_link_libraries(yolov5 myplugins)
#target_link_libraries(yolov5 ${OpenCV_LIBS})
target_link_libraries(yolov5
	nvinfer
	cudart
	${OpenCV_LIBS})
