cmake_minimum_required (VERSION 2.8)
project (wippe)

set(CMAKE_THREAD_LIBS_INIT "-lpthread")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -std=c++14")
set(CMAKE_HAVE_THREADS_LIBRARY 1)
set(CMAKE_USE_PTHREADS_INIT 1)
set(THREADS_PREFER_PTHREAD_FLAG ON)

set(OpenCV_LIBS opencv_core opencv_imgproc opencv_calib3d opencv_video opencv_features2d opencv_ml opencv_highgui opencv_objdetect)

find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)
find_package(raspicam REQUIRED)

add_executable (wippe src/wippe.cpp)
target_link_libraries (wippe ${raspicam_LIBS})
target_link_libraries (wippe ${raspicam_CV_LIBS})
target_link_libraries (wippe ${OpenCV_LIBS})
