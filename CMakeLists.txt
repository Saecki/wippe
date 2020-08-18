cmake_minimum_required (VERSION 3.5)
project (wippe)

set (CMAKE_THREAD_LIBS_INIT "-lpthread")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -std=c++17")
set (CMAKE_HAVE_THREADS_LIBRARY 1)
set (CMAKE_USE_PTHREADS_INIT 1)
set (THREADS_PREFER_PTHREAD_FLAG ON)

find_package (PkgConfig REQUIRED)
find_package (OpenCV REQUIRED)
find_package (raspicam REQUIRED)

find_library (wiringPi_LIB wiringPi)

include_directories (${OpenCV_INDCLUDE_DIRS})

add_executable (wippe src/wippe.cpp)

target_link_libraries (wippe ${raspicam_LIBS})
target_link_libraries (wippe ${raspicam_CV_LIBS})
target_link_libraries (wippe ${OpenCV_LIBS})
target_link_libraries (wippe ${wiringPi_LIB})
