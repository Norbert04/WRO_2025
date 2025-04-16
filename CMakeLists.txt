cmake_minimum_required (VERSION 3.12)

set (CMAKE_CXX_STANDARD 20)

set(CMAKE_THREAD_LIBS_INIT "-lpthread")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_HAVE_THREADS_LIBRARY 1)
set(CMAKE_USE_WIN32_THREADS_INIT 0)
set(CMAKE_USE_PTHREADS_INIT 1)
set(THREADS_PREFER_PTHREAD_FLAG ON)

IF(CMAKE_BUILD_TYPE MATCHES "Debug")
    add_compile_definitions(_DEBUG)
ENDIF(CMAKE_BUILD_TYPE MATCHES "Debug")

project ("WRO_2025")

# Include sub-projects.
add_subdirectory ("RPI")
