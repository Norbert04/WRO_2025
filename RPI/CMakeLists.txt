# Add source to this project's executable.
add_executable (WRO_2025 "WRO_2025.cpp" "Connection.cpp" "Movement.cpp" "DistanceSensor.cpp" "Camera.cpp" "Movement.h" "Connection.h" "DistaceSensor.h" "Camera.h")

find_library(WIRINGPI_LIBRARIES NAMES wiringPi)
target_link_libraries(WRO_2025 ${WIRINGPI_LIBRARIES})

# Find libcamera
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED libcamera)

# Link libcamera
target_include_directories(WRO_2025 PRIVATE ${LIBCAMERA_INCLUDE_DIRS})
target_link_libraries(WRO_2025 ${LIBCAMERA_LIBRARIES})

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET WRO_2025 PROPERTY CXX_STANDARD 20)
endif()
