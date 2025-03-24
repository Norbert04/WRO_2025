# Add source to this project's executable.
add_executable (WRO_2025 "WRO_2025.cpp" "Connection.cpp" "Movement.cpp" "DistanceSensor.cpp" "Camera.cpp" "Movement.h" "Connection.h" "DistaceSensor.h" "Camera.h")

find_library(WIRINGPI_LIBRARIES NAMES wiringPi)
find_library(LIBCAMERA_LIBRARIES NAMES libcamera)
target_link_libraries(WRO_2025 ${LIBCAMERA_LIBRARIES} ${WIRINGPI_LIBRARIES})

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET WRO_2025 PROPERTY CXX_STANDARD 20)
endif()
