# Add source to this project's executable.
add_executable (WRO_2025 "WRO_2025.cpp" "Connection.cpp"  "DistanceSensor.cpp" "Camera.cpp" "Robot.cpp"  "Connection.h" "DistanceSensor.h" "Camera.h" "Robot.h" "utils.h")

find_library(WIRINGPI_LIBRARIES NAMES wiringPi)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(WRO_2025 ${OpenCV_LIBS} ${WIRINGPI_LIBRARIES})

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET WRO_2025 PROPERTY CXX_STANDARD 20)
endif()
