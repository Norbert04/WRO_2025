# Add source to this project's executable.
add_executable (WRO_2025 "WRO_2025.cpp" "Connection.cpp" "Movement.cpp" "Movement.h" "Connection.h")

find_library(WIRINGPI_LIBRARIES NAMES wiringPi)
target_link_libraries(WRO_2025 ${WIRINGPI_LIBRARIES})

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET WRO_2025 PROPERTY CXX_STANDARD 20)
endif()
