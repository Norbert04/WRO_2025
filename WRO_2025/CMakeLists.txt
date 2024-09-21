# Add source to this project's executable.
add_executable (WRO_2025 "WRO_2025.cpp" "Connection.h" "Connection.cpp" "Movement.h" "Movement.cpp")

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET WRO_2025 PROPERTY CXX_STANDARD 20)
endif()
