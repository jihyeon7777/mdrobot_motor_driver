cmake_minimum_required(VERSION 3.14)
project(mdrobot_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

# --- library -----------------------------------------------------------
add_library(${PROJECT_NAME}
  src/crc.cpp
  src/codec.cpp
  src/frame.cpp
  src/status.cpp
  src/units.cpp
  src/transport.cpp
  src/protocol.cpp
  src/device.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# Build PIC so this static lib can be linked into shared libraries
# (e.g. the mdrobot_ros2_control pluginlib plugin .so).
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

# --- install -----------------------------------------------------------
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

# --- tests -------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  set(TEST_SOURCES
    test/test_crc.cpp
    test/test_codec.cpp
    test/test_frame.cpp
    test/test_protocol.cpp
    test/test_status.cpp
    test/test_units.cpp
    test/test_device.cpp
    test/test_transport.cpp
  )
  foreach(TEST_SRC ${TEST_SOURCES})
    get_filename_component(TEST_NAME ${TEST_SRC} NAME_WE)
    ament_add_gtest(${TEST_NAME} ${TEST_SRC})
    target_link_libraries(${TEST_NAME} ${PROJECT_NAME})
  endforeach()
endif()

ament_package()
