cmake_minimum_required(VERSION 3.16)
project(comma_camera CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(TURBOJPEG REQUIRED IMPORTED_TARGET libturbojpeg)

# NV12 conversion + JPEG encoding, independent of ROS and openpilot
add_library(comma_camera_core STATIC src/convert.cpp src/jpeg.cpp)
target_include_directories(comma_camera_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(comma_camera_core PUBLIC PkgConfig::TURBOJPEG)
set_target_properties(comma_camera_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ---- VisionIPC from an openpilot checkout built with scons ----
# OPENPILOT_DIR: the openpilot repository (with msgq_repo built)
# OPENPILOT_LIB_DIRS: extra directories with json11 / zmq / capnp static libraries
#                     (the LIB_DIR of openpilot's vendored dependency packages)
set(OPENPILOT_DIR "$ENV{OPENPILOT_DIR}" CACHE PATH "openpilot checkout with msgq built by scons")
set(OPENPILOT_LIB_DIRS "$ENV{OPENPILOT_LIB_DIRS}" CACHE STRING "extra library directories for openpilot dependencies")

set(_op_search
  ${OPENPILOT_DIR}/msgq_repo ${OPENPILOT_DIR}/msgq_repo/msgq ${OPENPILOT_DIR}/msgq_repo/msgq/visionipc
  ${OPENPILOT_DIR}/openpilot/common ${OPENPILOT_LIB_DIRS})

set(_op_libs "")
set(_op_missing "")
foreach(lib visionipc msgq common json11 zmq)
  find_library(OP_${lib}_LIB NAMES ${lib} PATHS ${_op_search} NO_DEFAULT_PATH)
  if(OP_${lib}_LIB)
    list(APPEND _op_libs ${OP_${lib}_LIB})
  else()
    list(APPEND _op_missing ${lib})
  endif()
endforeach()
# optional: needed only if the vendored libraries reference them
foreach(lib capnp kj)
  find_library(OP_${lib}_LIB NAMES ${lib} PATHS ${_op_search} NO_DEFAULT_PATH)
  if(OP_${lib}_LIB)
    list(APPEND _op_libs ${OP_${lib}_LIB})
  endif()
endforeach()
find_library(OPENCL_LIB OpenCL)

if(OPENPILOT_DIR AND EXISTS "${OPENPILOT_DIR}/msgq_repo/msgq/visionipc/visionipc_client.h" AND NOT _op_missing)
  add_library(camera_component SHARED src/camera_node.cpp)
  target_include_directories(camera_component PRIVATE ${OPENPILOT_DIR}/msgq_repo ${OPENPILOT_DIR}/openpilot)
  target_link_libraries(camera_component comma_camera_core -Wl,--start-group ${_op_libs} -Wl,--end-group pthread)
  if(OPENCL_LIB)
    target_link_libraries(camera_component ${OPENCL_LIB})
  endif()
  ament_target_dependencies(camera_component rclcpp rclcpp_components sensor_msgs)
  rclcpp_components_register_node(camera_component
    PLUGIN "comma_camera::CameraNode"
    EXECUTABLE camera_node)
  install(TARGETS camera_component
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
else()
  message(WARNING
    "comma_camera: VisionIPC not found (OPENPILOT_DIR='${OPENPILOT_DIR}', missing: ${_op_missing}). "
    "Only the conversion library and its tests are built; the camera node needs openpilot's msgq built with scons.")
endif()

install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_convert test/test_convert.cpp)
  target_link_libraries(test_convert comma_camera_core)
endif()

ament_package()
