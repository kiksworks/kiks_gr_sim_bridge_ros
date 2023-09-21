cmake_minimum_required(VERSION 3.8)
project(kiks_gr_sim_bridge)

# find packages
set(ros2_required_packages
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros)

foreach(ros2_required_package IN LISTS ros2_required_packages)
  find_package(${ros2_required_package} REQUIRED)
endforeach()

find_package(ament_cmake REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Qt6 QUIET)
if(Qt6_FOUND)
  find_package(Qt6 COMPONENTS Network REQUIRED)
else()
  find_package(Qt5 COMPONENTS Network REQUIRED)
endif()

# set c and cxx standard
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# set ros distribution definition
if($ENV{ROS_DISTRO} MATCHES "dashing")
  add_definitions(-DKIKS_ROS_DISTRO_DASHING)
elseif($ENV{ROS_DISTRO} MATCHES "foxy")
  add_definitions(-DKIKS_ROS_DISTRO_FOXY)
elseif($ENV{ROS_DISTRO} MATCHES "humble")
  add_definitions(-DKIKS_ROS_DISTRO_HUMBLE)
else()
  message(FATAL_ERROR "unsupported ros distro")
endif()

#generate proto library
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2)
endif()

protobuf_generate_cpp(proto_src proto_hdr
  ${CMAKE_CURRENT_SOURCE_DIR}/3rd/grSim/src/proto/grSim_Packet.proto
  ${CMAKE_CURRENT_SOURCE_DIR}/3rd/grSim/src/proto/grSim_Commands.proto
  ${CMAKE_CURRENT_SOURCE_DIR}/3rd/grSim/src/proto/grSim_Replacement.proto)
add_library(${PROJECT_NAME}_proto_lib ${proto_src})

target_link_libraries(${PROJECT_NAME}_proto_lib
  PRIVATE   protobuf::libprotobuf
  INTERFACE protobuf::libprotobuf
)

target_include_directories(${PROJECT_NAME}_proto_lib
  PRIVATE ${CMAKE_CURRENT_BINARY_DIR}
  INTERFACE ${CMAKE_CURRENT_BINARY_DIR}
)

# generate source library
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wextra -Wpedantic)
endif()

file(GLOB_RECURSE src_files
LIST_DIRECTORIES false
"src/${PROJECT_NAME}/*")

add_library(${PROJECT_NAME}_lib
  ${src_files}
)

target_include_directories(${PROJECT_NAME}_lib
  PRIVATE include
  INTERFACE include
)

#link all liblary to source liblary
target_link_libraries(${PROJECT_NAME}_lib ${PROJECT_NAME}_proto_lib)
if(Qt6_FOUND)
  target_link_libraries(${PROJECT_NAME}_lib Qt6::Network)
else()
  target_link_libraries(${PROJECT_NAME}_lib Qt5::Network)
endif()

ament_target_dependencies(${PROJECT_NAME}_lib ${ros2_required_packages})

# generate executor
file(GLOB executor_files
LIST_DIRECTORIES false
RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/src
"src/*")

foreach(executor_file IN LISTS executor_files)
  string(FIND ${executor_file} . index)
  string(SUBSTRING ${executor_file} 0 ${index} executor)
  set(executors ${executors};${executor})
  add_executable(${executor}
    src/${executor_file}
  )
  target_link_libraries(${executor} ${PROJECT_NAME}_lib)
endforeach()

# install executor
install(TARGETS
  ${executors}
  DESTINATION lib/${PROJECT_NAME}
)

# test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()