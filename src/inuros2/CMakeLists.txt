cmake_minimum_required(VERSION 3.8)
project(inuros2)

set(CMAKE_CXX_STANDARD 17)

add_definitions(-D_GE428_)
add_definitions(-D_GE430_)

#add_definitions(-DUSE_LIFECYCLE_MANAGEMENT)

## Paths for InuDev components. May be overriden externally.
if(DEFINED ENV{INUITIVE_STREAMS_INCLUDE})
  set(INUITIVE_STREAMS_LIBS $ENV{INUITIVE_STREAMS_LIBS})
  set(INUITIVE_STREAMS_INCLUDE $ENV{INUITIVE_STREAMS_INCLUDE})
  set(INUITIVE_COMMON_INCLUDE $ENV{INUITIVE_COMMON_INCLUDE})
  set(INUITIVE_BIN $ENV{INUITIVE_BIN})
else()
  set(INUITIVE_STREAMS_LIBS /opt/Inuitive/InuDev/lib)
  set(INUITIVE_STREAMS_INCLUDE /opt/Inuitive/InuDev/include)
  set(INUITIVE_COMMON_INCLUDE /opt/Inuitive/InuDev/include/Common)
  set(INUITIVE_BIN /opt/Inuitive/InuDev/bin)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -W   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -W  -O3")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")


SET(USE_PC_FILTER false CACHE BOOL "whether using point cloud filter")
if (USE_PC_FILTER)
   add_definitions(-DUSE_PC_FILTER)
endif()

# cmake_host_system_information(RESULT PRETTY_NAME QUERY DISTRIB_PRETTY_NAME)
# message(STATUS "${PRETTY_NAME}")

#cmake_host_system_information(RESULT DISTRO QUERY DISTRIB_INFO)
message(STATUS "${DISTRO_UBUNTU_CODENAME}")
#foreach(VAR IN LISTS DISTRO)
#  message(STATUS "${VAR}=`${${VAR}}`")
#endforeach()

# Status message
MESSAGE("$ENV{ROS_DISTRO}")

if(NOT( ($ENV{ROS_DISTRO} MATCHES "foxy") OR ($ENV{ROS_DISTRO} MATCHES "galactic") OR ($ENV{ROS_DISTRO} MATCHES "eloquent")))
    set(HIGH_ROS "1")
    add_definitions("-DHIGH_ROS=${HIGH_ROS}")
endif()

MESSAGE("${HIGH_ROS}")

if (${DISTRO_UBUNTU_CODENAME} MATCHES "bionic")
  include_directories(include
        src/filter
        /usr/include/eigen3
        /opt/ros/$ENV{ROS_DISTRO}/include
        /opt/ros/$ENV{ROS_DISTRO}/include/pcl_msgs
        /opt/Inuitive/InuDev/include/InuCommon
        /usr/include/python3.6m
        /usr/include/pcl-1.8
        ${INUITIVE_STREAMS_INCLUDE}
        )
  add_definitions("-DUSE_H=1")
else()
  include_directories(include
        src/filter
        /usr/include/eigen3
        /opt/ros/$ENV{ROS_DISTRO}/include
        /opt/ros/$ENV{ROS_DISTRO}/include/pcl_msgs
        /opt/Inuitive/InuDev/include/InuCommon
        ${INUITIVE_STREAMS_INCLUDE}
        )
endif()

set(src_filter
  src/filter/depth2pc.cpp
  src/filter/space_filter.cpp
  src/filter/temporal_filter.cpp
)

if (USE_PC_FILTER)
  set(src_filter
    ${src_filter}
    src/filter/pc_filter.cpp
  )
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
#find_package(inuros2_messages REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

find_library(InuSw_Streams_Lib REQUIRED NAMES InuStreams PATHS ${INUITIVE_STREAMS_LIBS})
find_library(InuSw_CommonUtils_Lib REQUIRED NAMES InuCommonUtilities PATHS ${INUITIVE_STREAMS_LIBS})
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set (SOURCES
        ${src_filter}
        src/ros_publisher.cpp
        src/ros_sensor.cpp
        src/ros_webcam_publisher.cpp
        src/ros_publisher_bookkeeping.cpp
        src/inuros2_node_factory.cpp
        src/ros_fisheye_publisher.cpp
        src/ros_imu_publisher.cpp
        #src/ros_slam_publisher.cpp
        src/ros_video_publishers_base.cpp
        src/ros_video_publisher.cpp
        src/ros_depth_publisher.cpp
        src/ros_pointcloud_publisher.cpp
        src/ros_features_publisher.cpp
        #src/ros_objectdetection_publisher.cpp
        )

set (INCLUDES
        include/ros_sensor.h
        include/ros_publisher.h
        include/ros_webcam_publisher.h
        include/ros_publisher_bookkeeping.h
        include/inuros2_node_factory.h
        include/ros_fisheye_publisher.h
        include/ros_imu_publisher.h
        include/ros_slam_publisher.h
        include/ros_video_publishers_base.h
        include/ros_video_publisher.h
        include/ros_depth_publisher.h
        include/ros_pointcloud_publisher.h
        include/ros_features_publisher.h
        include/ros_objectdetection_publisher.h
        include/config.h
        )

set(dependencies
    cv_bridge
    rclcpp
    rclcpp_components
    image_transport
    std_msgs
    sensor_msgs
    nav_msgs
    #inuros2_messages
    vision_msgs
    rclcpp_lifecycle
    pcl_conversions
    )

set(library_name inuros2_node)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Features.msg"
  DEPENDENCIES std_msgs
)

add_library(${library_name} SHARED
        ${INCLUDES}
        ${SOURCES}
        )

if(($ENV{ROS_DISTRO} MATCHES "foxy") OR ($ENV{ROS_DISTRO} MATCHES "galactic") OR ($ENV{ROS_DISTRO} MATCHES "eloquent"))
  rosidl_target_interfaces(${library_name} ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
  rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
  target_link_libraries(${library_name} "${cpp_typesupport_target}")
endif()

#target_include_directories(${library_name} PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>
#        ${OpenCV_INCLUDE_DIRS}
#        ${INUITIVE_INCLUDE_DIRS}
#        ${INUITIVE_COMMON_INCLUDE}
#        )

ament_target_dependencies(${library_name}
${dependencies}
)

target_link_libraries(${library_name}
        ${InuSw_Streams_Lib}
        ${InuSw_CommonUtils_Lib}
        -lstdc++fs
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        #${INUITIVE_BIN}/libopencv_highgui.so.4.5
        #${INUITIVE_BIN}/libopencv_imgcodecs.so.4.5
        #${INUITIVE_BIN}/libopencv_imgproc.so.4.5
        #${INUITIVE_BIN}/libopencv_core.so.4.5
        )

install(TARGETS ${library_name}
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION bin
)

rclcpp_components_register_node(${library_name}
  PLUGIN "inuros2::CInuDevRosNodeFactory"
  EXECUTABLE inuros2_camera_node
)

target_include_directories(inuros2_camera_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
        ${OpenCV_INCLUDE_DIRS}
        ${INUITIVE_INCLUDE_DIRS}
        ${INUITIVE_COMMON_INCLUDE}
        )
target_compile_features(inuros2_camera_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

#target_link_libraries(${PROJECT_NAME}
#  ${InuSw_Streams_Lib}
#)

## Specify libraries to link a library or executable target against
target_link_libraries(inuros2_camera_node
        ${InuSw_Streams_Lib}
        ${InuSw_CommonUtils_Lib}
        -lstdc++fs
        ${catkin_LIBRARIES}
        #${INUITIVE_BIN}/libopencv_highgui.so.4.5
        #${INUITIVE_BIN}/libopencv_imgcodecs.so.4.5
        #${INUITIVE_BIN}/libopencv_imgproc.so.4.5
        #${INUITIVE_BIN}/libopencv_core.so.4.5
        )

ament_target_dependencies(inuros2_camera_node
${dependencies}
)

  # Install binaries
install(TARGETS inuros2_camera_node
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION bin
)

# Install headers
install(
DIRECTORY include/
DESTINATION include
)

install(FILES
    ${InuSw_Streams_Lib}
    ${InuSw_CommonUtils_Lib}
    DESTINATION lib
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  )

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies} )

ament_package()
