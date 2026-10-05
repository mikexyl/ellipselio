cmake_minimum_required(VERSION 3.15)

project(ellipselio)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_POLICY_DEFAULT_CMP0074 NEW)
set(CMAKE_POLICY_DEFAULT_CMP0144 NEW)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Ofast -fopenmp -fPIC")

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(image_transport REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EllipseLioAnalytics.msg"
  DEPENDENCIES std_msgs
)
ament_export_dependencies(rosidl_default_runtime)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

if(EXISTS "/opt/ros/jazzy")
  set(ROS_JAZZY ON CACHE BOOL "Auto-detected ROS Jazzy (/opt/ros/jazzy exists)")
else()
  set(ROS_JAZZY OFF CACHE BOOL "Auto-detected ROS not Jazzy (/opt/ros/jazzy not found)")
endif()

if(ROS_JAZZY)
  message(STATUS "Auto-detected ROS Jazzy: enabling updated headers and includes")
  add_compile_definitions(ROS_JAZZY)
endif()

set(dependencies
  cv_bridge
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  visualization_msgs
  pcl_ros
  pcl_conversions
  image_transport
)

ament_auto_add_library(ellipselio_mapping SHARED src/map_processing.cpp src/lidar_processing.cpp src/imu_processing.cpp src/cam_processing.cpp)
option(ELLIPSELIO_AREA_EXPORT "Export accumulated map observations" ON)
if(ELLIPSELIO_AREA_EXPORT)
  find_package(submap_slam_io REQUIRED)
  target_sources(ellipselio_mapping PRIVATE src/area_maps.cpp src/area_export.cpp)
  target_link_libraries(ellipselio_mapping submap_slam::io)
  target_compile_definitions(ellipselio_mapping PRIVATE ELLIPSELIO_AREA_EXPORT)
  # Keep upstream estimator flags; serialize and validate exports with IEEE finite checks.
  set_source_files_properties(src/area_maps.cpp src/area_export.cpp PROPERTIES COMPILE_OPTIONS "-fno-fast-math")
endif()
target_include_directories(ellipselio_mapping PRIVATE ${PYTHON_INCLUDE_DIRS})

target_include_directories(ellipselio_mapping PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
target_link_libraries(ellipselio_mapping ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} ${EIGEN3_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${cpp_typesupport_target})

ament_target_dependencies(ellipselio_mapping ${dependencies})

rclcpp_components_register_node(ellipselio_mapping
PLUGIN "ellipselio::MappingNode"
EXECUTABLE ellipselio_mapping_node
)

add_executable(ellipselio_mapping_mt src/node_main.cpp)
target_link_libraries(ellipselio_mapping_mt ellipselio_mapping)
ament_target_dependencies(ellipselio_mapping_mt rclcpp rclcpp_components)
install(TARGETS ellipselio_mapping_mt DESTINATION lib/${PROJECT_NAME})
ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR INSTALL_TO_SHARE
launch
rviz
config
)
if(BUILD_TESTING AND ELLIPSELIO_AREA_EXPORT)
  enable_testing()
  add_executable(octree_growth_test tests/octree_growth.cpp)
  target_compile_options(octree_growth_test PRIVATE -UNDEBUG -fno-fast-math)
  target_include_directories(octree_growth_test PRIVATE include ${EIGEN3_INCLUDE_DIR})
  add_test(NAME octree_growth COMMAND octree_growth_test)
  add_executable(area_maps_test tests/area_maps.cpp)
  target_compile_options(area_maps_test PRIVATE -UNDEBUG -fno-fast-math)
  target_compile_definitions(area_maps_test PRIVATE ELLIPSELIO_AREA_EXPORT)
  target_link_libraries(area_maps_test ellipselio_mapping submap_slam::io)
  ament_target_dependencies(area_maps_test ${dependencies})
  add_test(NAME accumulated_area_maps COMMAND area_maps_test)
  add_executable(submap_writer_test tests/research_writer.cpp)
  target_compile_options(submap_writer_test PRIVATE -UNDEBUG)
  target_link_libraries(submap_writer_test ellipselio_mapping submap_slam::io)
  ament_target_dependencies(submap_writer_test rclcpp)
  add_test(NAME bounded_submap_writer COMMAND submap_writer_test /usr/bin/python3
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/slow_writer.py" "${CMAKE_CURRENT_BINARY_DIR}/writer-test.json")
endif()
