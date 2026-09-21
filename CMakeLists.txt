cmake_minimum_required(VERSION 3.15)

project(ellipselio)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_POLICY_DEFAULT_CMP0074 NEW)
set(CMAKE_POLICY_DEFAULT_CMP0144 NEW)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fopenmp -fPIC")

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

ament_auto_add_library(ellipselio_mapping SHARED src/area_maps.cpp src/submaps.cpp src/map_processing.cpp src/lidar_processing.cpp src/imu_processing.cpp src/cam_processing.cpp)
if(S3E_RESEARCH_SOURCE)
  find_package(nlohmann_json REQUIRED)
  target_sources(ellipselio_mapping PRIVATE src/research_frame.cpp "${S3E_RESEARCH_SOURCE}/src/research_export.cpp")
  target_include_directories(ellipselio_mapping PRIVATE "${S3E_RESEARCH_SOURCE}/include")
  target_link_libraries(ellipselio_mapping nlohmann_json::nlohmann_json)
  target_compile_definitions(ellipselio_mapping PRIVATE ELLIPSELIO_RESEARCH_EXPORT)
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

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR INSTALL_TO_SHARE
launch
rviz
config
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(submaps_test tests/submaps.cpp)
  target_compile_options(submaps_test PRIVATE -UNDEBUG)
  target_link_libraries(submaps_test ellipselio_mapping)
  ament_target_dependencies(submaps_test ${dependencies})
  if(S3E_RESEARCH_SOURCE)
    target_compile_definitions(submaps_test PRIVATE ELLIPSELIO_RESEARCH_EXPORT)
    target_include_directories(submaps_test PRIVATE "${S3E_RESEARCH_SOURCE}/include")
  endif()
  add_test(NAME recent_history_submaps COMMAND submaps_test)
  if(S3E_RESEARCH_SOURCE)
    add_executable(submap_writer_test tests/research_writer.cpp)
    target_compile_options(submap_writer_test PRIVATE -UNDEBUG)
    target_link_libraries(submap_writer_test ellipselio_mapping)
    target_include_directories(submap_writer_test PRIVATE "${S3E_RESEARCH_SOURCE}/include")
    ament_target_dependencies(submap_writer_test rclcpp)
    add_test(NAME bounded_submap_writer COMMAND submap_writer_test /usr/bin/python3
      "${CMAKE_CURRENT_SOURCE_DIR}/tests/slow_writer.py" "${CMAKE_CURRENT_BINARY_DIR}/writer-test.json")
    add_executable(area_maps_test tests/area_maps.cpp)
    target_compile_options(area_maps_test PRIVATE -UNDEBUG)
    target_compile_definitions(area_maps_test PRIVATE ELLIPSELIO_RESEARCH_EXPORT)
    target_include_directories(area_maps_test PRIVATE "${S3E_RESEARCH_SOURCE}/include")
    target_link_libraries(area_maps_test ellipselio_mapping)
    ament_target_dependencies(area_maps_test ${dependencies})
    add_test(NAME accumulated_area_maps COMMAND area_maps_test)
  endif()
endif()
