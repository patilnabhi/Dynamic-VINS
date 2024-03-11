cmake_minimum_required(VERSION 3.10)
project(vins_estimator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC -lglog)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

## Find system dependencies
find_package(Ceres REQUIRED)

## vins_lib
ament_auto_add_library(vins_lib SHARED
  src/utility/parameters.cpp
  src/estimator/estimator.cpp
  src/feature_manager/feature_manager.cpp
#   src/factor/pose_local_parameterization.cpp
  src/factor/projection_factor.cpp
  src/factor/projection_td_factor.cpp
#   src/factor/marginalization_factor.cpp
  src/utility/utility.cpp
#   src/utility/visualization.cpp
#   src/utility/CameraPoseVisualization.cpp
#   src/initial/solve_5pts.cpp
#   src/initial/initial_aligment.cpp
#   src/initial/initial_sfm.cpp
#   src/initial/initial_ex_rotation.cpp
#   src/feature_tracker/feature_tracker.cpp
)
target_link_libraries(vins_lib
  ${CERES_LIBRARIES}
)

## estimaor_node
ament_auto_add_library(${PROJECT}_node SHARED
  src/estimator_node.cpp
)
rclcpp_components_register_node(${PROJECT}_node
  PLUGIN "estimator_node_ns::EstimatorNode"
  EXECUTABLE estimator_node
)
target_link_libraries(${PROJECT}_node
  vins_lib
)

# install launch files
# install(DIRECTORY 
#     launch 
#     config
#     DESTINATION share/${PROJECT_NAME})

ament_auto_package()