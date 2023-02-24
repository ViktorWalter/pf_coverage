cmake_minimum_required(VERSION 3.5)
project(pf_coverage)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(turtlebot3 REQUIRED)
find_package(turtlebot3_msgs REQUIRED)
find_package(particle_filter REQUIRED)
find_package(GaussianMixtureModel REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(SFML REQUIRED COMPONENTS graphics window system)
# find_package(std_srvs REQUIRED)

################################################################################
# Build
################################################################################

add_library(${PROJECT_NAME}_lib
  "src/pf_coverage.cpp"
  "src/vision_emulator.cpp"
  "src/coverage_distributed_neighmaster.cpp"
  "src/coverage_distributed_singlenode.cpp"
)

set(dependencies
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "sensor_msgs"
  "tf2"
  "tf2_ros"
  "turtlebot3"
  "turtlebot3_msgs"
  "particle_filter"
  "GaussianMixtureModel"
  "visualization_msgs"
)


set(SFML_libraries
  "sfml-graphics"
  "sfml-window"
  "sfml-system"
)

target_link_libraries(${PROJECT_NAME}_lib)
ament_target_dependencies(${PROJECT_NAME}_lib ${dependencies})

add_executable(pf_coverage src/pf_coverage.cpp )
target_link_libraries(pf_coverage ${SFML_libraries})
ament_target_dependencies(pf_coverage ${dependencies})

add_executable(vision_emulator src/vision_emulator.cpp )
ament_target_dependencies(vision_emulator ${dependencies})

add_executable(coverage_distributed_neighmaster src/coverage_distributed_neighmaster.cpp )
target_link_libraries(coverage_distributed_neighmaster ${SFML_libraries})
ament_target_dependencies(coverage_distributed_neighmaster ${dependencies})

add_executable(coverage_distributed_singlenode src/coverage_distributed_singlenode.cpp )
target_link_libraries(coverage_distributed_singlenode ${SFML_libraries})
ament_target_dependencies(coverage_distributed_singlenode ${dependencies})

add_executable(turtlebot_supervisor src/turtlebot_supervisor.cpp )
# target_link_libraries(turtlebot_supervisor ${SFML_libraries})
ament_target_dependencies(turtlebot_supervisor ${dependencies})

add_executable(turtlepf_coverage src/turtlepf_coverage.cpp )
target_link_libraries(turtlepf_coverage ${SFML_libraries})
ament_target_dependencies(turtlepf_coverage ${dependencies})

add_executable(turtlebot_fakepose src/turtlebot_fakepose.cpp )
ament_target_dependencies(turtlebot_fakepose ${dependencies})

################################################################################
# Install
################################################################################

install(TARGETS
  pf_coverage
  vision_emulator
  coverage_distributed_neighmaster
  coverage_distributed_singlenode
  turtlebot_supervisor
  turtlepf_coverage
  turtlebot_fakepose
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include/pf_coverage
  DESTINATION include
)

# let the compiler search for headers in include folder
include_directories(include)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(visualization_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclpy)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(tf2)
ament_export_dependencies(turtlebot3)
ament_export_dependencies(particle_filter)
ament_export_dependencies(GaussianMixtureModel)

ament_package()
