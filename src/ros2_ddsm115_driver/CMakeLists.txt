cmake_minimum_required(VERSION 3.8)
project(ros2_ddsm115_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

#Add message files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Float64Array.msg"
  DEPENDENCIES std_msgs
)

#Core Library
 add_library(ddsm115_communicator
   src/ros2_ddsm115_driver/ddsm115_communicator.cpp
 )
 target_include_directories(ddsm115_communicator PUBLIC
   $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
   $<INSTALL_INTERFACE:include>
 )
 # Set required features (e.g., C++17)
 target_compile_features(ddsm115_communicator PUBLIC cxx_std_17)

add_executable(generic_ddsm115_driver src/generic_ddsm115_driver.cpp)
target_include_directories(generic_ddsm115_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(generic_ddsm115_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(generic_ddsm115_driver ddsm115_communicator)
ament_target_dependencies(generic_ddsm115_driver rclcpp std_msgs)

add_executable(sw_ddsm115_driver src/sw_ddsm115_driver.cpp)
target_include_directories(sw_ddsm115_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(sw_ddsm115_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(sw_ddsm115_driver ddsm115_communicator)
ament_target_dependencies(sw_ddsm115_driver rclcpp std_msgs)
target_link_libraries(sw_ddsm115_driver ${PROJECT_NAME}__rosidl_typesupport_cpp)
add_dependencies(sw_ddsm115_driver ${PROJECT_NAME}__rosidl_generator_cpp)

install(TARGETS generic_ddsm115_driver sw_ddsm115_driver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
 DESTINATION share/${PROJECT_NAME}/config
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

# Export package
ament_export_dependencies(rosidl_default_runtime)
ament_package()
