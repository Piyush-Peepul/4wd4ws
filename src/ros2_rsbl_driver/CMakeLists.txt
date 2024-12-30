cmake_minimum_required(VERSION 3.8)
project(ros2_rsbl_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)



# SCServo sources
set(SCSERVO_SOURCES
  include/SCServo_Linux/SCS.cpp
  include/SCServo_Linux/SCSCL.cpp
  include/SCServo_Linux/SCSerial.cpp
  include/SCServo_Linux/SMS_STS.cpp
  include/SCServo_Linux/SMSBL.cpp
  include/SCServo_Linux/SMSCL.cpp
)



# Wrapper library
add_library(generic_rsbl_driver_lib
  src/ros2_rsbl_driver/generic_rsbl_driver.cpp
  ${SCSERVO_SOURCES}  # Include SCServo source files
)
# Include directories for wrapper
target_include_directories(generic_rsbl_driver_lib PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PROJECT_SOURCE_DIR}/SCServo_Linux  # Include SCServo headers
)
# Set required features (e.g., C++17)
target_compile_features(generic_rsbl_driver_lib PUBLIC cxx_std_17)
# Link dependencies for wrapper
ament_target_dependencies(generic_rsbl_driver_lib rclcpp std_msgs)



# Main executable
add_executable(generic_rsbl_driver src/generic_rsbl_driver.cpp)
# Include directories for the executable
target_include_directories(generic_rsbl_driver PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# Set compile features for the executable
target_compile_features(generic_rsbl_driver PUBLIC c_std_99 cxx_std_17)
# Link the wrapper library to the executable
target_link_libraries(generic_rsbl_driver generic_rsbl_driver_lib)
# Link dependencies for the executable
ament_target_dependencies(generic_rsbl_driver rclcpp std_msgs)



add_executable(sw_rsbl_driver src/sw_rsbl_driver.cpp)
# Include directories for the executable
target_include_directories(sw_rsbl_driver PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# Set compile features for the executable
target_compile_features(sw_rsbl_driver PUBLIC c_std_99 cxx_std_17)
# Link the wrapper library to the executable
target_link_libraries(sw_rsbl_driver generic_rsbl_driver_lib)
# Link dependencies for the executable
ament_target_dependencies(sw_rsbl_driver rclcpp std_msgs)



add_executable(calibrate_offset src/calibrate_offset.cpp)
# Include directories for the executable
target_include_directories(calibrate_offset PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# Set compile features for the executable
target_compile_features(calibrate_offset PUBLIC c_std_99 cxx_std_17)
# Link the wrapper library to the executable
target_link_libraries(calibrate_offset generic_rsbl_driver_lib)
# Link dependencies for the executable
ament_target_dependencies(calibrate_offset rclcpp std_msgs)



#add_executable(test_servos src/test_servos.cpp)
#ament_target_dependencies(test_servos rclcpp std_msgs)



# Install targets
install(TARGETS generic_rsbl_driver generic_rsbl_driver_lib calibrate_offset sw_rsbl_driver #test_servos
	DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
 DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
 DESTINATION share/${PROJECT_NAME}/launch
)

# Enable testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip linter checks
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
