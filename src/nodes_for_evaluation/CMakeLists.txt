cmake_minimum_required(VERSION 3.8)
project(nodes_for_evaluation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set RPATH options
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_msg REQUIRED)
find_package(message_filters REQUIRED)

# Add PMU analyzer - use prebuilt library
set(PMU_ANALYZER_LIB ${CMAKE_CURRENT_SOURCE_DIR}/../../external/pmu_analyzer/lib/libpmuanalyzer.so)
if(EXISTS ${PMU_ANALYZER_LIB})
    message(STATUS "Using prebuilt PMU analyzer library: ${PMU_ANALYZER_LIB}")
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../external/pmu_analyzer/include)
else()
    message(FATAL_ERROR "PMU analyzer library not found at ${PMU_ANALYZER_LIB}")
endif()

# subscribe_publisher
add_executable(subscribe_publisher subscribe_publisher.cpp)
ament_target_dependencies(subscribe_publisher rclcpp custom_msg)
target_link_libraries(subscribe_publisher ${PMU_ANALYZER_LIB})
set_target_properties(subscribe_publisher PROPERTIES
  BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/../../external/pmu_analyzer/lib"
  INSTALL_RPATH "$ORIGIN/../lib:$ORIGIN/../../custom_msg/lib:${CMAKE_INSTALL_PREFIX}/lib")

# sync_subscriber
add_executable(sync_subscriber sync_subscriber.cpp)
ament_target_dependencies(sync_subscriber rclcpp custom_msg message_filters)
target_link_libraries(sync_subscriber ${PMU_ANALYZER_LIB})
set_target_properties(sync_subscriber PROPERTIES
  BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/../../external/pmu_analyzer/lib"
  INSTALL_RPATH "$ORIGIN/../lib:$ORIGIN/../../custom_msg/lib:${CMAKE_INSTALL_PREFIX}/lib")

# source_publisher
add_executable(source_publisher source_publisher.cpp)
ament_target_dependencies(source_publisher rclcpp custom_msg)
set_target_properties(source_publisher PROPERTIES
  INSTALL_RPATH "$ORIGIN/../../custom_msg/lib:${CMAKE_INSTALL_PREFIX}/lib")

# Install PMU analyzer library
install(FILES ${PMU_ANALYZER_LIB}
  DESTINATION lib)

install(TARGETS
  subscribe_publisher
  sync_subscriber
  source_publisher
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  ../../launch
  ../../config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
