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
find_package(rclcpp_components REQUIRED)
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

# Component libraries
add_library(source_publisher_component SHARED source_publisher.cpp)
ament_target_dependencies(source_publisher_component rclcpp rclcpp_components custom_msg)
rclcpp_components_register_nodes(source_publisher_component "SourcePublisher")

add_library(subscribe_publisher_component SHARED subscribe_publisher.cpp)
ament_target_dependencies(subscribe_publisher_component rclcpp rclcpp_components custom_msg)
target_link_libraries(subscribe_publisher_component ${PMU_ANALYZER_LIB})
rclcpp_components_register_nodes(subscribe_publisher_component "SubscribePublisher")
set_target_properties(subscribe_publisher_component PROPERTIES
  BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/../../external/pmu_analyzer/lib"
  INSTALL_RPATH "$ORIGIN/../lib:$ORIGIN/../../custom_msg/lib:${CMAKE_INSTALL_PREFIX}/lib")

add_library(sync_subscriber_component SHARED sync_subscriber.cpp)
ament_target_dependencies(sync_subscriber_component rclcpp rclcpp_components custom_msg message_filters)
target_link_libraries(sync_subscriber_component ${PMU_ANALYZER_LIB})
rclcpp_components_register_nodes(sync_subscriber_component "SyncSubscriber<1>;SyncSubscriber<2>;SyncSubscriber<3>;SyncSubscriber<4>;SyncSubscriber<5>;SyncSubscriber<6>;SyncSubscriber<7>;SyncSubscriber<8>")
set_target_properties(sync_subscriber_component PROPERTIES
  BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/../../external/pmu_analyzer/lib"
  INSTALL_RPATH "$ORIGIN/../lib:$ORIGIN/../../custom_msg/lib:${CMAKE_INSTALL_PREFIX}/lib")


# Install PMU analyzer library
install(FILES ${PMU_ANALYZER_LIB}
  DESTINATION lib)

install(TARGETS
  source_publisher_component
  subscribe_publisher_component
  sync_subscriber_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

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
