cmake_minimum_required(VERSION 3.16)
project(uav_autonomy)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
foreach(dep rclcpp sensor_msgs geometry_msgs nav_msgs std_msgs tf2_ros tf2_eigen px4_msgs uav_common_msg)
  find_package(${dep} REQUIRED)
endforeach()
find_package(uav_ganyu_control REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)
add_executable(autonomy_node src/autonomy_node.cpp)
target_include_directories(autonomy_node PRIVATE include ${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(autonomy_node Eigen3::Eigen ${OCTOMAP_LIBRARIES})
target_compile_options(autonomy_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(autonomy_node rclcpp sensor_msgs geometry_msgs nav_msgs std_msgs tf2_ros tf2_eigen uav_common_msg uav_ganyu_control)
add_executable(map_query src/map_query.cpp)
target_link_libraries(map_query ${OCTOMAP_LIBRARIES})
install(TARGETS autonomy_node map_query DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/px4_state_bridge.py DESTINATION lib/${PROJECT_NAME} RENAME px4_state_bridge)
install(PROGRAMS scripts/cloud_proxy.py DESTINATION lib/${PROJECT_NAME} RENAME cloud_proxy)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  enable_testing()
  add_executable(interface_check test/interface_check.cpp)
  target_include_directories(interface_check PRIVATE include)
  target_link_libraries(interface_check Eigen3::Eigen ${OCTOMAP_LIBRARIES})
  ament_target_dependencies(interface_check sensor_msgs uav_common_msg uav_ganyu_control)
  add_test(NAME interface_check COMMAND interface_check)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME geodesy_check COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/geodesy_check.py)
  add_test(NAME backend_environment_check COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/backend_environment_check.py)
  add_test(NAME process_cleanup_check COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/process_cleanup_check.py)
  add_test(NAME waypoint_context_check COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/waypoint_context_check.py)
  add_test(NAME reference_liveness_check COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/reference_liveness_check.py $<TARGET_FILE:autonomy_node>)
  add_test(NAME reference_liveness_rear_check COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/reference_liveness_check.py $<TARGET_FILE:autonomy_node> --rear)
  add_test(NAME reference_takeoff_gap_check COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/reference_liveness_check.py $<TARGET_FILE:autonomy_node> --takeoff-gap)
  add_test(NAME reference_viewing_move_check COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/reference_liveness_check.py $<TARGET_FILE:autonomy_node> --viewing-move)
  add_test(NAME reference_indoor_route_check COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/reference_liveness_check.py $<TARGET_FILE:autonomy_node> --indoor-route)
  set_tests_properties(reference_indoor_route_check PROPERTIES TIMEOUT 65)
  add_test(NAME reference_initial_alignment_check COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/reference_liveness_check.py $<TARGET_FILE:autonomy_node> --initial-alignment)
  set_tests_properties(reference_initial_alignment_check PROPERTIES TIMEOUT 45)
  add_test(NAME reference_native_horizon_check COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/reference_liveness_check.py $<TARGET_FILE:autonomy_node> --tail-envelope)
  set_tests_properties(reference_native_horizon_check PROPERTIES TIMEOUT 90)
  add_test(NAME reference_supported_hold_check COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/reference_liveness_check.py $<TARGET_FILE:autonomy_node> --viewing-offset)
  set_tests_properties(reference_supported_hold_check PROPERTIES TIMEOUT 65)
  add_test(NAME reference_planner_stall_check COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/reference_liveness_check.py $<TARGET_FILE:autonomy_node> --planner-stall)
  set_tests_properties(reference_planner_stall_check PROPERTIES TIMEOUT 75)
  add_test(NAME reference_indoor_viewing_check COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/reference_liveness_check.py $<TARGET_FILE:autonomy_node> --indoor-viewing)
  set_tests_properties(reference_indoor_viewing_check PROPERTIES TIMEOUT 75)
  set_tests_properties(reference_liveness_check reference_liveness_rear_check reference_takeoff_gap_check reference_viewing_move_check PROPERTIES TIMEOUT 45)
  # The existing takeoff gap case also checks bounded no-lift/partial-lift
  # failures and cancellation of an unarmed delayed request.
  set_tests_properties(reference_takeoff_gap_check PROPERTIES TIMEOUT 180)
  # Viewing recovery also completes the following scan, retries the original
  # mission goal, and verifies that the old unsafe candidate remains rejected.
  set_tests_properties(reference_viewing_move_check PROPERTIES TIMEOUT 150)
  add_test(NAME reference_clock_rollback_check COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/reference_liveness_check.py $<TARGET_FILE:autonomy_node> --clock-reset)
  set_tests_properties(reference_clock_rollback_check PROPERTIES TIMEOUT 45)
  add_test(NAME reference_patrol_viewing_check COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/reference_liveness_check.py $<TARGET_FILE:autonomy_node> --patrol-viewing)
  set_tests_properties(reference_patrol_viewing_check PROPERTIES TIMEOUT 150)
endif()
ament_package()
