cmake_minimum_required(VERSION 2.8.3)
project(radar_code)

include_directories(
 ../include
 ${catkin_INCLUDE_DIRS}
)

file(GLOB req_objs /home/pi/catkin_ws_mqp/src/firefighting_robot_MQP/scripts/radar_code/acc_radar_code_modified/rpi_sparkfun/lib/req_objs/*)

add_library(acconeer STATIC IMPORTED)
add_library(customer STATIC IMPORTED)

set_target_properties(acconeer PROPERTIES IMPORTED_LOCATION /home/pi/catkin_ws_mqp/src/firefighting_robot_MQP/scripts/radar_code/acc_radar_code_modified/rpi_sparkfun/lib/libacconeer.a)
set_target_properties(customer PROPERTIES IMPORTED_LOCATION /home/pi/catkin_ws_mqp/src/firefighting_robot_MQP/scripts/radar_code/acc_radar_code_modified/rpi_sparkfun/lib/libcustomer.a)


add_library(radar_processing_helpers radar_processing_helpers.c)
add_executable(distance_detector_ros distance_detector_ros.c)
target_link_libraries(radar_processing_helpers acconeer customer ${req_objs} -lm -lpthread)
target_link_libraries(distance_detector_ros radar_processing_helpers)
