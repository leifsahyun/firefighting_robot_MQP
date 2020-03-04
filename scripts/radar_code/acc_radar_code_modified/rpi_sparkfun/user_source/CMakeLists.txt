cmake_minimum_required(VERSION 2.8.3)
project(radar_code)

include_directories(
 ../include
 ${catkin_INCLUDE_DIRS}
)

file(GLOB acc_libs ../lib/*.a)
file(GLOB board ../out/obj/acc_board_rpi_sparkfun.o)

set(OBJS
	../out/obj/acc_driver_spi_linux_spidev.o ../out/obj/acc_driver_gpio_linux_sysfs.o ../out/obj/acc_driver_hal.o ../out/obj/acc_driver_os_linux.o ../out/obj/acc_device_os.o ../out/obj/acc_device_spi.o ../out/obj/acc_device_gpio.o ../out/obj/acc_log.o ../out/obj/acc_log_integration.o ../out/obj/acc_app_integration_linux.o
)

file(GLOB acc_objs ../out/obj/acc_*.o)

set_source_files_properties(
	${acc_objs}
	PROPERTIES
	EXTERNAL_OBJECT true
	GENERATED true
)

#find_library(ACCONEER_LIB NAMES acconeer HINTS "../lib/")
#find_library(CUSTOMER_LIB NAMES customer HINTS "../out/lib")
#message(STATUS "ACCONEER_LIBS = [" ${ACCONEER_LIBS} "]")
#message(STATUS "CUSTOMER_LIBS = [" ${CUSTOMER_LIBS} "]")

add_library(acconeer STATIC IMPORTED)
add_library(customer STATIC IMPORTED)
add_library(board STATIC IMPORTED)
set_target_properties(acconeer PROPERTIES IMPORTED_LOCATION ../lib/libacconeer.a)
set_target_properties(customer PROPERTIES IMPORTED_LOCATION ../lib/libcustomer.a)
set_target_properties(board PROPERTIES IMPORTED_LOCATION ../lib/acc_board_rpi_sparkfun.o)

add_library(radar_processing_helpers radar_processing_helpers.cpp)
add_executable(distance_detector_ros distance_detector_ros.cpp)
target_link_libraries(radar_processing_helpers ${acc_libs})
target_link_libraries(distance_detector_ros radar_processing_helpers ${acc_objs})
