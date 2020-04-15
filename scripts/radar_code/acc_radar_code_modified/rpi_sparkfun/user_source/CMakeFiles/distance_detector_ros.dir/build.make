# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.13

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/pi/catkin_ws_mqp/src/firefighting_robot_MQP/scripts/radar_code/acc_radar_code_modified/rpi_sparkfun/user_source

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/catkin_ws_mqp/src/firefighting_robot_MQP/scripts/radar_code/acc_radar_code_modified/rpi_sparkfun/user_source

# Include any dependencies generated for this target.
include CMakeFiles/distance_detector_ros.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/distance_detector_ros.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/distance_detector_ros.dir/flags.make

CMakeFiles/distance_detector_ros.dir/distance_detector_ros.c.o: CMakeFiles/distance_detector_ros.dir/flags.make
CMakeFiles/distance_detector_ros.dir/distance_detector_ros.c.o: distance_detector_ros.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/catkin_ws_mqp/src/firefighting_robot_MQP/scripts/radar_code/acc_radar_code_modified/rpi_sparkfun/user_source/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/distance_detector_ros.dir/distance_detector_ros.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/distance_detector_ros.dir/distance_detector_ros.c.o   -c /home/pi/catkin_ws_mqp/src/firefighting_robot_MQP/scripts/radar_code/acc_radar_code_modified/rpi_sparkfun/user_source/distance_detector_ros.c

CMakeFiles/distance_detector_ros.dir/distance_detector_ros.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/distance_detector_ros.dir/distance_detector_ros.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/catkin_ws_mqp/src/firefighting_robot_MQP/scripts/radar_code/acc_radar_code_modified/rpi_sparkfun/user_source/distance_detector_ros.c > CMakeFiles/distance_detector_ros.dir/distance_detector_ros.c.i

CMakeFiles/distance_detector_ros.dir/distance_detector_ros.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/distance_detector_ros.dir/distance_detector_ros.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/catkin_ws_mqp/src/firefighting_robot_MQP/scripts/radar_code/acc_radar_code_modified/rpi_sparkfun/user_source/distance_detector_ros.c -o CMakeFiles/distance_detector_ros.dir/distance_detector_ros.c.s

# Object files for target distance_detector_ros
distance_detector_ros_OBJECTS = \
"CMakeFiles/distance_detector_ros.dir/distance_detector_ros.c.o"

# External object files for target distance_detector_ros
distance_detector_ros_EXTERNAL_OBJECTS =

distance_detector_ros: CMakeFiles/distance_detector_ros.dir/distance_detector_ros.c.o
distance_detector_ros: CMakeFiles/distance_detector_ros.dir/build.make
distance_detector_ros: libradar_processing_helpers.a
distance_detector_ros: /home/pi/catkin_ws_mqp/src/firefighting_robot_MQP/scripts/radar_code/acc_radar_code_modified/rpi_sparkfun/lib/libacconeer.a
distance_detector_ros: /home/pi/catkin_ws_mqp/src/firefighting_robot_MQP/scripts/radar_code/acc_radar_code_modified/rpi_sparkfun/lib/libcustomer.a
distance_detector_ros: /home/pi/catkin_ws_mqp/src/firefighting_robot_MQP/scripts/radar_code/acc_radar_code_modified/rpi_sparkfun/lib/req_objs/acc_board_rpi_sparkfun.o
distance_detector_ros: /home/pi/catkin_ws_mqp/src/firefighting_robot_MQP/scripts/radar_code/acc_radar_code_modified/rpi_sparkfun/lib/req_objs/acc_device_gpio.o
distance_detector_ros: /home/pi/catkin_ws_mqp/src/firefighting_robot_MQP/scripts/radar_code/acc_radar_code_modified/rpi_sparkfun/lib/req_objs/acc_driver_gpio_linux_sysfs.o
distance_detector_ros: /home/pi/catkin_ws_mqp/src/firefighting_robot_MQP/scripts/radar_code/acc_radar_code_modified/rpi_sparkfun/lib/req_objs/acc_driver_os_linux.o
distance_detector_ros: /home/pi/catkin_ws_mqp/src/firefighting_robot_MQP/scripts/radar_code/acc_radar_code_modified/rpi_sparkfun/lib/req_objs/acc_driver_spi_linux_spidev.o
distance_detector_ros: CMakeFiles/distance_detector_ros.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/catkin_ws_mqp/src/firefighting_robot_MQP/scripts/radar_code/acc_radar_code_modified/rpi_sparkfun/user_source/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C executable distance_detector_ros"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/distance_detector_ros.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/distance_detector_ros.dir/build: distance_detector_ros

.PHONY : CMakeFiles/distance_detector_ros.dir/build

CMakeFiles/distance_detector_ros.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/distance_detector_ros.dir/cmake_clean.cmake
.PHONY : CMakeFiles/distance_detector_ros.dir/clean

CMakeFiles/distance_detector_ros.dir/depend:
	cd /home/pi/catkin_ws_mqp/src/firefighting_robot_MQP/scripts/radar_code/acc_radar_code_modified/rpi_sparkfun/user_source && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/catkin_ws_mqp/src/firefighting_robot_MQP/scripts/radar_code/acc_radar_code_modified/rpi_sparkfun/user_source /home/pi/catkin_ws_mqp/src/firefighting_robot_MQP/scripts/radar_code/acc_radar_code_modified/rpi_sparkfun/user_source /home/pi/catkin_ws_mqp/src/firefighting_robot_MQP/scripts/radar_code/acc_radar_code_modified/rpi_sparkfun/user_source /home/pi/catkin_ws_mqp/src/firefighting_robot_MQP/scripts/radar_code/acc_radar_code_modified/rpi_sparkfun/user_source /home/pi/catkin_ws_mqp/src/firefighting_robot_MQP/scripts/radar_code/acc_radar_code_modified/rpi_sparkfun/user_source/CMakeFiles/distance_detector_ros.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/distance_detector_ros.dir/depend
