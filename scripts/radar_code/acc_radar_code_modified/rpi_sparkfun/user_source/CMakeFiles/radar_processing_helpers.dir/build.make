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
include CMakeFiles/radar_processing_helpers.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/radar_processing_helpers.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/radar_processing_helpers.dir/flags.make

CMakeFiles/radar_processing_helpers.dir/radar_processing_helpers.cpp.o: CMakeFiles/radar_processing_helpers.dir/flags.make
CMakeFiles/radar_processing_helpers.dir/radar_processing_helpers.cpp.o: radar_processing_helpers.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/catkin_ws_mqp/src/firefighting_robot_MQP/scripts/radar_code/acc_radar_code_modified/rpi_sparkfun/user_source/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/radar_processing_helpers.dir/radar_processing_helpers.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/radar_processing_helpers.dir/radar_processing_helpers.cpp.o -c /home/pi/catkin_ws_mqp/src/firefighting_robot_MQP/scripts/radar_code/acc_radar_code_modified/rpi_sparkfun/user_source/radar_processing_helpers.cpp

CMakeFiles/radar_processing_helpers.dir/radar_processing_helpers.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/radar_processing_helpers.dir/radar_processing_helpers.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/catkin_ws_mqp/src/firefighting_robot_MQP/scripts/radar_code/acc_radar_code_modified/rpi_sparkfun/user_source/radar_processing_helpers.cpp > CMakeFiles/radar_processing_helpers.dir/radar_processing_helpers.cpp.i

CMakeFiles/radar_processing_helpers.dir/radar_processing_helpers.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/radar_processing_helpers.dir/radar_processing_helpers.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/catkin_ws_mqp/src/firefighting_robot_MQP/scripts/radar_code/acc_radar_code_modified/rpi_sparkfun/user_source/radar_processing_helpers.cpp -o CMakeFiles/radar_processing_helpers.dir/radar_processing_helpers.cpp.s

# Object files for target radar_processing_helpers
radar_processing_helpers_OBJECTS = \
"CMakeFiles/radar_processing_helpers.dir/radar_processing_helpers.cpp.o"

# External object files for target radar_processing_helpers
radar_processing_helpers_EXTERNAL_OBJECTS =

libradar_processing_helpers.a: CMakeFiles/radar_processing_helpers.dir/radar_processing_helpers.cpp.o
libradar_processing_helpers.a: CMakeFiles/radar_processing_helpers.dir/build.make
libradar_processing_helpers.a: CMakeFiles/radar_processing_helpers.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/catkin_ws_mqp/src/firefighting_robot_MQP/scripts/radar_code/acc_radar_code_modified/rpi_sparkfun/user_source/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libradar_processing_helpers.a"
	$(CMAKE_COMMAND) -P CMakeFiles/radar_processing_helpers.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/radar_processing_helpers.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/radar_processing_helpers.dir/build: libradar_processing_helpers.a

.PHONY : CMakeFiles/radar_processing_helpers.dir/build

CMakeFiles/radar_processing_helpers.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/radar_processing_helpers.dir/cmake_clean.cmake
.PHONY : CMakeFiles/radar_processing_helpers.dir/clean

CMakeFiles/radar_processing_helpers.dir/depend:
	cd /home/pi/catkin_ws_mqp/src/firefighting_robot_MQP/scripts/radar_code/acc_radar_code_modified/rpi_sparkfun/user_source && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/catkin_ws_mqp/src/firefighting_robot_MQP/scripts/radar_code/acc_radar_code_modified/rpi_sparkfun/user_source /home/pi/catkin_ws_mqp/src/firefighting_robot_MQP/scripts/radar_code/acc_radar_code_modified/rpi_sparkfun/user_source /home/pi/catkin_ws_mqp/src/firefighting_robot_MQP/scripts/radar_code/acc_radar_code_modified/rpi_sparkfun/user_source /home/pi/catkin_ws_mqp/src/firefighting_robot_MQP/scripts/radar_code/acc_radar_code_modified/rpi_sparkfun/user_source /home/pi/catkin_ws_mqp/src/firefighting_robot_MQP/scripts/radar_code/acc_radar_code_modified/rpi_sparkfun/user_source/CMakeFiles/radar_processing_helpers.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/radar_processing_helpers.dir/depend

