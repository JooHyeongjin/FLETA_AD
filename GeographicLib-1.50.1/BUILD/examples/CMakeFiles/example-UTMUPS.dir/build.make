# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/a/catkin_ws/src/GeographicLib-1.50.1

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/a/catkin_ws/src/GeographicLib-1.50.1/BUILD

# Include any dependencies generated for this target.
include examples/CMakeFiles/example-UTMUPS.dir/depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/example-UTMUPS.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/example-UTMUPS.dir/flags.make

examples/CMakeFiles/example-UTMUPS.dir/example-UTMUPS.cpp.o: examples/CMakeFiles/example-UTMUPS.dir/flags.make
examples/CMakeFiles/example-UTMUPS.dir/example-UTMUPS.cpp.o: ../examples/example-UTMUPS.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/a/catkin_ws/src/GeographicLib-1.50.1/BUILD/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/example-UTMUPS.dir/example-UTMUPS.cpp.o"
	cd /home/a/catkin_ws/src/GeographicLib-1.50.1/BUILD/examples && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/example-UTMUPS.dir/example-UTMUPS.cpp.o -c /home/a/catkin_ws/src/GeographicLib-1.50.1/examples/example-UTMUPS.cpp

examples/CMakeFiles/example-UTMUPS.dir/example-UTMUPS.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/example-UTMUPS.dir/example-UTMUPS.cpp.i"
	cd /home/a/catkin_ws/src/GeographicLib-1.50.1/BUILD/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/a/catkin_ws/src/GeographicLib-1.50.1/examples/example-UTMUPS.cpp > CMakeFiles/example-UTMUPS.dir/example-UTMUPS.cpp.i

examples/CMakeFiles/example-UTMUPS.dir/example-UTMUPS.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/example-UTMUPS.dir/example-UTMUPS.cpp.s"
	cd /home/a/catkin_ws/src/GeographicLib-1.50.1/BUILD/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/a/catkin_ws/src/GeographicLib-1.50.1/examples/example-UTMUPS.cpp -o CMakeFiles/example-UTMUPS.dir/example-UTMUPS.cpp.s

examples/CMakeFiles/example-UTMUPS.dir/example-UTMUPS.cpp.o.requires:

.PHONY : examples/CMakeFiles/example-UTMUPS.dir/example-UTMUPS.cpp.o.requires

examples/CMakeFiles/example-UTMUPS.dir/example-UTMUPS.cpp.o.provides: examples/CMakeFiles/example-UTMUPS.dir/example-UTMUPS.cpp.o.requires
	$(MAKE) -f examples/CMakeFiles/example-UTMUPS.dir/build.make examples/CMakeFiles/example-UTMUPS.dir/example-UTMUPS.cpp.o.provides.build
.PHONY : examples/CMakeFiles/example-UTMUPS.dir/example-UTMUPS.cpp.o.provides

examples/CMakeFiles/example-UTMUPS.dir/example-UTMUPS.cpp.o.provides.build: examples/CMakeFiles/example-UTMUPS.dir/example-UTMUPS.cpp.o


# Object files for target example-UTMUPS
example__UTMUPS_OBJECTS = \
"CMakeFiles/example-UTMUPS.dir/example-UTMUPS.cpp.o"

# External object files for target example-UTMUPS
example__UTMUPS_EXTERNAL_OBJECTS =

examples/example-UTMUPS: examples/CMakeFiles/example-UTMUPS.dir/example-UTMUPS.cpp.o
examples/example-UTMUPS: examples/CMakeFiles/example-UTMUPS.dir/build.make
examples/example-UTMUPS: src/libGeographic.so.19.0.1
examples/example-UTMUPS: examples/CMakeFiles/example-UTMUPS.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/a/catkin_ws/src/GeographicLib-1.50.1/BUILD/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable example-UTMUPS"
	cd /home/a/catkin_ws/src/GeographicLib-1.50.1/BUILD/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/example-UTMUPS.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/example-UTMUPS.dir/build: examples/example-UTMUPS

.PHONY : examples/CMakeFiles/example-UTMUPS.dir/build

examples/CMakeFiles/example-UTMUPS.dir/requires: examples/CMakeFiles/example-UTMUPS.dir/example-UTMUPS.cpp.o.requires

.PHONY : examples/CMakeFiles/example-UTMUPS.dir/requires

examples/CMakeFiles/example-UTMUPS.dir/clean:
	cd /home/a/catkin_ws/src/GeographicLib-1.50.1/BUILD/examples && $(CMAKE_COMMAND) -P CMakeFiles/example-UTMUPS.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/example-UTMUPS.dir/clean

examples/CMakeFiles/example-UTMUPS.dir/depend:
	cd /home/a/catkin_ws/src/GeographicLib-1.50.1/BUILD && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/a/catkin_ws/src/GeographicLib-1.50.1 /home/a/catkin_ws/src/GeographicLib-1.50.1/examples /home/a/catkin_ws/src/GeographicLib-1.50.1/BUILD /home/a/catkin_ws/src/GeographicLib-1.50.1/BUILD/examples /home/a/catkin_ws/src/GeographicLib-1.50.1/BUILD/examples/CMakeFiles/example-UTMUPS.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/example-UTMUPS.dir/depend

