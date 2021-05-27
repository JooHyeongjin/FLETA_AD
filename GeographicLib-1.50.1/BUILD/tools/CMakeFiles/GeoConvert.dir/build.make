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
include tools/CMakeFiles/GeoConvert.dir/depend.make

# Include the progress variables for this target.
include tools/CMakeFiles/GeoConvert.dir/progress.make

# Include the compile flags for this target's objects.
include tools/CMakeFiles/GeoConvert.dir/flags.make

tools/CMakeFiles/GeoConvert.dir/GeoConvert.cpp.o: tools/CMakeFiles/GeoConvert.dir/flags.make
tools/CMakeFiles/GeoConvert.dir/GeoConvert.cpp.o: ../tools/GeoConvert.cpp
tools/CMakeFiles/GeoConvert.dir/GeoConvert.cpp.o: man/GeoConvert.usage
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/a/catkin_ws/src/GeographicLib-1.50.1/BUILD/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object tools/CMakeFiles/GeoConvert.dir/GeoConvert.cpp.o"
	cd /home/a/catkin_ws/src/GeographicLib-1.50.1/BUILD/tools && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/GeoConvert.dir/GeoConvert.cpp.o -c /home/a/catkin_ws/src/GeographicLib-1.50.1/tools/GeoConvert.cpp

tools/CMakeFiles/GeoConvert.dir/GeoConvert.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/GeoConvert.dir/GeoConvert.cpp.i"
	cd /home/a/catkin_ws/src/GeographicLib-1.50.1/BUILD/tools && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/a/catkin_ws/src/GeographicLib-1.50.1/tools/GeoConvert.cpp > CMakeFiles/GeoConvert.dir/GeoConvert.cpp.i

tools/CMakeFiles/GeoConvert.dir/GeoConvert.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/GeoConvert.dir/GeoConvert.cpp.s"
	cd /home/a/catkin_ws/src/GeographicLib-1.50.1/BUILD/tools && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/a/catkin_ws/src/GeographicLib-1.50.1/tools/GeoConvert.cpp -o CMakeFiles/GeoConvert.dir/GeoConvert.cpp.s

tools/CMakeFiles/GeoConvert.dir/GeoConvert.cpp.o.requires:

.PHONY : tools/CMakeFiles/GeoConvert.dir/GeoConvert.cpp.o.requires

tools/CMakeFiles/GeoConvert.dir/GeoConvert.cpp.o.provides: tools/CMakeFiles/GeoConvert.dir/GeoConvert.cpp.o.requires
	$(MAKE) -f tools/CMakeFiles/GeoConvert.dir/build.make tools/CMakeFiles/GeoConvert.dir/GeoConvert.cpp.o.provides.build
.PHONY : tools/CMakeFiles/GeoConvert.dir/GeoConvert.cpp.o.provides

tools/CMakeFiles/GeoConvert.dir/GeoConvert.cpp.o.provides.build: tools/CMakeFiles/GeoConvert.dir/GeoConvert.cpp.o


# Object files for target GeoConvert
GeoConvert_OBJECTS = \
"CMakeFiles/GeoConvert.dir/GeoConvert.cpp.o"

# External object files for target GeoConvert
GeoConvert_EXTERNAL_OBJECTS =

tools/GeoConvert: tools/CMakeFiles/GeoConvert.dir/GeoConvert.cpp.o
tools/GeoConvert: tools/CMakeFiles/GeoConvert.dir/build.make
tools/GeoConvert: src/libGeographic.so.19.0.1
tools/GeoConvert: tools/CMakeFiles/GeoConvert.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/a/catkin_ws/src/GeographicLib-1.50.1/BUILD/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable GeoConvert"
	cd /home/a/catkin_ws/src/GeographicLib-1.50.1/BUILD/tools && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/GeoConvert.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
tools/CMakeFiles/GeoConvert.dir/build: tools/GeoConvert

.PHONY : tools/CMakeFiles/GeoConvert.dir/build

tools/CMakeFiles/GeoConvert.dir/requires: tools/CMakeFiles/GeoConvert.dir/GeoConvert.cpp.o.requires

.PHONY : tools/CMakeFiles/GeoConvert.dir/requires

tools/CMakeFiles/GeoConvert.dir/clean:
	cd /home/a/catkin_ws/src/GeographicLib-1.50.1/BUILD/tools && $(CMAKE_COMMAND) -P CMakeFiles/GeoConvert.dir/cmake_clean.cmake
.PHONY : tools/CMakeFiles/GeoConvert.dir/clean

tools/CMakeFiles/GeoConvert.dir/depend:
	cd /home/a/catkin_ws/src/GeographicLib-1.50.1/BUILD && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/a/catkin_ws/src/GeographicLib-1.50.1 /home/a/catkin_ws/src/GeographicLib-1.50.1/tools /home/a/catkin_ws/src/GeographicLib-1.50.1/BUILD /home/a/catkin_ws/src/GeographicLib-1.50.1/BUILD/tools /home/a/catkin_ws/src/GeographicLib-1.50.1/BUILD/tools/CMakeFiles/GeoConvert.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : tools/CMakeFiles/GeoConvert.dir/depend

