# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.4

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
CMAKE_SOURCE_DIR = /users/visics/kkelchte/Gazebo/Modelplugin

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /users/visics/kkelchte/Gazebo/Modelplugin/build

# Include any dependencies generated for this target.
include CMakeFiles/camera_move_test.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/camera_move_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/camera_move_test.dir/flags.make

CMakeFiles/camera_move_test.dir/camera_move_test.cc.o: CMakeFiles/camera_move_test.dir/flags.make
CMakeFiles/camera_move_test.dir/camera_move_test.cc.o: ../camera_move_test.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/users/visics/kkelchte/Gazebo/Modelplugin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/camera_move_test.dir/camera_move_test.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/camera_move_test.dir/camera_move_test.cc.o -c /users/visics/kkelchte/Gazebo/Modelplugin/camera_move_test.cc

CMakeFiles/camera_move_test.dir/camera_move_test.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/camera_move_test.dir/camera_move_test.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /users/visics/kkelchte/Gazebo/Modelplugin/camera_move_test.cc > CMakeFiles/camera_move_test.dir/camera_move_test.cc.i

CMakeFiles/camera_move_test.dir/camera_move_test.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/camera_move_test.dir/camera_move_test.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /users/visics/kkelchte/Gazebo/Modelplugin/camera_move_test.cc -o CMakeFiles/camera_move_test.dir/camera_move_test.cc.s

CMakeFiles/camera_move_test.dir/camera_move_test.cc.o.requires:

.PHONY : CMakeFiles/camera_move_test.dir/camera_move_test.cc.o.requires

CMakeFiles/camera_move_test.dir/camera_move_test.cc.o.provides: CMakeFiles/camera_move_test.dir/camera_move_test.cc.o.requires
	$(MAKE) -f CMakeFiles/camera_move_test.dir/build.make CMakeFiles/camera_move_test.dir/camera_move_test.cc.o.provides.build
.PHONY : CMakeFiles/camera_move_test.dir/camera_move_test.cc.o.provides

CMakeFiles/camera_move_test.dir/camera_move_test.cc.o.provides.build: CMakeFiles/camera_move_test.dir/camera_move_test.cc.o


# Object files for target camera_move_test
camera_move_test_OBJECTS = \
"CMakeFiles/camera_move_test.dir/camera_move_test.cc.o"

# External object files for target camera_move_test
camera_move_test_EXTERNAL_OBJECTS =

libcamera_move_test.so: CMakeFiles/camera_move_test.dir/camera_move_test.cc.o
libcamera_move_test.so: CMakeFiles/camera_move_test.dir/build.make
libcamera_move_test.so: CMakeFiles/camera_move_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/users/visics/kkelchte/Gazebo/Modelplugin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libcamera_move_test.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/camera_move_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/camera_move_test.dir/build: libcamera_move_test.so

.PHONY : CMakeFiles/camera_move_test.dir/build

CMakeFiles/camera_move_test.dir/requires: CMakeFiles/camera_move_test.dir/camera_move_test.cc.o.requires

.PHONY : CMakeFiles/camera_move_test.dir/requires

CMakeFiles/camera_move_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/camera_move_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/camera_move_test.dir/clean

CMakeFiles/camera_move_test.dir/depend:
	cd /users/visics/kkelchte/Gazebo/Modelplugin/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /users/visics/kkelchte/Gazebo/Modelplugin /users/visics/kkelchte/Gazebo/Modelplugin /users/visics/kkelchte/Gazebo/Modelplugin/build /users/visics/kkelchte/Gazebo/Modelplugin/build /users/visics/kkelchte/Gazebo/Modelplugin/build/CMakeFiles/camera_move_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/camera_move_test.dir/depend

