# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.8

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
CMAKE_COMMAND = /home/robocon/Documents/software/clion-2017.2.3/bin/cmake/bin/cmake

# The command to remove a file.
RM = /home/robocon/Documents/software/clion-2017.2.3/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/robocon/workspace/agent-v2.0

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/robocon/workspace/agent-v2.0/cmake-build-debug

# Include any dependencies generated for this target.
include src/trace/CMakeFiles/trace.dir/depend.make

# Include the progress variables for this target.
include src/trace/CMakeFiles/trace.dir/progress.make

# Include the compile flags for this target's objects.
include src/trace/CMakeFiles/trace.dir/flags.make

src/trace/CMakeFiles/trace.dir/TraceController.cpp.o: src/trace/CMakeFiles/trace.dir/flags.make
src/trace/CMakeFiles/trace.dir/TraceController.cpp.o: ../src/trace/TraceController.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robocon/workspace/agent-v2.0/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/trace/CMakeFiles/trace.dir/TraceController.cpp.o"
	cd /home/robocon/workspace/agent-v2.0/cmake-build-debug/src/trace && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/trace.dir/TraceController.cpp.o -c /home/robocon/workspace/agent-v2.0/src/trace/TraceController.cpp

src/trace/CMakeFiles/trace.dir/TraceController.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/trace.dir/TraceController.cpp.i"
	cd /home/robocon/workspace/agent-v2.0/cmake-build-debug/src/trace && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robocon/workspace/agent-v2.0/src/trace/TraceController.cpp > CMakeFiles/trace.dir/TraceController.cpp.i

src/trace/CMakeFiles/trace.dir/TraceController.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/trace.dir/TraceController.cpp.s"
	cd /home/robocon/workspace/agent-v2.0/cmake-build-debug/src/trace && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robocon/workspace/agent-v2.0/src/trace/TraceController.cpp -o CMakeFiles/trace.dir/TraceController.cpp.s

src/trace/CMakeFiles/trace.dir/TraceController.cpp.o.requires:

.PHONY : src/trace/CMakeFiles/trace.dir/TraceController.cpp.o.requires

src/trace/CMakeFiles/trace.dir/TraceController.cpp.o.provides: src/trace/CMakeFiles/trace.dir/TraceController.cpp.o.requires
	$(MAKE) -f src/trace/CMakeFiles/trace.dir/build.make src/trace/CMakeFiles/trace.dir/TraceController.cpp.o.provides.build
.PHONY : src/trace/CMakeFiles/trace.dir/TraceController.cpp.o.provides

src/trace/CMakeFiles/trace.dir/TraceController.cpp.o.provides.build: src/trace/CMakeFiles/trace.dir/TraceController.cpp.o


src/trace/CMakeFiles/trace.dir/BallAssociate.cpp.o: src/trace/CMakeFiles/trace.dir/flags.make
src/trace/CMakeFiles/trace.dir/BallAssociate.cpp.o: ../src/trace/BallAssociate.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robocon/workspace/agent-v2.0/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/trace/CMakeFiles/trace.dir/BallAssociate.cpp.o"
	cd /home/robocon/workspace/agent-v2.0/cmake-build-debug/src/trace && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/trace.dir/BallAssociate.cpp.o -c /home/robocon/workspace/agent-v2.0/src/trace/BallAssociate.cpp

src/trace/CMakeFiles/trace.dir/BallAssociate.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/trace.dir/BallAssociate.cpp.i"
	cd /home/robocon/workspace/agent-v2.0/cmake-build-debug/src/trace && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robocon/workspace/agent-v2.0/src/trace/BallAssociate.cpp > CMakeFiles/trace.dir/BallAssociate.cpp.i

src/trace/CMakeFiles/trace.dir/BallAssociate.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/trace.dir/BallAssociate.cpp.s"
	cd /home/robocon/workspace/agent-v2.0/cmake-build-debug/src/trace && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robocon/workspace/agent-v2.0/src/trace/BallAssociate.cpp -o CMakeFiles/trace.dir/BallAssociate.cpp.s

src/trace/CMakeFiles/trace.dir/BallAssociate.cpp.o.requires:

.PHONY : src/trace/CMakeFiles/trace.dir/BallAssociate.cpp.o.requires

src/trace/CMakeFiles/trace.dir/BallAssociate.cpp.o.provides: src/trace/CMakeFiles/trace.dir/BallAssociate.cpp.o.requires
	$(MAKE) -f src/trace/CMakeFiles/trace.dir/build.make src/trace/CMakeFiles/trace.dir/BallAssociate.cpp.o.provides.build
.PHONY : src/trace/CMakeFiles/trace.dir/BallAssociate.cpp.o.provides

src/trace/CMakeFiles/trace.dir/BallAssociate.cpp.o.provides.build: src/trace/CMakeFiles/trace.dir/BallAssociate.cpp.o


src/trace/CMakeFiles/trace.dir/BallDetector.cpp.o: src/trace/CMakeFiles/trace.dir/flags.make
src/trace/CMakeFiles/trace.dir/BallDetector.cpp.o: ../src/trace/BallDetector.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robocon/workspace/agent-v2.0/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/trace/CMakeFiles/trace.dir/BallDetector.cpp.o"
	cd /home/robocon/workspace/agent-v2.0/cmake-build-debug/src/trace && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/trace.dir/BallDetector.cpp.o -c /home/robocon/workspace/agent-v2.0/src/trace/BallDetector.cpp

src/trace/CMakeFiles/trace.dir/BallDetector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/trace.dir/BallDetector.cpp.i"
	cd /home/robocon/workspace/agent-v2.0/cmake-build-debug/src/trace && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robocon/workspace/agent-v2.0/src/trace/BallDetector.cpp > CMakeFiles/trace.dir/BallDetector.cpp.i

src/trace/CMakeFiles/trace.dir/BallDetector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/trace.dir/BallDetector.cpp.s"
	cd /home/robocon/workspace/agent-v2.0/cmake-build-debug/src/trace && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robocon/workspace/agent-v2.0/src/trace/BallDetector.cpp -o CMakeFiles/trace.dir/BallDetector.cpp.s

src/trace/CMakeFiles/trace.dir/BallDetector.cpp.o.requires:

.PHONY : src/trace/CMakeFiles/trace.dir/BallDetector.cpp.o.requires

src/trace/CMakeFiles/trace.dir/BallDetector.cpp.o.provides: src/trace/CMakeFiles/trace.dir/BallDetector.cpp.o.requires
	$(MAKE) -f src/trace/CMakeFiles/trace.dir/build.make src/trace/CMakeFiles/trace.dir/BallDetector.cpp.o.provides.build
.PHONY : src/trace/CMakeFiles/trace.dir/BallDetector.cpp.o.provides

src/trace/CMakeFiles/trace.dir/BallDetector.cpp.o.provides.build: src/trace/CMakeFiles/trace.dir/BallDetector.cpp.o


src/trace/CMakeFiles/trace.dir/CircleDetector.cpp.o: src/trace/CMakeFiles/trace.dir/flags.make
src/trace/CMakeFiles/trace.dir/CircleDetector.cpp.o: ../src/trace/CircleDetector.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robocon/workspace/agent-v2.0/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object src/trace/CMakeFiles/trace.dir/CircleDetector.cpp.o"
	cd /home/robocon/workspace/agent-v2.0/cmake-build-debug/src/trace && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/trace.dir/CircleDetector.cpp.o -c /home/robocon/workspace/agent-v2.0/src/trace/CircleDetector.cpp

src/trace/CMakeFiles/trace.dir/CircleDetector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/trace.dir/CircleDetector.cpp.i"
	cd /home/robocon/workspace/agent-v2.0/cmake-build-debug/src/trace && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robocon/workspace/agent-v2.0/src/trace/CircleDetector.cpp > CMakeFiles/trace.dir/CircleDetector.cpp.i

src/trace/CMakeFiles/trace.dir/CircleDetector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/trace.dir/CircleDetector.cpp.s"
	cd /home/robocon/workspace/agent-v2.0/cmake-build-debug/src/trace && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robocon/workspace/agent-v2.0/src/trace/CircleDetector.cpp -o CMakeFiles/trace.dir/CircleDetector.cpp.s

src/trace/CMakeFiles/trace.dir/CircleDetector.cpp.o.requires:

.PHONY : src/trace/CMakeFiles/trace.dir/CircleDetector.cpp.o.requires

src/trace/CMakeFiles/trace.dir/CircleDetector.cpp.o.provides: src/trace/CMakeFiles/trace.dir/CircleDetector.cpp.o.requires
	$(MAKE) -f src/trace/CMakeFiles/trace.dir/build.make src/trace/CMakeFiles/trace.dir/CircleDetector.cpp.o.provides.build
.PHONY : src/trace/CMakeFiles/trace.dir/CircleDetector.cpp.o.provides

src/trace/CMakeFiles/trace.dir/CircleDetector.cpp.o.provides.build: src/trace/CMakeFiles/trace.dir/CircleDetector.cpp.o


src/trace/CMakeFiles/trace.dir/FitTrace.cpp.o: src/trace/CMakeFiles/trace.dir/flags.make
src/trace/CMakeFiles/trace.dir/FitTrace.cpp.o: ../src/trace/FitTrace.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robocon/workspace/agent-v2.0/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object src/trace/CMakeFiles/trace.dir/FitTrace.cpp.o"
	cd /home/robocon/workspace/agent-v2.0/cmake-build-debug/src/trace && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/trace.dir/FitTrace.cpp.o -c /home/robocon/workspace/agent-v2.0/src/trace/FitTrace.cpp

src/trace/CMakeFiles/trace.dir/FitTrace.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/trace.dir/FitTrace.cpp.i"
	cd /home/robocon/workspace/agent-v2.0/cmake-build-debug/src/trace && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robocon/workspace/agent-v2.0/src/trace/FitTrace.cpp > CMakeFiles/trace.dir/FitTrace.cpp.i

src/trace/CMakeFiles/trace.dir/FitTrace.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/trace.dir/FitTrace.cpp.s"
	cd /home/robocon/workspace/agent-v2.0/cmake-build-debug/src/trace && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robocon/workspace/agent-v2.0/src/trace/FitTrace.cpp -o CMakeFiles/trace.dir/FitTrace.cpp.s

src/trace/CMakeFiles/trace.dir/FitTrace.cpp.o.requires:

.PHONY : src/trace/CMakeFiles/trace.dir/FitTrace.cpp.o.requires

src/trace/CMakeFiles/trace.dir/FitTrace.cpp.o.provides: src/trace/CMakeFiles/trace.dir/FitTrace.cpp.o.requires
	$(MAKE) -f src/trace/CMakeFiles/trace.dir/build.make src/trace/CMakeFiles/trace.dir/FitTrace.cpp.o.provides.build
.PHONY : src/trace/CMakeFiles/trace.dir/FitTrace.cpp.o.provides

src/trace/CMakeFiles/trace.dir/FitTrace.cpp.o.provides.build: src/trace/CMakeFiles/trace.dir/FitTrace.cpp.o


src/trace/CMakeFiles/trace.dir/rgbd_camera.cpp.o: src/trace/CMakeFiles/trace.dir/flags.make
src/trace/CMakeFiles/trace.dir/rgbd_camera.cpp.o: ../src/trace/rgbd_camera.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robocon/workspace/agent-v2.0/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object src/trace/CMakeFiles/trace.dir/rgbd_camera.cpp.o"
	cd /home/robocon/workspace/agent-v2.0/cmake-build-debug/src/trace && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/trace.dir/rgbd_camera.cpp.o -c /home/robocon/workspace/agent-v2.0/src/trace/rgbd_camera.cpp

src/trace/CMakeFiles/trace.dir/rgbd_camera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/trace.dir/rgbd_camera.cpp.i"
	cd /home/robocon/workspace/agent-v2.0/cmake-build-debug/src/trace && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robocon/workspace/agent-v2.0/src/trace/rgbd_camera.cpp > CMakeFiles/trace.dir/rgbd_camera.cpp.i

src/trace/CMakeFiles/trace.dir/rgbd_camera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/trace.dir/rgbd_camera.cpp.s"
	cd /home/robocon/workspace/agent-v2.0/cmake-build-debug/src/trace && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robocon/workspace/agent-v2.0/src/trace/rgbd_camera.cpp -o CMakeFiles/trace.dir/rgbd_camera.cpp.s

src/trace/CMakeFiles/trace.dir/rgbd_camera.cpp.o.requires:

.PHONY : src/trace/CMakeFiles/trace.dir/rgbd_camera.cpp.o.requires

src/trace/CMakeFiles/trace.dir/rgbd_camera.cpp.o.provides: src/trace/CMakeFiles/trace.dir/rgbd_camera.cpp.o.requires
	$(MAKE) -f src/trace/CMakeFiles/trace.dir/build.make src/trace/CMakeFiles/trace.dir/rgbd_camera.cpp.o.provides.build
.PHONY : src/trace/CMakeFiles/trace.dir/rgbd_camera.cpp.o.provides

src/trace/CMakeFiles/trace.dir/rgbd_camera.cpp.o.provides.build: src/trace/CMakeFiles/trace.dir/rgbd_camera.cpp.o


src/trace/CMakeFiles/trace.dir/Trajectory.cpp.o: src/trace/CMakeFiles/trace.dir/flags.make
src/trace/CMakeFiles/trace.dir/Trajectory.cpp.o: ../src/trace/Trajectory.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robocon/workspace/agent-v2.0/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object src/trace/CMakeFiles/trace.dir/Trajectory.cpp.o"
	cd /home/robocon/workspace/agent-v2.0/cmake-build-debug/src/trace && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/trace.dir/Trajectory.cpp.o -c /home/robocon/workspace/agent-v2.0/src/trace/Trajectory.cpp

src/trace/CMakeFiles/trace.dir/Trajectory.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/trace.dir/Trajectory.cpp.i"
	cd /home/robocon/workspace/agent-v2.0/cmake-build-debug/src/trace && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robocon/workspace/agent-v2.0/src/trace/Trajectory.cpp > CMakeFiles/trace.dir/Trajectory.cpp.i

src/trace/CMakeFiles/trace.dir/Trajectory.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/trace.dir/Trajectory.cpp.s"
	cd /home/robocon/workspace/agent-v2.0/cmake-build-debug/src/trace && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robocon/workspace/agent-v2.0/src/trace/Trajectory.cpp -o CMakeFiles/trace.dir/Trajectory.cpp.s

src/trace/CMakeFiles/trace.dir/Trajectory.cpp.o.requires:

.PHONY : src/trace/CMakeFiles/trace.dir/Trajectory.cpp.o.requires

src/trace/CMakeFiles/trace.dir/Trajectory.cpp.o.provides: src/trace/CMakeFiles/trace.dir/Trajectory.cpp.o.requires
	$(MAKE) -f src/trace/CMakeFiles/trace.dir/build.make src/trace/CMakeFiles/trace.dir/Trajectory.cpp.o.provides.build
.PHONY : src/trace/CMakeFiles/trace.dir/Trajectory.cpp.o.provides

src/trace/CMakeFiles/trace.dir/Trajectory.cpp.o.provides.build: src/trace/CMakeFiles/trace.dir/Trajectory.cpp.o


src/trace/CMakeFiles/trace.dir/transformer.cpp.o: src/trace/CMakeFiles/trace.dir/flags.make
src/trace/CMakeFiles/trace.dir/transformer.cpp.o: ../src/trace/transformer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robocon/workspace/agent-v2.0/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object src/trace/CMakeFiles/trace.dir/transformer.cpp.o"
	cd /home/robocon/workspace/agent-v2.0/cmake-build-debug/src/trace && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/trace.dir/transformer.cpp.o -c /home/robocon/workspace/agent-v2.0/src/trace/transformer.cpp

src/trace/CMakeFiles/trace.dir/transformer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/trace.dir/transformer.cpp.i"
	cd /home/robocon/workspace/agent-v2.0/cmake-build-debug/src/trace && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robocon/workspace/agent-v2.0/src/trace/transformer.cpp > CMakeFiles/trace.dir/transformer.cpp.i

src/trace/CMakeFiles/trace.dir/transformer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/trace.dir/transformer.cpp.s"
	cd /home/robocon/workspace/agent-v2.0/cmake-build-debug/src/trace && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robocon/workspace/agent-v2.0/src/trace/transformer.cpp -o CMakeFiles/trace.dir/transformer.cpp.s

src/trace/CMakeFiles/trace.dir/transformer.cpp.o.requires:

.PHONY : src/trace/CMakeFiles/trace.dir/transformer.cpp.o.requires

src/trace/CMakeFiles/trace.dir/transformer.cpp.o.provides: src/trace/CMakeFiles/trace.dir/transformer.cpp.o.requires
	$(MAKE) -f src/trace/CMakeFiles/trace.dir/build.make src/trace/CMakeFiles/trace.dir/transformer.cpp.o.provides.build
.PHONY : src/trace/CMakeFiles/trace.dir/transformer.cpp.o.provides

src/trace/CMakeFiles/trace.dir/transformer.cpp.o.provides.build: src/trace/CMakeFiles/trace.dir/transformer.cpp.o


src/trace/CMakeFiles/trace.dir/Recorder.cpp.o: src/trace/CMakeFiles/trace.dir/flags.make
src/trace/CMakeFiles/trace.dir/Recorder.cpp.o: ../src/trace/Recorder.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robocon/workspace/agent-v2.0/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object src/trace/CMakeFiles/trace.dir/Recorder.cpp.o"
	cd /home/robocon/workspace/agent-v2.0/cmake-build-debug/src/trace && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/trace.dir/Recorder.cpp.o -c /home/robocon/workspace/agent-v2.0/src/trace/Recorder.cpp

src/trace/CMakeFiles/trace.dir/Recorder.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/trace.dir/Recorder.cpp.i"
	cd /home/robocon/workspace/agent-v2.0/cmake-build-debug/src/trace && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robocon/workspace/agent-v2.0/src/trace/Recorder.cpp > CMakeFiles/trace.dir/Recorder.cpp.i

src/trace/CMakeFiles/trace.dir/Recorder.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/trace.dir/Recorder.cpp.s"
	cd /home/robocon/workspace/agent-v2.0/cmake-build-debug/src/trace && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robocon/workspace/agent-v2.0/src/trace/Recorder.cpp -o CMakeFiles/trace.dir/Recorder.cpp.s

src/trace/CMakeFiles/trace.dir/Recorder.cpp.o.requires:

.PHONY : src/trace/CMakeFiles/trace.dir/Recorder.cpp.o.requires

src/trace/CMakeFiles/trace.dir/Recorder.cpp.o.provides: src/trace/CMakeFiles/trace.dir/Recorder.cpp.o.requires
	$(MAKE) -f src/trace/CMakeFiles/trace.dir/build.make src/trace/CMakeFiles/trace.dir/Recorder.cpp.o.provides.build
.PHONY : src/trace/CMakeFiles/trace.dir/Recorder.cpp.o.provides

src/trace/CMakeFiles/trace.dir/Recorder.cpp.o.provides.build: src/trace/CMakeFiles/trace.dir/Recorder.cpp.o


# Object files for target trace
trace_OBJECTS = \
"CMakeFiles/trace.dir/TraceController.cpp.o" \
"CMakeFiles/trace.dir/BallAssociate.cpp.o" \
"CMakeFiles/trace.dir/BallDetector.cpp.o" \
"CMakeFiles/trace.dir/CircleDetector.cpp.o" \
"CMakeFiles/trace.dir/FitTrace.cpp.o" \
"CMakeFiles/trace.dir/rgbd_camera.cpp.o" \
"CMakeFiles/trace.dir/Trajectory.cpp.o" \
"CMakeFiles/trace.dir/transformer.cpp.o" \
"CMakeFiles/trace.dir/Recorder.cpp.o"

# External object files for target trace
trace_EXTERNAL_OBJECTS =

src/trace/libtrace.a: src/trace/CMakeFiles/trace.dir/TraceController.cpp.o
src/trace/libtrace.a: src/trace/CMakeFiles/trace.dir/BallAssociate.cpp.o
src/trace/libtrace.a: src/trace/CMakeFiles/trace.dir/BallDetector.cpp.o
src/trace/libtrace.a: src/trace/CMakeFiles/trace.dir/CircleDetector.cpp.o
src/trace/libtrace.a: src/trace/CMakeFiles/trace.dir/FitTrace.cpp.o
src/trace/libtrace.a: src/trace/CMakeFiles/trace.dir/rgbd_camera.cpp.o
src/trace/libtrace.a: src/trace/CMakeFiles/trace.dir/Trajectory.cpp.o
src/trace/libtrace.a: src/trace/CMakeFiles/trace.dir/transformer.cpp.o
src/trace/libtrace.a: src/trace/CMakeFiles/trace.dir/Recorder.cpp.o
src/trace/libtrace.a: src/trace/CMakeFiles/trace.dir/build.make
src/trace/libtrace.a: src/trace/CMakeFiles/trace.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/robocon/workspace/agent-v2.0/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Linking CXX static library libtrace.a"
	cd /home/robocon/workspace/agent-v2.0/cmake-build-debug/src/trace && $(CMAKE_COMMAND) -P CMakeFiles/trace.dir/cmake_clean_target.cmake
	cd /home/robocon/workspace/agent-v2.0/cmake-build-debug/src/trace && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/trace.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/trace/CMakeFiles/trace.dir/build: src/trace/libtrace.a

.PHONY : src/trace/CMakeFiles/trace.dir/build

src/trace/CMakeFiles/trace.dir/requires: src/trace/CMakeFiles/trace.dir/TraceController.cpp.o.requires
src/trace/CMakeFiles/trace.dir/requires: src/trace/CMakeFiles/trace.dir/BallAssociate.cpp.o.requires
src/trace/CMakeFiles/trace.dir/requires: src/trace/CMakeFiles/trace.dir/BallDetector.cpp.o.requires
src/trace/CMakeFiles/trace.dir/requires: src/trace/CMakeFiles/trace.dir/CircleDetector.cpp.o.requires
src/trace/CMakeFiles/trace.dir/requires: src/trace/CMakeFiles/trace.dir/FitTrace.cpp.o.requires
src/trace/CMakeFiles/trace.dir/requires: src/trace/CMakeFiles/trace.dir/rgbd_camera.cpp.o.requires
src/trace/CMakeFiles/trace.dir/requires: src/trace/CMakeFiles/trace.dir/Trajectory.cpp.o.requires
src/trace/CMakeFiles/trace.dir/requires: src/trace/CMakeFiles/trace.dir/transformer.cpp.o.requires
src/trace/CMakeFiles/trace.dir/requires: src/trace/CMakeFiles/trace.dir/Recorder.cpp.o.requires

.PHONY : src/trace/CMakeFiles/trace.dir/requires

src/trace/CMakeFiles/trace.dir/clean:
	cd /home/robocon/workspace/agent-v2.0/cmake-build-debug/src/trace && $(CMAKE_COMMAND) -P CMakeFiles/trace.dir/cmake_clean.cmake
.PHONY : src/trace/CMakeFiles/trace.dir/clean

src/trace/CMakeFiles/trace.dir/depend:
	cd /home/robocon/workspace/agent-v2.0/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robocon/workspace/agent-v2.0 /home/robocon/workspace/agent-v2.0/src/trace /home/robocon/workspace/agent-v2.0/cmake-build-debug /home/robocon/workspace/agent-v2.0/cmake-build-debug/src/trace /home/robocon/workspace/agent-v2.0/cmake-build-debug/src/trace/CMakeFiles/trace.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/trace/CMakeFiles/trace.dir/depend
