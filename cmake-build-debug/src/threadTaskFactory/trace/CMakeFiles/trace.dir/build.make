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
CMAKE_SOURCE_DIR = /home/robocon/workspace/singleton

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/robocon/workspace/singleton/cmake-build-debug

# Include any dependencies generated for this target.
include src/threadTaskFactory/trace/CMakeFiles/trace.dir/depend.make

# Include the progress variables for this target.
include src/threadTaskFactory/trace/CMakeFiles/trace.dir/progress.make

# Include the compile flags for this target's objects.
include src/threadTaskFactory/trace/CMakeFiles/trace.dir/flags.make

src/threadTaskFactory/trace/CMakeFiles/trace.dir/TraceController.cpp.o: src/threadTaskFactory/trace/CMakeFiles/trace.dir/flags.make
src/threadTaskFactory/trace/CMakeFiles/trace.dir/TraceController.cpp.o: ../src/threadTaskFactory/trace/TraceController.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robocon/workspace/singleton/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/threadTaskFactory/trace/CMakeFiles/trace.dir/TraceController.cpp.o"
	cd /home/robocon/workspace/singleton/cmake-build-debug/src/threadTaskFactory/trace && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/trace.dir/TraceController.cpp.o -c /home/robocon/workspace/singleton/src/threadTaskFactory/trace/TraceController.cpp

src/threadTaskFactory/trace/CMakeFiles/trace.dir/TraceController.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/trace.dir/TraceController.cpp.i"
	cd /home/robocon/workspace/singleton/cmake-build-debug/src/threadTaskFactory/trace && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robocon/workspace/singleton/src/threadTaskFactory/trace/TraceController.cpp > CMakeFiles/trace.dir/TraceController.cpp.i

src/threadTaskFactory/trace/CMakeFiles/trace.dir/TraceController.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/trace.dir/TraceController.cpp.s"
	cd /home/robocon/workspace/singleton/cmake-build-debug/src/threadTaskFactory/trace && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robocon/workspace/singleton/src/threadTaskFactory/trace/TraceController.cpp -o CMakeFiles/trace.dir/TraceController.cpp.s

src/threadTaskFactory/trace/CMakeFiles/trace.dir/TraceController.cpp.o.requires:

.PHONY : src/threadTaskFactory/trace/CMakeFiles/trace.dir/TraceController.cpp.o.requires

src/threadTaskFactory/trace/CMakeFiles/trace.dir/TraceController.cpp.o.provides: src/threadTaskFactory/trace/CMakeFiles/trace.dir/TraceController.cpp.o.requires
	$(MAKE) -f src/threadTaskFactory/trace/CMakeFiles/trace.dir/build.make src/threadTaskFactory/trace/CMakeFiles/trace.dir/TraceController.cpp.o.provides.build
.PHONY : src/threadTaskFactory/trace/CMakeFiles/trace.dir/TraceController.cpp.o.provides

src/threadTaskFactory/trace/CMakeFiles/trace.dir/TraceController.cpp.o.provides.build: src/threadTaskFactory/trace/CMakeFiles/trace.dir/TraceController.cpp.o


src/threadTaskFactory/trace/CMakeFiles/trace.dir/BallAssociate.cpp.o: src/threadTaskFactory/trace/CMakeFiles/trace.dir/flags.make
src/threadTaskFactory/trace/CMakeFiles/trace.dir/BallAssociate.cpp.o: ../src/threadTaskFactory/trace/BallAssociate.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robocon/workspace/singleton/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/threadTaskFactory/trace/CMakeFiles/trace.dir/BallAssociate.cpp.o"
	cd /home/robocon/workspace/singleton/cmake-build-debug/src/threadTaskFactory/trace && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/trace.dir/BallAssociate.cpp.o -c /home/robocon/workspace/singleton/src/threadTaskFactory/trace/BallAssociate.cpp

src/threadTaskFactory/trace/CMakeFiles/trace.dir/BallAssociate.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/trace.dir/BallAssociate.cpp.i"
	cd /home/robocon/workspace/singleton/cmake-build-debug/src/threadTaskFactory/trace && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robocon/workspace/singleton/src/threadTaskFactory/trace/BallAssociate.cpp > CMakeFiles/trace.dir/BallAssociate.cpp.i

src/threadTaskFactory/trace/CMakeFiles/trace.dir/BallAssociate.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/trace.dir/BallAssociate.cpp.s"
	cd /home/robocon/workspace/singleton/cmake-build-debug/src/threadTaskFactory/trace && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robocon/workspace/singleton/src/threadTaskFactory/trace/BallAssociate.cpp -o CMakeFiles/trace.dir/BallAssociate.cpp.s

src/threadTaskFactory/trace/CMakeFiles/trace.dir/BallAssociate.cpp.o.requires:

.PHONY : src/threadTaskFactory/trace/CMakeFiles/trace.dir/BallAssociate.cpp.o.requires

src/threadTaskFactory/trace/CMakeFiles/trace.dir/BallAssociate.cpp.o.provides: src/threadTaskFactory/trace/CMakeFiles/trace.dir/BallAssociate.cpp.o.requires
	$(MAKE) -f src/threadTaskFactory/trace/CMakeFiles/trace.dir/build.make src/threadTaskFactory/trace/CMakeFiles/trace.dir/BallAssociate.cpp.o.provides.build
.PHONY : src/threadTaskFactory/trace/CMakeFiles/trace.dir/BallAssociate.cpp.o.provides

src/threadTaskFactory/trace/CMakeFiles/trace.dir/BallAssociate.cpp.o.provides.build: src/threadTaskFactory/trace/CMakeFiles/trace.dir/BallAssociate.cpp.o


src/threadTaskFactory/trace/CMakeFiles/trace.dir/BallDetector.cpp.o: src/threadTaskFactory/trace/CMakeFiles/trace.dir/flags.make
src/threadTaskFactory/trace/CMakeFiles/trace.dir/BallDetector.cpp.o: ../src/threadTaskFactory/trace/BallDetector.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robocon/workspace/singleton/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/threadTaskFactory/trace/CMakeFiles/trace.dir/BallDetector.cpp.o"
	cd /home/robocon/workspace/singleton/cmake-build-debug/src/threadTaskFactory/trace && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/trace.dir/BallDetector.cpp.o -c /home/robocon/workspace/singleton/src/threadTaskFactory/trace/BallDetector.cpp

src/threadTaskFactory/trace/CMakeFiles/trace.dir/BallDetector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/trace.dir/BallDetector.cpp.i"
	cd /home/robocon/workspace/singleton/cmake-build-debug/src/threadTaskFactory/trace && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robocon/workspace/singleton/src/threadTaskFactory/trace/BallDetector.cpp > CMakeFiles/trace.dir/BallDetector.cpp.i

src/threadTaskFactory/trace/CMakeFiles/trace.dir/BallDetector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/trace.dir/BallDetector.cpp.s"
	cd /home/robocon/workspace/singleton/cmake-build-debug/src/threadTaskFactory/trace && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robocon/workspace/singleton/src/threadTaskFactory/trace/BallDetector.cpp -o CMakeFiles/trace.dir/BallDetector.cpp.s

src/threadTaskFactory/trace/CMakeFiles/trace.dir/BallDetector.cpp.o.requires:

.PHONY : src/threadTaskFactory/trace/CMakeFiles/trace.dir/BallDetector.cpp.o.requires

src/threadTaskFactory/trace/CMakeFiles/trace.dir/BallDetector.cpp.o.provides: src/threadTaskFactory/trace/CMakeFiles/trace.dir/BallDetector.cpp.o.requires
	$(MAKE) -f src/threadTaskFactory/trace/CMakeFiles/trace.dir/build.make src/threadTaskFactory/trace/CMakeFiles/trace.dir/BallDetector.cpp.o.provides.build
.PHONY : src/threadTaskFactory/trace/CMakeFiles/trace.dir/BallDetector.cpp.o.provides

src/threadTaskFactory/trace/CMakeFiles/trace.dir/BallDetector.cpp.o.provides.build: src/threadTaskFactory/trace/CMakeFiles/trace.dir/BallDetector.cpp.o


src/threadTaskFactory/trace/CMakeFiles/trace.dir/CircleDetector.cpp.o: src/threadTaskFactory/trace/CMakeFiles/trace.dir/flags.make
src/threadTaskFactory/trace/CMakeFiles/trace.dir/CircleDetector.cpp.o: ../src/threadTaskFactory/trace/CircleDetector.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robocon/workspace/singleton/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object src/threadTaskFactory/trace/CMakeFiles/trace.dir/CircleDetector.cpp.o"
	cd /home/robocon/workspace/singleton/cmake-build-debug/src/threadTaskFactory/trace && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/trace.dir/CircleDetector.cpp.o -c /home/robocon/workspace/singleton/src/threadTaskFactory/trace/CircleDetector.cpp

src/threadTaskFactory/trace/CMakeFiles/trace.dir/CircleDetector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/trace.dir/CircleDetector.cpp.i"
	cd /home/robocon/workspace/singleton/cmake-build-debug/src/threadTaskFactory/trace && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robocon/workspace/singleton/src/threadTaskFactory/trace/CircleDetector.cpp > CMakeFiles/trace.dir/CircleDetector.cpp.i

src/threadTaskFactory/trace/CMakeFiles/trace.dir/CircleDetector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/trace.dir/CircleDetector.cpp.s"
	cd /home/robocon/workspace/singleton/cmake-build-debug/src/threadTaskFactory/trace && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robocon/workspace/singleton/src/threadTaskFactory/trace/CircleDetector.cpp -o CMakeFiles/trace.dir/CircleDetector.cpp.s

src/threadTaskFactory/trace/CMakeFiles/trace.dir/CircleDetector.cpp.o.requires:

.PHONY : src/threadTaskFactory/trace/CMakeFiles/trace.dir/CircleDetector.cpp.o.requires

src/threadTaskFactory/trace/CMakeFiles/trace.dir/CircleDetector.cpp.o.provides: src/threadTaskFactory/trace/CMakeFiles/trace.dir/CircleDetector.cpp.o.requires
	$(MAKE) -f src/threadTaskFactory/trace/CMakeFiles/trace.dir/build.make src/threadTaskFactory/trace/CMakeFiles/trace.dir/CircleDetector.cpp.o.provides.build
.PHONY : src/threadTaskFactory/trace/CMakeFiles/trace.dir/CircleDetector.cpp.o.provides

src/threadTaskFactory/trace/CMakeFiles/trace.dir/CircleDetector.cpp.o.provides.build: src/threadTaskFactory/trace/CMakeFiles/trace.dir/CircleDetector.cpp.o


src/threadTaskFactory/trace/CMakeFiles/trace.dir/FitTrace.cpp.o: src/threadTaskFactory/trace/CMakeFiles/trace.dir/flags.make
src/threadTaskFactory/trace/CMakeFiles/trace.dir/FitTrace.cpp.o: ../src/threadTaskFactory/trace/FitTrace.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robocon/workspace/singleton/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object src/threadTaskFactory/trace/CMakeFiles/trace.dir/FitTrace.cpp.o"
	cd /home/robocon/workspace/singleton/cmake-build-debug/src/threadTaskFactory/trace && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/trace.dir/FitTrace.cpp.o -c /home/robocon/workspace/singleton/src/threadTaskFactory/trace/FitTrace.cpp

src/threadTaskFactory/trace/CMakeFiles/trace.dir/FitTrace.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/trace.dir/FitTrace.cpp.i"
	cd /home/robocon/workspace/singleton/cmake-build-debug/src/threadTaskFactory/trace && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robocon/workspace/singleton/src/threadTaskFactory/trace/FitTrace.cpp > CMakeFiles/trace.dir/FitTrace.cpp.i

src/threadTaskFactory/trace/CMakeFiles/trace.dir/FitTrace.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/trace.dir/FitTrace.cpp.s"
	cd /home/robocon/workspace/singleton/cmake-build-debug/src/threadTaskFactory/trace && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robocon/workspace/singleton/src/threadTaskFactory/trace/FitTrace.cpp -o CMakeFiles/trace.dir/FitTrace.cpp.s

src/threadTaskFactory/trace/CMakeFiles/trace.dir/FitTrace.cpp.o.requires:

.PHONY : src/threadTaskFactory/trace/CMakeFiles/trace.dir/FitTrace.cpp.o.requires

src/threadTaskFactory/trace/CMakeFiles/trace.dir/FitTrace.cpp.o.provides: src/threadTaskFactory/trace/CMakeFiles/trace.dir/FitTrace.cpp.o.requires
	$(MAKE) -f src/threadTaskFactory/trace/CMakeFiles/trace.dir/build.make src/threadTaskFactory/trace/CMakeFiles/trace.dir/FitTrace.cpp.o.provides.build
.PHONY : src/threadTaskFactory/trace/CMakeFiles/trace.dir/FitTrace.cpp.o.provides

src/threadTaskFactory/trace/CMakeFiles/trace.dir/FitTrace.cpp.o.provides.build: src/threadTaskFactory/trace/CMakeFiles/trace.dir/FitTrace.cpp.o


src/threadTaskFactory/trace/CMakeFiles/trace.dir/rgbd_camera.cpp.o: src/threadTaskFactory/trace/CMakeFiles/trace.dir/flags.make
src/threadTaskFactory/trace/CMakeFiles/trace.dir/rgbd_camera.cpp.o: ../src/threadTaskFactory/trace/rgbd_camera.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robocon/workspace/singleton/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object src/threadTaskFactory/trace/CMakeFiles/trace.dir/rgbd_camera.cpp.o"
	cd /home/robocon/workspace/singleton/cmake-build-debug/src/threadTaskFactory/trace && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/trace.dir/rgbd_camera.cpp.o -c /home/robocon/workspace/singleton/src/threadTaskFactory/trace/rgbd_camera.cpp

src/threadTaskFactory/trace/CMakeFiles/trace.dir/rgbd_camera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/trace.dir/rgbd_camera.cpp.i"
	cd /home/robocon/workspace/singleton/cmake-build-debug/src/threadTaskFactory/trace && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robocon/workspace/singleton/src/threadTaskFactory/trace/rgbd_camera.cpp > CMakeFiles/trace.dir/rgbd_camera.cpp.i

src/threadTaskFactory/trace/CMakeFiles/trace.dir/rgbd_camera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/trace.dir/rgbd_camera.cpp.s"
	cd /home/robocon/workspace/singleton/cmake-build-debug/src/threadTaskFactory/trace && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robocon/workspace/singleton/src/threadTaskFactory/trace/rgbd_camera.cpp -o CMakeFiles/trace.dir/rgbd_camera.cpp.s

src/threadTaskFactory/trace/CMakeFiles/trace.dir/rgbd_camera.cpp.o.requires:

.PHONY : src/threadTaskFactory/trace/CMakeFiles/trace.dir/rgbd_camera.cpp.o.requires

src/threadTaskFactory/trace/CMakeFiles/trace.dir/rgbd_camera.cpp.o.provides: src/threadTaskFactory/trace/CMakeFiles/trace.dir/rgbd_camera.cpp.o.requires
	$(MAKE) -f src/threadTaskFactory/trace/CMakeFiles/trace.dir/build.make src/threadTaskFactory/trace/CMakeFiles/trace.dir/rgbd_camera.cpp.o.provides.build
.PHONY : src/threadTaskFactory/trace/CMakeFiles/trace.dir/rgbd_camera.cpp.o.provides

src/threadTaskFactory/trace/CMakeFiles/trace.dir/rgbd_camera.cpp.o.provides.build: src/threadTaskFactory/trace/CMakeFiles/trace.dir/rgbd_camera.cpp.o


src/threadTaskFactory/trace/CMakeFiles/trace.dir/Trajectory.cpp.o: src/threadTaskFactory/trace/CMakeFiles/trace.dir/flags.make
src/threadTaskFactory/trace/CMakeFiles/trace.dir/Trajectory.cpp.o: ../src/threadTaskFactory/trace/Trajectory.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robocon/workspace/singleton/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object src/threadTaskFactory/trace/CMakeFiles/trace.dir/Trajectory.cpp.o"
	cd /home/robocon/workspace/singleton/cmake-build-debug/src/threadTaskFactory/trace && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/trace.dir/Trajectory.cpp.o -c /home/robocon/workspace/singleton/src/threadTaskFactory/trace/Trajectory.cpp

src/threadTaskFactory/trace/CMakeFiles/trace.dir/Trajectory.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/trace.dir/Trajectory.cpp.i"
	cd /home/robocon/workspace/singleton/cmake-build-debug/src/threadTaskFactory/trace && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robocon/workspace/singleton/src/threadTaskFactory/trace/Trajectory.cpp > CMakeFiles/trace.dir/Trajectory.cpp.i

src/threadTaskFactory/trace/CMakeFiles/trace.dir/Trajectory.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/trace.dir/Trajectory.cpp.s"
	cd /home/robocon/workspace/singleton/cmake-build-debug/src/threadTaskFactory/trace && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robocon/workspace/singleton/src/threadTaskFactory/trace/Trajectory.cpp -o CMakeFiles/trace.dir/Trajectory.cpp.s

src/threadTaskFactory/trace/CMakeFiles/trace.dir/Trajectory.cpp.o.requires:

.PHONY : src/threadTaskFactory/trace/CMakeFiles/trace.dir/Trajectory.cpp.o.requires

src/threadTaskFactory/trace/CMakeFiles/trace.dir/Trajectory.cpp.o.provides: src/threadTaskFactory/trace/CMakeFiles/trace.dir/Trajectory.cpp.o.requires
	$(MAKE) -f src/threadTaskFactory/trace/CMakeFiles/trace.dir/build.make src/threadTaskFactory/trace/CMakeFiles/trace.dir/Trajectory.cpp.o.provides.build
.PHONY : src/threadTaskFactory/trace/CMakeFiles/trace.dir/Trajectory.cpp.o.provides

src/threadTaskFactory/trace/CMakeFiles/trace.dir/Trajectory.cpp.o.provides.build: src/threadTaskFactory/trace/CMakeFiles/trace.dir/Trajectory.cpp.o


src/threadTaskFactory/trace/CMakeFiles/trace.dir/transformer.cpp.o: src/threadTaskFactory/trace/CMakeFiles/trace.dir/flags.make
src/threadTaskFactory/trace/CMakeFiles/trace.dir/transformer.cpp.o: ../src/threadTaskFactory/trace/transformer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robocon/workspace/singleton/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object src/threadTaskFactory/trace/CMakeFiles/trace.dir/transformer.cpp.o"
	cd /home/robocon/workspace/singleton/cmake-build-debug/src/threadTaskFactory/trace && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/trace.dir/transformer.cpp.o -c /home/robocon/workspace/singleton/src/threadTaskFactory/trace/transformer.cpp

src/threadTaskFactory/trace/CMakeFiles/trace.dir/transformer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/trace.dir/transformer.cpp.i"
	cd /home/robocon/workspace/singleton/cmake-build-debug/src/threadTaskFactory/trace && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robocon/workspace/singleton/src/threadTaskFactory/trace/transformer.cpp > CMakeFiles/trace.dir/transformer.cpp.i

src/threadTaskFactory/trace/CMakeFiles/trace.dir/transformer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/trace.dir/transformer.cpp.s"
	cd /home/robocon/workspace/singleton/cmake-build-debug/src/threadTaskFactory/trace && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robocon/workspace/singleton/src/threadTaskFactory/trace/transformer.cpp -o CMakeFiles/trace.dir/transformer.cpp.s

src/threadTaskFactory/trace/CMakeFiles/trace.dir/transformer.cpp.o.requires:

.PHONY : src/threadTaskFactory/trace/CMakeFiles/trace.dir/transformer.cpp.o.requires

src/threadTaskFactory/trace/CMakeFiles/trace.dir/transformer.cpp.o.provides: src/threadTaskFactory/trace/CMakeFiles/trace.dir/transformer.cpp.o.requires
	$(MAKE) -f src/threadTaskFactory/trace/CMakeFiles/trace.dir/build.make src/threadTaskFactory/trace/CMakeFiles/trace.dir/transformer.cpp.o.provides.build
.PHONY : src/threadTaskFactory/trace/CMakeFiles/trace.dir/transformer.cpp.o.provides

src/threadTaskFactory/trace/CMakeFiles/trace.dir/transformer.cpp.o.provides.build: src/threadTaskFactory/trace/CMakeFiles/trace.dir/transformer.cpp.o


src/threadTaskFactory/trace/CMakeFiles/trace.dir/Recorder.cpp.o: src/threadTaskFactory/trace/CMakeFiles/trace.dir/flags.make
src/threadTaskFactory/trace/CMakeFiles/trace.dir/Recorder.cpp.o: ../src/threadTaskFactory/trace/Recorder.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robocon/workspace/singleton/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object src/threadTaskFactory/trace/CMakeFiles/trace.dir/Recorder.cpp.o"
	cd /home/robocon/workspace/singleton/cmake-build-debug/src/threadTaskFactory/trace && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/trace.dir/Recorder.cpp.o -c /home/robocon/workspace/singleton/src/threadTaskFactory/trace/Recorder.cpp

src/threadTaskFactory/trace/CMakeFiles/trace.dir/Recorder.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/trace.dir/Recorder.cpp.i"
	cd /home/robocon/workspace/singleton/cmake-build-debug/src/threadTaskFactory/trace && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robocon/workspace/singleton/src/threadTaskFactory/trace/Recorder.cpp > CMakeFiles/trace.dir/Recorder.cpp.i

src/threadTaskFactory/trace/CMakeFiles/trace.dir/Recorder.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/trace.dir/Recorder.cpp.s"
	cd /home/robocon/workspace/singleton/cmake-build-debug/src/threadTaskFactory/trace && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robocon/workspace/singleton/src/threadTaskFactory/trace/Recorder.cpp -o CMakeFiles/trace.dir/Recorder.cpp.s

src/threadTaskFactory/trace/CMakeFiles/trace.dir/Recorder.cpp.o.requires:

.PHONY : src/threadTaskFactory/trace/CMakeFiles/trace.dir/Recorder.cpp.o.requires

src/threadTaskFactory/trace/CMakeFiles/trace.dir/Recorder.cpp.o.provides: src/threadTaskFactory/trace/CMakeFiles/trace.dir/Recorder.cpp.o.requires
	$(MAKE) -f src/threadTaskFactory/trace/CMakeFiles/trace.dir/build.make src/threadTaskFactory/trace/CMakeFiles/trace.dir/Recorder.cpp.o.provides.build
.PHONY : src/threadTaskFactory/trace/CMakeFiles/trace.dir/Recorder.cpp.o.provides

src/threadTaskFactory/trace/CMakeFiles/trace.dir/Recorder.cpp.o.provides.build: src/threadTaskFactory/trace/CMakeFiles/trace.dir/Recorder.cpp.o


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

src/threadTaskFactory/trace/libtrace.a: src/threadTaskFactory/trace/CMakeFiles/trace.dir/TraceController.cpp.o
src/threadTaskFactory/trace/libtrace.a: src/threadTaskFactory/trace/CMakeFiles/trace.dir/BallAssociate.cpp.o
src/threadTaskFactory/trace/libtrace.a: src/threadTaskFactory/trace/CMakeFiles/trace.dir/BallDetector.cpp.o
src/threadTaskFactory/trace/libtrace.a: src/threadTaskFactory/trace/CMakeFiles/trace.dir/CircleDetector.cpp.o
src/threadTaskFactory/trace/libtrace.a: src/threadTaskFactory/trace/CMakeFiles/trace.dir/FitTrace.cpp.o
src/threadTaskFactory/trace/libtrace.a: src/threadTaskFactory/trace/CMakeFiles/trace.dir/rgbd_camera.cpp.o
src/threadTaskFactory/trace/libtrace.a: src/threadTaskFactory/trace/CMakeFiles/trace.dir/Trajectory.cpp.o
src/threadTaskFactory/trace/libtrace.a: src/threadTaskFactory/trace/CMakeFiles/trace.dir/transformer.cpp.o
src/threadTaskFactory/trace/libtrace.a: src/threadTaskFactory/trace/CMakeFiles/trace.dir/Recorder.cpp.o
src/threadTaskFactory/trace/libtrace.a: src/threadTaskFactory/trace/CMakeFiles/trace.dir/build.make
src/threadTaskFactory/trace/libtrace.a: src/threadTaskFactory/trace/CMakeFiles/trace.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/robocon/workspace/singleton/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Linking CXX static library libtrace.a"
	cd /home/robocon/workspace/singleton/cmake-build-debug/src/threadTaskFactory/trace && $(CMAKE_COMMAND) -P CMakeFiles/trace.dir/cmake_clean_target.cmake
	cd /home/robocon/workspace/singleton/cmake-build-debug/src/threadTaskFactory/trace && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/trace.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/threadTaskFactory/trace/CMakeFiles/trace.dir/build: src/threadTaskFactory/trace/libtrace.a

.PHONY : src/threadTaskFactory/trace/CMakeFiles/trace.dir/build

src/threadTaskFactory/trace/CMakeFiles/trace.dir/requires: src/threadTaskFactory/trace/CMakeFiles/trace.dir/TraceController.cpp.o.requires
src/threadTaskFactory/trace/CMakeFiles/trace.dir/requires: src/threadTaskFactory/trace/CMakeFiles/trace.dir/BallAssociate.cpp.o.requires
src/threadTaskFactory/trace/CMakeFiles/trace.dir/requires: src/threadTaskFactory/trace/CMakeFiles/trace.dir/BallDetector.cpp.o.requires
src/threadTaskFactory/trace/CMakeFiles/trace.dir/requires: src/threadTaskFactory/trace/CMakeFiles/trace.dir/CircleDetector.cpp.o.requires
src/threadTaskFactory/trace/CMakeFiles/trace.dir/requires: src/threadTaskFactory/trace/CMakeFiles/trace.dir/FitTrace.cpp.o.requires
src/threadTaskFactory/trace/CMakeFiles/trace.dir/requires: src/threadTaskFactory/trace/CMakeFiles/trace.dir/rgbd_camera.cpp.o.requires
src/threadTaskFactory/trace/CMakeFiles/trace.dir/requires: src/threadTaskFactory/trace/CMakeFiles/trace.dir/Trajectory.cpp.o.requires
src/threadTaskFactory/trace/CMakeFiles/trace.dir/requires: src/threadTaskFactory/trace/CMakeFiles/trace.dir/transformer.cpp.o.requires
src/threadTaskFactory/trace/CMakeFiles/trace.dir/requires: src/threadTaskFactory/trace/CMakeFiles/trace.dir/Recorder.cpp.o.requires

.PHONY : src/threadTaskFactory/trace/CMakeFiles/trace.dir/requires

src/threadTaskFactory/trace/CMakeFiles/trace.dir/clean:
	cd /home/robocon/workspace/singleton/cmake-build-debug/src/threadTaskFactory/trace && $(CMAKE_COMMAND) -P CMakeFiles/trace.dir/cmake_clean.cmake
.PHONY : src/threadTaskFactory/trace/CMakeFiles/trace.dir/clean

src/threadTaskFactory/trace/CMakeFiles/trace.dir/depend:
	cd /home/robocon/workspace/singleton/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robocon/workspace/singleton /home/robocon/workspace/singleton/src/threadTaskFactory/trace /home/robocon/workspace/singleton/cmake-build-debug /home/robocon/workspace/singleton/cmake-build-debug/src/threadTaskFactory/trace /home/robocon/workspace/singleton/cmake-build-debug/src/threadTaskFactory/trace/CMakeFiles/trace.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/threadTaskFactory/trace/CMakeFiles/trace.dir/depend

