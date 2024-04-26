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
CMAKE_SOURCE_DIR = /home/edgeboard/car/demo/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/edgeboard/car/demo/build

# Include any dependencies generated for this target.
include CMakeFiles/img2video.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/img2video.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/img2video.dir/flags.make

CMakeFiles/img2video.dir/tool/img2video.cpp.o: CMakeFiles/img2video.dir/flags.make
CMakeFiles/img2video.dir/tool/img2video.cpp.o: /home/edgeboard/car/demo/src/tool/img2video.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/edgeboard/car/demo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/img2video.dir/tool/img2video.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/img2video.dir/tool/img2video.cpp.o -c /home/edgeboard/car/demo/src/tool/img2video.cpp

CMakeFiles/img2video.dir/tool/img2video.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/img2video.dir/tool/img2video.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/edgeboard/car/demo/src/tool/img2video.cpp > CMakeFiles/img2video.dir/tool/img2video.cpp.i

CMakeFiles/img2video.dir/tool/img2video.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/img2video.dir/tool/img2video.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/edgeboard/car/demo/src/tool/img2video.cpp -o CMakeFiles/img2video.dir/tool/img2video.cpp.s

CMakeFiles/img2video.dir/tool/img2video.cpp.o.requires:

.PHONY : CMakeFiles/img2video.dir/tool/img2video.cpp.o.requires

CMakeFiles/img2video.dir/tool/img2video.cpp.o.provides: CMakeFiles/img2video.dir/tool/img2video.cpp.o.requires
	$(MAKE) -f CMakeFiles/img2video.dir/build.make CMakeFiles/img2video.dir/tool/img2video.cpp.o.provides.build
.PHONY : CMakeFiles/img2video.dir/tool/img2video.cpp.o.provides

CMakeFiles/img2video.dir/tool/img2video.cpp.o.provides.build: CMakeFiles/img2video.dir/tool/img2video.cpp.o


# Object files for target img2video
img2video_OBJECTS = \
"CMakeFiles/img2video.dir/tool/img2video.cpp.o"

# External object files for target img2video
img2video_EXTERNAL_OBJECTS =

img2video: CMakeFiles/img2video.dir/tool/img2video.cpp.o
img2video: CMakeFiles/img2video.dir/build.make
img2video: /usr/local/lib/libopencv_dnn.so.4.4.0
img2video: /usr/local/lib/libopencv_gapi.so.4.4.0
img2video: /usr/local/lib/libopencv_highgui.so.4.4.0
img2video: /usr/local/lib/libopencv_ml.so.4.4.0
img2video: /usr/local/lib/libopencv_objdetect.so.4.4.0
img2video: /usr/local/lib/libopencv_photo.so.4.4.0
img2video: /usr/local/lib/libopencv_stitching.so.4.4.0
img2video: /usr/local/lib/libopencv_video.so.4.4.0
img2video: /usr/local/lib/libopencv_videoio.so.4.4.0
img2video: /usr/local/lib/libopencv_imgcodecs.so.4.4.0
img2video: /usr/local/lib/libopencv_calib3d.so.4.4.0
img2video: /usr/local/lib/libopencv_features2d.so.4.4.0
img2video: /usr/local/lib/libopencv_flann.so.4.4.0
img2video: /usr/local/lib/libopencv_imgproc.so.4.4.0
img2video: /usr/local/lib/libopencv_core.so.4.4.0
img2video: CMakeFiles/img2video.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/edgeboard/car/demo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable img2video"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/img2video.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/img2video.dir/build: img2video

.PHONY : CMakeFiles/img2video.dir/build

CMakeFiles/img2video.dir/requires: CMakeFiles/img2video.dir/tool/img2video.cpp.o.requires

.PHONY : CMakeFiles/img2video.dir/requires

CMakeFiles/img2video.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/img2video.dir/cmake_clean.cmake
.PHONY : CMakeFiles/img2video.dir/clean

CMakeFiles/img2video.dir/depend:
	cd /home/edgeboard/car/demo/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/edgeboard/car/demo/src /home/edgeboard/car/demo/src /home/edgeboard/car/demo/build /home/edgeboard/car/demo/build /home/edgeboard/car/demo/build/CMakeFiles/img2video.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/img2video.dir/depend

