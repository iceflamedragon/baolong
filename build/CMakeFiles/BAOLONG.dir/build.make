# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/bingflame/code_best/baolong

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/bingflame/code_best/baolong/build

# Include any dependencies generated for this target.
include CMakeFiles/BAOLONG.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/BAOLONG.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/BAOLONG.dir/flags.make

CMakeFiles/BAOLONG.dir/main.cpp.o: CMakeFiles/BAOLONG.dir/flags.make
CMakeFiles/BAOLONG.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bingflame/code_best/baolong/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/BAOLONG.dir/main.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/BAOLONG.dir/main.cpp.o -c /home/bingflame/code_best/baolong/main.cpp

CMakeFiles/BAOLONG.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/BAOLONG.dir/main.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bingflame/code_best/baolong/main.cpp > CMakeFiles/BAOLONG.dir/main.cpp.i

CMakeFiles/BAOLONG.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/BAOLONG.dir/main.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bingflame/code_best/baolong/main.cpp -o CMakeFiles/BAOLONG.dir/main.cpp.s

CMakeFiles/BAOLONG.dir/src/line.cpp.o: CMakeFiles/BAOLONG.dir/flags.make
CMakeFiles/BAOLONG.dir/src/line.cpp.o: ../src/line.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bingflame/code_best/baolong/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/BAOLONG.dir/src/line.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/BAOLONG.dir/src/line.cpp.o -c /home/bingflame/code_best/baolong/src/line.cpp

CMakeFiles/BAOLONG.dir/src/line.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/BAOLONG.dir/src/line.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bingflame/code_best/baolong/src/line.cpp > CMakeFiles/BAOLONG.dir/src/line.cpp.i

CMakeFiles/BAOLONG.dir/src/line.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/BAOLONG.dir/src/line.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bingflame/code_best/baolong/src/line.cpp -o CMakeFiles/BAOLONG.dir/src/line.cpp.s

CMakeFiles/BAOLONG.dir/src/picture.cpp.o: CMakeFiles/BAOLONG.dir/flags.make
CMakeFiles/BAOLONG.dir/src/picture.cpp.o: ../src/picture.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bingflame/code_best/baolong/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/BAOLONG.dir/src/picture.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/BAOLONG.dir/src/picture.cpp.o -c /home/bingflame/code_best/baolong/src/picture.cpp

CMakeFiles/BAOLONG.dir/src/picture.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/BAOLONG.dir/src/picture.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bingflame/code_best/baolong/src/picture.cpp > CMakeFiles/BAOLONG.dir/src/picture.cpp.i

CMakeFiles/BAOLONG.dir/src/picture.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/BAOLONG.dir/src/picture.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bingflame/code_best/baolong/src/picture.cpp -o CMakeFiles/BAOLONG.dir/src/picture.cpp.s

CMakeFiles/BAOLONG.dir/src/serial.cpp.o: CMakeFiles/BAOLONG.dir/flags.make
CMakeFiles/BAOLONG.dir/src/serial.cpp.o: ../src/serial.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bingflame/code_best/baolong/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/BAOLONG.dir/src/serial.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/BAOLONG.dir/src/serial.cpp.o -c /home/bingflame/code_best/baolong/src/serial.cpp

CMakeFiles/BAOLONG.dir/src/serial.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/BAOLONG.dir/src/serial.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bingflame/code_best/baolong/src/serial.cpp > CMakeFiles/BAOLONG.dir/src/serial.cpp.i

CMakeFiles/BAOLONG.dir/src/serial.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/BAOLONG.dir/src/serial.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bingflame/code_best/baolong/src/serial.cpp -o CMakeFiles/BAOLONG.dir/src/serial.cpp.s

CMakeFiles/BAOLONG.dir/src/on_Mouse.cpp.o: CMakeFiles/BAOLONG.dir/flags.make
CMakeFiles/BAOLONG.dir/src/on_Mouse.cpp.o: ../src/on_Mouse.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bingflame/code_best/baolong/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/BAOLONG.dir/src/on_Mouse.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/BAOLONG.dir/src/on_Mouse.cpp.o -c /home/bingflame/code_best/baolong/src/on_Mouse.cpp

CMakeFiles/BAOLONG.dir/src/on_Mouse.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/BAOLONG.dir/src/on_Mouse.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bingflame/code_best/baolong/src/on_Mouse.cpp > CMakeFiles/BAOLONG.dir/src/on_Mouse.cpp.i

CMakeFiles/BAOLONG.dir/src/on_Mouse.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/BAOLONG.dir/src/on_Mouse.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bingflame/code_best/baolong/src/on_Mouse.cpp -o CMakeFiles/BAOLONG.dir/src/on_Mouse.cpp.s

# Object files for target BAOLONG
BAOLONG_OBJECTS = \
"CMakeFiles/BAOLONG.dir/main.cpp.o" \
"CMakeFiles/BAOLONG.dir/src/line.cpp.o" \
"CMakeFiles/BAOLONG.dir/src/picture.cpp.o" \
"CMakeFiles/BAOLONG.dir/src/serial.cpp.o" \
"CMakeFiles/BAOLONG.dir/src/on_Mouse.cpp.o"

# External object files for target BAOLONG
BAOLONG_EXTERNAL_OBJECTS =

BAOLONG: CMakeFiles/BAOLONG.dir/main.cpp.o
BAOLONG: CMakeFiles/BAOLONG.dir/src/line.cpp.o
BAOLONG: CMakeFiles/BAOLONG.dir/src/picture.cpp.o
BAOLONG: CMakeFiles/BAOLONG.dir/src/serial.cpp.o
BAOLONG: CMakeFiles/BAOLONG.dir/src/on_Mouse.cpp.o
BAOLONG: CMakeFiles/BAOLONG.dir/build.make
BAOLONG: /usr/local/lib/libopencv_gapi.so.4.8.1
BAOLONG: /usr/local/lib/libopencv_highgui.so.4.8.1
BAOLONG: /usr/local/lib/libopencv_ml.so.4.8.1
BAOLONG: /usr/local/lib/libopencv_objdetect.so.4.8.1
BAOLONG: /usr/local/lib/libopencv_photo.so.4.8.1
BAOLONG: /usr/local/lib/libopencv_stitching.so.4.8.1
BAOLONG: /usr/local/lib/libopencv_video.so.4.8.1
BAOLONG: /usr/local/lib/libopencv_videoio.so.4.8.1
BAOLONG: /usr/local/lib/libopencv_imgcodecs.so.4.8.1
BAOLONG: /usr/local/lib/libopencv_dnn.so.4.8.1
BAOLONG: /usr/local/lib/libopencv_calib3d.so.4.8.1
BAOLONG: /usr/local/lib/libopencv_features2d.so.4.8.1
BAOLONG: /usr/local/lib/libopencv_flann.so.4.8.1
BAOLONG: /usr/local/lib/libopencv_imgproc.so.4.8.1
BAOLONG: /usr/local/lib/libopencv_core.so.4.8.1
BAOLONG: CMakeFiles/BAOLONG.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/bingflame/code_best/baolong/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX executable BAOLONG"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/BAOLONG.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/BAOLONG.dir/build: BAOLONG

.PHONY : CMakeFiles/BAOLONG.dir/build

CMakeFiles/BAOLONG.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/BAOLONG.dir/cmake_clean.cmake
.PHONY : CMakeFiles/BAOLONG.dir/clean

CMakeFiles/BAOLONG.dir/depend:
	cd /home/bingflame/code_best/baolong/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bingflame/code_best/baolong /home/bingflame/code_best/baolong /home/bingflame/code_best/baolong/build /home/bingflame/code_best/baolong/build /home/bingflame/code_best/baolong/build/CMakeFiles/BAOLONG.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/BAOLONG.dir/depend

