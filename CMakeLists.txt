

# cmake needs this line
cmake_minimum_required(VERSION 3.5)

# Define project name

project(BAOLONG LANGUAGES CXX)
 
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
find_package(OpenCV REQUIRED)

# If the package has been found, several variables will
# be set, you can find the full list with descriptions
# in the OpenCVConfig.cmake file.
# Print some message showing some of them
message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

set(THREADS_HAVE_PTHREAD_ARG 1)
find_package(Threads REQUIRED)

include_directories(/usr/local/include/opencv4/ include src  usr/local/lib /usr/local/include/libserial/ /usr/include/libserial)
# Declare the executable target built from your sources
add_executable(BAOLONG main.cpp src/line.cpp src/picture.cpp src/serial.cpp src/on_Mouse.cpp)

find_package(PkgConfig)
pkg_check_modules(SERIAL libserial)

# Link your application with OpenCV libraries
# target_link_libraries(BAOLONG  ${OpenCV_LIBS} )



target_link_libraries(BAOLONG PRIVATE ${SERIAL_LDFLAGS} ${CMAKE_THREAD_LIBS_INIT}  ${OpenCV_LIBS})
target_include_directories(BAOLONG PRIVATE ${SERIAL_INCLUDE_DIRS})
