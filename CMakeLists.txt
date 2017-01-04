cmake_minimum_required(VERSION 2.6)

project(main)

set(CMAKE_BUILD_TYPE debug)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -O2 -std=c++11")


FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})
find_package( OpenCV REQUIRED )

file(
	GLOB
	source_files
	src/*.[ch]pp
)

include_directories(${CMAKE_CURRENT_BINARY_DIR})


add_executable(
	pgm
	${source_files}
)

TARGET_LINK_LIBRARIES(pgm ${OpenCV_LIBS} ${QT_LIBRARIES})
