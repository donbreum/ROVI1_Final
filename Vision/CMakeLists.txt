set(CMAKE_CXX_STANDARD 11) # C++11...
set(CMAKE_CXX_STANDARD_REQUIRED ON) #...is required...
set(CMAKE_CXX_EXTENSIONS OFF) #...without compiler extensions like gnu++11
set(OpenCV_DIR /opt/ros/kinetic/share/OpenCV-3.2.0-dev)


set(SOURCE_FILES main.cpp)
file(GLOB SOURCES
        *.h
        *.hpp
        *.cpp
        )
	
cmake_minimum_required(VERSION 2.8)
project( ROVI_FinalVision ) 
find_package( OpenCV 3 REQUIRED )

#add_library(vis src/Vision.hpp src/Vision.cpp)
add_executable(ROVI_FinalVision ${SOURCE_FILES} ${SOURCES})
target_link_libraries( ROVI_FinalVision ${OpenCV_LIBS})  

