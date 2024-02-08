cmake_minimum_required(VERSION 3.0)
project( capstone )

# Make project require C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

find_library(REALSENSE2_FOUND realsense2 HINTS ${LIBRARY_DIR} REQUIRED)
if(NOT REALSENSE2_FOUND)
    SET(REALSENSE2_FOUND "realsense2")
    message(WARN "Failed to find_library(realsense2)")
endif()
message(INFO "Found find_library(realsense2)")


find_package( OpenCV REQUIRED )

include_directories( ${CMAKE_SOURCE_DIR}/include/communication )
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${CMAKE_SOURCE_DIR}/include/core )
include_directories( ${ADDITIONAL_INCLUDE_DIRS} )

add_executable( capstone.exe src/communication/log.cpp src/communication/comm.cpp src/core/imaging.cpp  src/core/pid_ctrl.cpp src/core/motor_controller.cpp src/core/robot.cpp src/core/main.cpp)

# set gcc flag
target_link_libraries( capstone.exe -lrealsense2 )

target_link_libraries( capstone.exe ${OpenCV_LIBS} )
target_link_libraries( capstone.exe ${LIBRARY_DIR} )
# target_link_libraries(capstone.exe  realsense2.dll libglfw3.a libopengl32.a libglu32.a)
MESSAGE ( include directories ) 

install(
    TARGETS
    capstone.exe
    RUNTIME DESTINATION
    ${CMAKE_INSTALL_PREFIX}/bin
)

target_link_libraries( capstone.exe ${OpenCV_LIBS} )
target_link_libraries( capstone.exe  i2c )
MESSAGE ( include directories ) 
