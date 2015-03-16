cmake_minimum_required(VERSION 2.8)
project( 3dscaner )
find_package( OpenCV REQUIRED )
find_package(PCL 1.2 REQUIRED)

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable( color_select_helper color_select_helper.cpp )
add_executable( calibrate calibrate.cpp )
add_executable( capture capture.cpp )
add_executable( scan scan.cpp )
add_executable( filter filter.cpp )
add_executable( render render.cpp )

add_library(scanlib scanlib.cpp)
add_library(capturelib capturelib.cpp)
add_library(arduino_driver arduino_driver.cpp)
target_link_libraries( color_select_helper   ${OpenCV_LIBS} )
target_link_libraries( calibrate   ${OpenCV_LIBS} /usr/local/lib/libinifile++.so)
target_link_libraries( capture  capturelib arduino_driver ${OpenCV_LIBS}  /usr/local/lib/libinifile++.so )
target_link_libraries( scan  scanlib arduino_driver ${OpenCV_LIBS} ${PCL_LIBRARIES} /usr/local/lib/libinifile++.so )
target_link_libraries( filter    ${PCL_LIBRARIES})
target_link_libraries( render   ${PCL_LIBRARIES} )


