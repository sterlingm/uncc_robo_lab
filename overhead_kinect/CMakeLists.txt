cmake_minimum_required(VERSION 2.8.12.2)
project(view_cloud)

# OpenCV
find_package(OpenCV REQUIRED)
IF (OpenCV_FOUND)
  MESSAGE("-- Found OpenCV version ${OpenCV_VERSION}")
  MESSAGE("-- OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
ENDIF (OpenCV_FOUND)

# PCL
find_package(PCL REQUIRED)
IF (PCL_FOUND)
  MESSAGE("-- Found PCL version ${PCL_VERSION}")
  MESSAGE("-- PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS}")
  MESSAGE("-- PCL_LIBRARY_DIRS: ${PCL_LIBRARY_DIRS}")
  MESSAGE("-- PCL_LIBRARIES: ${PCL_LIBRARIES}")
  MESSAGE("-- PCL_DEFINITIONS: ${PCL_DEFINITIONS}")
ENDIF (PCL_FOUND)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  segment
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(
  ${PCL_DEFINITIONS}
)

aux_source_directory(. DIR_SRCS)

add_executable(capture_cloud capture-image.cpp)
target_link_libraries(capture_cloud 
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)


add_executable(calibrate calibrate.cpp)
target_link_libraries(calibrate 
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)


add_executable(cloud_viewer cloud_viewer.cpp)
target_link_libraries(cloud_viewer
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)
