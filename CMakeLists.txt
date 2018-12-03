cmake_minimum_required(VERSION 2.8.3)
project(open_karto)

set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
  DEPENDS Boost
  INCLUDE_DIRS
    include
  LIBRARIES
    karto
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_library(karto SHARED src/Karto.cpp src/Mapper.cpp)
target_link_libraries(karto ${Boost_LIBRARIES})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS karto
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)


### tutorial1
add_executable(
  Karto_Tutorial1 
  samples/tutorial1.cpp
)

target_link_libraries(Karto_Tutorial1 karto)

if (UNIX)
  target_link_libraries(Karto_Tutorial1 "pthread")
endif()

### tutorial2
set(CMAKE_CXX_STANDARD 11)
find_package(catkin REQUIRED
  COMPONENTS
    cmake_modules
    message_filters
    nav_msgs
    open_karto
    rosconsole
    roscpp
    sensor_msgs
    sparse_bundle_adjustment
    tf
    visualization_msgs
)

include_directories(${catkin_INCLUDE_DIRS})
include_directories("/usr/include/eigen3")
include_directories(samples)
catkin_package()

add_executable(
  Karto_Tutorial2
  samples/tutorial2.cpp
  samples/spa_solver.cpp
)
target_link_libraries(Karto_Tutorial2 karto)

if (UNIX)
  target_link_libraries(Karto_Tutorial2 "pthread" ${catkin_LIBRARIES})
endif()

install(TARGETS Karto_Tutorial2
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
