cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(gazebo-rsb-model-state-plugin)

find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

find_package(gazebo 6.4.0 REQUIRED)
message(STATUS "GAZEBO version: ${GAZEBO_VERSION}") 
message(STATUS "GAZEBO_INCLUDE_DIRS: ${GAZEBO_INCLUDE_DIRS}")
message(STATUS "GAZEBO_LIBRARIES: ${GAZEBO_LIBRARIES}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

find_package(RSC REQUIRED)
message(STATUS "RSC version: ${RSC_VERSION}")	
include_directories(BEFORE SYSTEM ${RSC_INCLUDE_DIRS})	
add_definitions(${RSC_CFLAGS})
link_directories(${RSC_LIBRARY_DIR})

find_package(RSB 0.13 REQUIRED)	
message(STATUS "RSB version: ${RSB_VERSION}")	
include_directories(BEFORE SYSTEM ${RSB_INCLUDE_DIRS})	
add_definitions(${RSB_DEFINITIONS})

find_package(RST REQUIRED COMPONENTS sandbox)
INCLUDE_DIRECTORIES(BEFORE SYSTEM ${RST_INCLUDE_DIRS})
ADD_DEFINITIONS(${RST_CFLAGS} ${RSTSANDBOX_CFLAGS})

add_library(${PROJECT_NAME} SHARED src/gazebo-rsb-model-state-plugin.cpp src/gazebo-rsb-model-state-plugin.hpp)
target_link_libraries(${PROJECT_NAME} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${RSC_LIBRARIES} ${RSB_LIBRARIES} ${RST_LIBRARIES} ${RSTSANDBOX_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
     RUNTIME DESTINATION bin COMPONENT libraries
     LIBRARY DESTINATION lib COMPONENT libraries)