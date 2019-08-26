cmake_minimum_required(VERSION 3.5)
project(visualstatesros2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wunused-parameter)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 REQUIRED Widgets)

#include_directories(include/visualstates_INCLUDE_DIRS)

SET(SOURCE_CPP_RUNTIME
	src/visualstates/codegen/cpp/state.cpp
	src/visualstates/codegen/cpp/transition.cpp
	src/visualstates/codegen/cpp/temporaltransition.cpp
    src/visualstates/codegen/cpp/runtimegui.cpp
)

SET(HEADERS include/visualstates/conditionaltransition.h
            include/visualstates/runtimegui.h
            include/visualstates/state.h
            include/visualstates/temporaltransition.h
            include/visualstates/test.h 
            include/visualstates/transition.h
    )
include_directories(${rclcpp_INCLUDE_DIRS}   ${std_msgs_INCLUDE_DIRS})

#include_directories(include/visualstates/conditionaltransition.h
 #                   include/visualstates/runtimegui.h
  #                  include/visualstates/state.h
   #                 include/visualstates/temporaltransition.h
    #                include/visualstates/test.h  
     #     )


ADD_LIBRARY(visualStatesRunTime SHARED ${SOURCE_CPP_RUNTIME})

TARGET_LINK_LIBRARIES(visualStatesRunTime pthread)

#check_if_required
#ament_target_dependencies(visualStatesRunTime "rclcpp" "std_msgs" )

INSTALL(TARGETS visualStatesRunTime LIBRARY DESTINATION lib)

INSTALL(PROGRAMS scripts/main.py DESTINATION bin)

install(
  DIRECTORY include/visualstates
  DESTINATION lib/${PROJECT_NAME}
)

#install runtime headers
INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/visualstates/
		DESTINATION bin)

# install cpp sources for ros2 node generation
INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/visualstates/codegen/cpp/
        DESTINATION bin/codegen)

INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/resources DESTINATION bin)

install(FILES ${HEADERS} DESTINATION bin)

ament_package()

