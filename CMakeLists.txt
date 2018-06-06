cmake_minimum_required(VERSION 2.8)
project(visualstates)

find_package(catkin REQUIRED)

catkin_python_setup()
catkin_package()

#if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
#	set(CMAKE_INSTALL_PREFIX /opt/VisualStates/ CACHE PATH "default install path" FORCE )
#endif()

# set the cmake variables to be used in the source code
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/visualstates/gui/cmakevars.py.in
	${CMAKE_CURRENT_SOURCE_DIR}/src/visualstates/gui/cmakevars.py
)

configure_file(
    visualstates.in
    visualstates
    @ONLY
)

SET(SOURCE_CPP_RUNTIME
	src/codegen/cpp/state.cpp
	src/codegen/cpp/transition.cpp
	src/codegen/cpp/temporaltransition.cpp
    src/codegen/cpp/runtimegui.cpp
)

include_directories(include)

ADD_LIBRARY(visualStatesRunTime SHARED ${SOURCE_CPP_RUNTIME})
TARGET_LINK_LIBRARIES(visualStatesRunTime pthread)
INSTALL(TARGETS visualStatesRunTime LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

#INSTALL(PROGRAMS visualStates DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#install runtime headers
INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/visualstates/
		DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# install cpp sources for ros node generation
INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/codegen/cpp/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/codegen)

#install python runtime dependencies
#INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/codegen/python/
#		DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python2.7/codegen/python/
#		PATTERN __pycache__ EXCLUDE )

# install python runtime gui dependencies
#INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/gui
#		DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python2.7/gui/
#		FILES_MATCHING PATTERN "*.py"
#		PATTERN __pycache__ EXCLUDE 
#		)


# install python module __init__.py
#INSTALL(FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/__init__.py
#		DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python2.7/)
#INSTALL(FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/__init__.py
#		DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python2.7/codegen/)
#INSTALL(FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/__init__.py
#		DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python2.7/codegen/python/)
#INSTALL(FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/__init__.py
#		DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python2.7/gui/)

#INSTALL(FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/visualStates.py
#		DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python2.7/)

INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/resources DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/examples DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Install launcher
#install(
#    FILES ${CMAKE_CURRENT_BINARY_DIR}/visualStates_py
#   PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_EXECUTE GROUP_READ WORLD_EXECUTE WORLD_READ
#    DESTINATION bin
#)
