cmake_minimum_required(VERSION 2.8)
project(VisualStates)

find_package(catkin REQUIRED)

catkin_python_setup()

if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
	set(CMAKE_INSTALL_PREFIX /opt/VisualStates/ CACHE PATH "default install path" FORCE )
endif()

# set the cmake variables to be used in the source code
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/gui/cmakevars.py.in 
	${CMAKE_CURRENT_SOURCE_DIR}/gui/cmakevars.py
)

configure_file(
    visualStates_py.in
    visualStates_py
    @ONLY
)


SET(SOURCE_CPP_RUNTIME
	codegen/cpp/state.cpp
	codegen/cpp/transition.cpp
	codegen/cpp/temporaltransition.cpp
    codegen/cpp/runtimegui.cpp
)

ADD_LIBRARY(visualStatesRunTime SHARED ${SOURCE_CPP_RUNTIME})
TARGET_LINK_LIBRARIES(visualStatesRunTime pthread)
INSTALL(TARGETS visualStatesRunTime LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

INSTALL(PROGRAMS src/visualStates.py DESTINATINATION ${CATKIN_PACKAGE_BIN_DESTINATION}

#install runtime headers
INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/codegen/cpp/
		DESTINATION ${CMAKE_INSTALL_PREFIX}/include/
		FILES_MATCHING PATTERN "*.h")

# install cpp sources for ros node generation
INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/codegen/cpp/
        DESTINATION ${CMAKE_INSTALL_PREFIX}/share/)

#install python runtime dependencies
INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/codegen/python/
		DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python2.7/codegen/python/
		PATTERN __pycache__ EXCLUDE )

# install python runtime gui dependencies
INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/gui
		DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python2.7/
		FILES_MATCHING PATTERN "*.py"
		)

# install python module __init__.py
INSTALL(FILES ${CMAKE_CURRENT_SOURCE_DIR}/__init__.py
		DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python2.7/)
INSTALL(FILES ${CMAKE_CURRENT_SOURCE_DIR}/__init__.py
		DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python2.7/codegen/)
INSTALL(FILES ${CMAKE_CURRENT_SOURCE_DIR}/__init__.py
		DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python2.7/codegen/python/)

INSTALL(FILES ${CMAKE_CURRENT_SOURCE_DIR}/visualStates.py
		DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python2.7/)

INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/resources DESTINATION ${CMAKE_INSTALL_PREFIX}/share/)

INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/examples DESTINATION ${CMAKE_INSTALL_PREFIX}/share/)

# Install launcher
install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/visualStates_py
    PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_EXECUTE GROUP_READ WORLD_EXECUTE WORLD_READ
    DESTINATION bin
)