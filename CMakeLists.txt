cmake_minimum_required(VERSION 2.8)

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

message("install_prefix:${CMAKE_INSTALL_PREFIX}")
#INCLUDE_DIRECTORIES(${CMAKE_INSTALL_PREFIX}/include)
#LINK_DIRECTORIES(${CMAKE_INSTALL_PREFIX}/lib)

ADD_LIBRARY(visualStatesRunTime SHARED ${SOURCE_CPP_RUNTIME})
TARGET_LINK_LIBRARIES(visualStatesRunTime pthread)

INSTALL(TARGETS visualStatesRunTime DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/ COMPONENT visualStates)

#install runtime headers
INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/codegen/cpp/
		DESTINATION ${CMAKE_INSTALL_PREFIX}/include/ COMPONENT visualStates
		FILES_MATCHING PATTERN "*.h")

# install cpp sources for ros node generation
INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/codegen/cpp/
        DESTINATION ${CMAKE_INSTALL_PREFIX}/share/ COMPONENT visualStates)

#install python runtime dependencies
INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/codegen/python/
		DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python2.7/codegen/python/
		COMPONENT visualStates
		PATTERN __pycache__ EXCLUDE )

# install python runtime gui dependencies
INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/core DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python2.7 COMPONENT visualStates)

INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/generators DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python2.7 COMPONENT visualStates)

INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/gui DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python2.7 COMPONENT visualStates)

INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/interfaces DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python2.7 COMPONENT visualStates)

INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/parsers DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python2.7 COMPONENT visualStates)

INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/configs DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python2.7 COMPONENT visualStates)

# install python module __init__.py
INSTALL(FILES ${CMAKE_CURRENT_SOURCE_DIR}/__init__.py
		DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python2.7/ COMPONENT visualStates)
INSTALL(FILES ${CMAKE_CURRENT_SOURCE_DIR}/__init__.py
		DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python2.7/codegen/ COMPONENT visualStates)
INSTALL(FILES ${CMAKE_CURRENT_SOURCE_DIR}/__init__.py
		DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python2.7/codegen/python/ COMPONENT visualStates)

INSTALL(FILES ${CMAKE_CURRENT_SOURCE_DIR}/visualStates.py
		DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python2.7/ COMPONENT visualStates)

INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/resources DESTINATION ${CMAKE_INSTALL_PREFIX}/share/)

INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/examples DESTINATION ${CMAKE_INSTALL_PREFIX}/share/)

# Install launcher
install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/visualStates_py
    PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_EXECUTE GROUP_READ WORLD_EXECUTE WORLD_READ
    DESTINATION bin
    COMPONENT visualStates
)
