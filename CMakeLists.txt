cmake_minimum_required(VERSION 2.6)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/modules)

project(TwistSpline)
find_package(Maya REQUIRED)
find_package(OpenGL REQUIRED)
set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_BINARY_DIR}/../output/Maya${MAYA_VERSION})

set(SOURCE_FILES
    "mpkdtree.h"

    "twistSpline.h"
    "twistSpline_maya.h"
    "twistSplineData.h"
    "twistSplineNode.h"
    "riderConstraint.h"
    "decomposeRotation.h"
    "twistTangentNode.h"

    "pluginMain.cpp"
    "twistSplineData.cpp"
    "twistSplineNode.cpp"
    "riderConstraint.cpp"
    "twistTangentNode.cpp"
)

include_directories(${MAYA_INCLUDE_DIR})
link_directories(${MAYA_LIBRARY_DIR})
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${MAYA_LIBRARIES} ${OPENGL_LIBRARY})

MAYA_PLUGIN(${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} ${MAYA_TARGET_TYPE} DESTINATION plug-ins)

