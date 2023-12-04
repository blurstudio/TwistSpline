cmake_minimum_required(VERSION 3.9)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

project(TwistSpline)

set(CMAKE_CXX_STANDARD 14)

if (${CMAKE_VERSION} VERSION_GREATER_EQUAL 3.11)
    cmake_policy(SET CMP0072 NEW)
endif()

set(MAYA_VERSION 2020 CACHE STRING "Maya version")

find_package(Maya REQUIRED)
find_package(OpenGL REQUIRED)

set(SOURCE_FILES
    "src/drawOverride.h"
    "src/twistSpline.h"
    "src/twistSpline_maya.h"
    "src/twistSplineData.h"
    "src/twistSplineNode.h"
    "src/twistSplineUtils.h"
    "src/riderConstraint.h"
    "src/decomposeRotation.h"
    "src/twistTangentNode.h"

    "src/pluginMain.cpp"
    "src/drawOverride.cpp"
    "src/twistSplineData.cpp"
    "src/twistSplineNode.cpp"
    "src/riderConstraint.cpp"
    "src/twistTangentNode.cpp"
)

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})
target_include_directories(${PROJECT_NAME}
    PRIVATE Maya::Maya
    PUBLIC "${CMAKE_CURRENT_BINARY_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}"
)
target_link_libraries(${PROJECT_NAME} PRIVATE Maya::Maya OpenGL::GL)
MAYA_PLUGIN(${PROJECT_NAME})

