cmake_minimum_required(VERSION 3.1)
project(PCLPlugin)

find_package(PCL 1.9 REQUIRED)


file(GLOB_RECURSE HEADER_FILES
    "src/*.h"
    "src/*.inl"
)

file(GLOB_RECURSE SOURCE_FILES

    "src/*.cpp"
)

file(GLOB_RECURSE SCENES_FILES
    "scenes/*.scn"
    "scenes/*.xml"
)

file(GLOB_RECURSE IGNORED_FILES
    "ignored/*.h"
    "ignored/*.inl"
    "ignored/*.cpp"
)

file(GLOB_RECURSE DEPRECATED_FILES
    "deprecated/*.h"
    "deprecated/*.inl"
    "deprecated/*.cpp"
)


include_directories("${CMAKE_CURRENT_SOURCE_DIR}/ignored")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/deprecated")
include_directories(${PCL_INCLUDE_DIRS})

set_source_files_properties(${IGNORED_FILES} PROPERTIES HEADER_FILE_ONLY TRUE)
#set_source_files_properties(${DEPRECATED_FILES} PROPERTIES HEADER_FILE_ONLY TRUE)

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES} ${HEADER_FILES} ${SCENES_FILES} ${IGNORED_FILES} ${DEPRECATED_FILES}  ${README_FILES} )
#set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DPLUGIN_DATA_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\"")
#target_link_libraries(${PROJECT_NAME} SofaFramework SofaBase SofaSimulation SofaGeneral ConstraintGeometry NeedleConstraint)

set(${PROJECT_NAME}_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${PCLPLUGIN_INCLUDE_DIRS}>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
set_target_properties(${PROJECT_NAME} PROPERTIES DEBUG_POSTFIX "_d")
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DPLUGIN_DATA_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\"")
target_link_libraries(${PROJECT_NAME} SofaSimpleFem SofaGeneralSimpleFem SofaBaseTopology SofaBaseMechanics  SofaBaseLinearSolver
SofaBaseVisual SofaHelper SofaOpenglVisual SofaComponentBase SofaSimulationTree SofaUserInteraction ConstraintGeometry  ${PCL_LIBRARIES})

