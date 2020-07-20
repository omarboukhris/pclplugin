cmake_minimum_required(VERSION 3.1)
project(PCLPlugin)

if(COMMAND setupGitInfo)
        setupGitInfo(PLUGIN_GIT_INFO)
endif()

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


find_package(PCL 1.9 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

set_source_files_properties(${IGNORED_FILES} PROPERTIES HEADER_FILE_ONLY TRUE)
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES} ${HEADER_FILES} ${SCENES_FILES} ${README_FILES} )
set(${PROJECT_NAME}_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/src;${PCL_INCLUDE_DIRS}")

target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${${PROJECT_NAME}_INCLUDE_DIRS}>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DPLUGIN_DATA_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\"")
target_link_libraries(
        ${PROJECT_NAME}
	SofaSimpleFem SofaGeneralSimpleFem
	SofaBaseTopology SofaBaseMechanics
	SofaBaseMechanics
	SofaBaseLinearSolver
	SofaBaseVisual SofaHelper SofaOpenglVisual
        SofaSimulationTree SofaUserInteraction
	ConstraintGeometry NeedleConstraint ${PCL_LIBRARIES}
)
#target_link_libraries(${PROJECT_NAME} SofaFramework SofaBase SofaSimulation SofaGeneral ConstraintGeometry NeedleConstraint)

sofa_generate_package(
	NAME ${PROJECT_NAME}
	VERSION 0.1
	TARGETS ${PROJECT_NAME}
	INCLUDE_INSTALL_DIR ${${PROJECT_NAME}_INCLUDE_DIRS}
)

