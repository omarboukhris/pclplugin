cmake_minimum_required(VERSION 3.1)
project(PCLPlugin)

#if(COMMAND setupGitInfo)
#        setupGitInfo(PLUGIN_GIT_INFO)
#endif()
 
file(GLOB_RECURSE HEADER_FILES
	"src/sofa/PCLPlugin/filter/*.h"
	"src/sofa/PCLPlugin/filter/*.inl"
	"src/sofa/PCLPlugin/registration/*.h"
	"src/sofa/PCLPlugin/registration/*.inl"
	"src/sofa/PCLPlugin/utils/*.h"
	"src/sofa/PCLPlugin/utils/*.inl"
	"src/sofa/PCLPlugin/PointCloudData.h"
)

file(GLOB_RECURSE SOURCE_FILES
	"src/sofa/PCLPlugin/filter/*.cpp"
	"src/sofa/PCLPlugin/registration/*.cpp"
	"src/sofa/PCLPlugin/utils/*.cpp"
	"src/sofa/PCLPlugin/initPlugin.cpp"
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

#find_package(ConstraintGeometry QUIET)
list(APPEND "" ${HEADER_FILES_CG})
list(APPEND "" ${SOURCE_FILES_CG})
if(ConstraintGeometry_FOUND)
	file(GLOB_RECURSE HEADER_FILES_CG "src/sofa/PCLPlugin/PCLLeastSquare.h src/sofa/PCLPlugin/PCLLeastSquare.inl")
	file(GLOB_RECURSE SOURCE_FILES_CG "src/sofa/PCLPlugin/PCLLeastSquare.cpp")
endif()

find_package(PCL 1.9 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

set_source_files_properties(${IGNORED_FILES} PROPERTIES HEADER_FILE_ONLY TRUE)
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES} ${SOURCE_FILES_CG} ${HEADER_FILES} ${HEADER_FILES_CG} ${SCENES_FILES} ${README_FILES} )
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
	${PCL_LIBRARIES}
)

if (ConstraintGeometry_FOUND)
	target_link_libraries( ${PROJECT_NAME} ConstraintGeometry)
endif()

#target_link_libraries(${PROJECT_NAME} SofaFramework SofaBase SofaSimulation SofaGeneral ConstraintGeometry NeedleConstraint)

sofa_create_package_with_targets(
    PACKAGE_NAME ${PROJECT_NAME} 
    PACKAGE_VERSION 0.1 
    TARGETS ${PROJECT_NAME} AUTO_SET_TARGET_PROPERTIES
    INCLUDE_SOURCE_DIR "src"
    INCLUDE_INSTALL_DIR ${PROJECT_NAME} 
    RELOCATABLE "plugins"
    )

