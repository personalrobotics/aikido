#==============================================================================
# Dependencies
#

include(CheckCXXSourceCompiles)
set(CMAKE_REQUIRED_DEFINITIONS "")
set(CMAKE_REQUIRED_FLAGS "")

#==============================================================================
# Libraries
#
set(sources
  Robot.cpp
  Manipulator.cpp
  MobileBase.cpp
)

add_library("${PROJECT_NAME}_io" SHARED ${sources})
target_include_directories("${PROJECT_NAME}_io" SYSTEM
  PUBLIC
    ${DART_INCLUDE_DIRS}
)

target_link_libraries("${PROJECT_NAME}_robot"
  PUBLIC
    "${PROJECT_NAME}_common"
    ${DART_LIBRARIES}
)

target_compile_options("${PROJECT_NAME}_robot"
  PUBLIC ${AIKIDO_CXX_STANDARD_FLAGS}
)

add_component(${PROJECT_NAME} robot)
add_component_targets(${PROJECT_NAME} robot "${PROJECT_NAME}_robot")
add_component_dependencies(${PROJECT_NAME} robot common)

format_add_sources(${sources})
