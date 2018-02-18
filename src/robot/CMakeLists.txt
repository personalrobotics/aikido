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
  ConcreteRobot.cpp
  ConcreteManipulator.cpp
  GrabMetadata.cpp
  util.cpp
  # Hand.cpp
)

add_library("${PROJECT_NAME}_robot" SHARED ${sources})
target_include_directories("${PROJECT_NAME}_robot" SYSTEM
  PUBLIC
    ${DART_INCLUDE_DIRS}
)

target_link_libraries("${PROJECT_NAME}_robot"
  PUBLIC
    "${PROJECT_NAME}_common"
    "${PROJECT_NAME}_io"
    ${DART_LIBRARIES}
)

target_compile_options("${PROJECT_NAME}_robot"
  PUBLIC ${AIKIDO_CXX_STANDARD_FLAGS}
)

add_component(${PROJECT_NAME} robot)
add_component_targets(${PROJECT_NAME} robot "${PROJECT_NAME}_robot")
add_component_dependencies(${PROJECT_NAME} robot common io)

format_add_sources(${sources})
