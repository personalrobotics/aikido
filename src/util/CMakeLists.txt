#==============================================================================
# Dependencies
#
# TODO: Move CatkinResourceRetriever into a separate, optional component.
find_package(TinyXML2 REQUIRED)

#==============================================================================
# Libraries
#
set(sources
  CatkinResourceRetriever.cpp
  ExecutorMultiplexer.cpp
  ExecutorThread.cpp
  PseudoInverse.cpp
  RNG.cpp
  StepSequence.cpp
  VanDerCorput.cpp
  stream.cpp
)

add_library("${PROJECT_NAME}_util" SHARED ${sources})
target_include_directories("${PROJECT_NAME}_util" SYSTEM
  PUBLIC ${Boost_INCLUDE_DIRS}
  PRIVATE ${TinyXML_INCLUDE_DIRS}
)
target_link_libraries("${PROJECT_NAME}_util"
  ${Boost_FILESYSTEM_LIBRARY}
  ${DART_LIBRARIES}
  ${TinyXML2_LIBRARIES}
)

add_component(util)
add_component_targets(util "${PROJECT_NAME}_util")

coveralls_add_sources(${sources})
