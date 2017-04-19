#==============================================================================
# Dependencies
#
# TODO: Move CatkinResourceRetriever into a separate, optional component.
find_package(TinyXML2 REQUIRED)

find_package(YamlCpp REQUIRED)

#==============================================================================
# Libraries
#
set(sources
  CatkinResourceRetriever.cpp
  ExecutorMultiplexer.cpp
  ExecutorThread.cpp
  KinBodyParser.cpp
  PseudoInverse.cpp
  RNG.cpp
  StepSequence.cpp
  stream.cpp
  string.cpp
  VanDerCorput.cpp
)

add_library("${PROJECT_NAME}_util" SHARED ${sources})
target_include_directories("${PROJECT_NAME}_util" SYSTEM
  PUBLIC
    ${Boost_INCLUDE_DIRS}
    ${DART_INCLUDE_DIRS}
    ${YAMLCPP_INCLUDE_DIRS}
  PRIVATE
    ${TinyXML2_INCLUDE_DIRS}
)
target_link_libraries("${PROJECT_NAME}_util"
  PUBLIC
    ${Boost_FILESYSTEM_LIBRARY}
    ${DART_LIBRARIES}
    ${YAMLCPP_LIBRARIES}
  PRIVATE
    ${TinyXML2_LIBRARIES}
)
target_compile_options("${PROJECT_NAME}_util"
  PUBLIC ${AIKIDO_CXX_STANDARD_FLAGS}
)

add_component(${PROJECT_NAME} util)
add_component_targets(${PROJECT_NAME} util "${PROJECT_NAME}_util")

coveralls_add_sources(${sources})
format_add_sources(${sources})
