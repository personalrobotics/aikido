#==============================================================================
# Dependencies

#==============================================================================
# Libraries
#
set(sources
  ExecutorMultiplexer.cpp
  ExecutorThread.cpp
  PseudoInverse.cpp
  RNG.cpp
  StepSequence.cpp
  stream.cpp
  string.cpp
  VanDerCorput.cpp
)

add_library("${PROJECT_NAME}_common" SHARED ${sources})
target_include_directories("${PROJECT_NAME}_common" SYSTEM
  PUBLIC
    ${Boost_INCLUDE_DIRS}
    ${DART_INCLUDE_DIRS}
    ${YAMLCPP_INCLUDE_DIRS}
  PRIVATE
    ${TinyXML2_INCLUDE_DIRS}
)
target_link_libraries("${PROJECT_NAME}_common"
  PUBLIC
    ${Boost_FILESYSTEM_LIBRARY}
    ${DART_LIBRARIES}
    ${YAMLCPP_LIBRARIES}
  PRIVATE
    ${TinyXML2_LIBRARIES}
)
target_compile_options("${PROJECT_NAME}_common"
  PUBLIC ${AIKIDO_CXX_STANDARD_FLAGS}
)
if(YAMLCPP_NODE_HAS_MARK)
  target_compile_definitions("${PROJECT_NAME}_common"
    PUBLIC YAMLCPP_NODE_HAS_MARK)
endif()

add_component(${PROJECT_NAME} common)
add_component_targets(${PROJECT_NAME} common "${PROJECT_NAME}_common")

clang_format_add_sources(${sources})
