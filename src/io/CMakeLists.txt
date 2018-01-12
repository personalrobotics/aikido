#==============================================================================
# Dependencies
#
# TODO: Move CatkinResourceRetriever into a separate, optional component.
find_package(TinyXML2 REQUIRED)

find_package(YamlCpp REQUIRED)

include(CheckCXXSourceCompiles)
set(CMAKE_REQUIRED_DEFINITIONS "")
set(CMAKE_REQUIRED_FLAGS "")
set(CMAKE_REQUIRED_INCLUDES ${YAMLCPP_INCLUDE_DIRS})
set(CMAKE_REQUIRED_LIBRARIES ${YAMLCPP_LIBRARIES})
check_cxx_source_compiles(
  "
  #include <yaml-cpp/yaml.h>
  int main()
  {
    YAML::Node node;
    node.Mark();
  }
  "
  YAMLCPP_NODE_HAS_MARK)

#==============================================================================
# Libraries
#
set(sources
  CatkinResourceRetriever.cpp
  KinBodyParser.cpp
)

add_library("${PROJECT_NAME}_io" SHARED ${sources})
target_include_directories("${PROJECT_NAME}_io" SYSTEM
  PUBLIC
    ${Boost_INCLUDE_DIRS}
    ${DART_INCLUDE_DIRS}
    ${YAMLCPP_INCLUDE_DIRS}
  PRIVATE
    ${TinyXML2_INCLUDE_DIRS}
)
target_link_libraries("${PROJECT_NAME}_io"
  PUBLIC
    "${PROJECT_NAME}_common"
    ${Boost_FILESYSTEM_LIBRARY}
    ${DART_LIBRARIES}
    ${YAMLCPP_LIBRARIES}
  PRIVATE
    ${TinyXML2_LIBRARIES}
)
target_compile_options("${PROJECT_NAME}_io"
  PUBLIC ${AIKIDO_CXX_STANDARD_FLAGS}
)
if(YAMLCPP_NODE_HAS_MARK)
  target_compile_definitions("${PROJECT_NAME}_io"
    PUBLIC YAMLCPP_NODE_HAS_MARK)
endif()

# Define TINYXML2_MAJOR_VERSION_GE_6 to support API breaking changes of
# tinyxml2 6
if (NOT TinyXML2_VERSION VERSION_LESS "6.0.0")
  target_compile_definitions("${PROJECT_NAME}_io"
    PRIVATE "TINYXML2_MAJOR_VERSION_GE_6")
endif()

add_component(${PROJECT_NAME} io)
add_component_targets(${PROJECT_NAME} io "${PROJECT_NAME}_io")
add_component_dependencies(${PROJECT_NAME} io common)

format_add_sources(${sources})
