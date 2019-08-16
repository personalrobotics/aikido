set(sources
  BSpline.cpp
  Interpolated.cpp
  Spline.cpp
  util.cpp
)

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

add_library("${PROJECT_NAME}_trajectory" SHARED ${sources})
target_include_directories("${PROJECT_NAME}_trajectory" SYSTEM
  PUBLIC
    ${YAMLCPP_INCLUDE_DIRS}
)
target_link_libraries("${PROJECT_NAME}_trajectory"
  PUBLIC
    "${PROJECT_NAME}_common"
    "${PROJECT_NAME}_statespace"
    ${YAMLCPP_LIBRARIES}
)
target_compile_options("${PROJECT_NAME}_trajectory"
  PUBLIC ${AIKIDO_CXX_STANDARD_FLAGS}
)
if(YAMLCPP_NODE_HAS_MARK)
  target_compile_definitions("${PROJECT_NAME}_io"
    PUBLIC YAMLCPP_NODE_HAS_MARK)
endif()

add_component(${PROJECT_NAME} trajectory)
add_component_targets(${PROJECT_NAME} trajectory "${PROJECT_NAME}_trajectory")
add_component_dependencies(${PROJECT_NAME} trajectory common statespace)

clang_format_add_sources(${sources})
