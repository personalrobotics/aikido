#==============================================================================
# Dependencies
#
find_package(roscpp QUIET)
aikido_check_package(roscpp "aikido::sensor" "roscpp")


#find_package(catkin REQUIRED)
find_package(eyerectoo_stream)
#catkin_package(CATKIN_DEPENDS eyerectoo_stream)
#add_message_files(FILES Journal.msg)

#==============================================================================
# Libraries
#
set(sources
  EyeTracker.cpp
)

add_library("${PROJECT_NAME}_sensor" SHARED ${sources})
target_include_directories("${PROJECT_NAME}_sensor" SYSTEM
  PUBLIC
    ${roscpp_INCLUDE_DIRS}
)

target_link_libraries("${PROJECT_NAME}_sensor"
  PUBLIC
  ${DART_LIBRARIES}
  ${roscpp_LIBRARIES}
)

add_component(${PROJECT_NAME} sensor)
add_component_targets(${PROJECT_NAME} sensor "${PROJECT_NAME}_sensor")

format_add_sources(${sources})

target_compile_options("${PROJECT_NAME}_sensor"
  PUBLIC ${AIKIDO_CXX_STANDARD_FLAGS}
)
