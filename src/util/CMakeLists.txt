add_library("${PROJECT_NAME}_util" SHARED
  CatkinResourceRetriever.cpp
  PseudoInverse.cpp
  VanDerCorput.cpp
  StepSequence.cpp
  RNG.cpp
)
target_link_libraries("${PROJECT_NAME}_util"
  ${Boost_FILESYSTEM_LIBRARY}
  ${DART_LIBRARIES}
  ${TinyXML2_LIBRARIES}
)

add_component(util)
add_component_targets(util "${PROJECT_NAME}_util")
