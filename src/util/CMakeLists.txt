aikido_add_component(util REQUIRED)
aikido_add_library("${PROJECT_NAME}_util" COMPONENT util
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
