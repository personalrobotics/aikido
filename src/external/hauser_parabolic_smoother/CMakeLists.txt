add_library("${PROJECT_NAME}_external_hauserparabolicsmoother" STATIC
  DynamicPath.cpp
  ParabolicRamp.cpp
  Timer.cpp
)
target_include_directories("${PROJECT_NAME}_external_hauserparabolicsmoother"
  PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}"
)
set_target_properties("${PROJECT_NAME}_external_hauserparabolicsmoother"
  PROPERTIES POSITION_INDEPENDENT_CODE TRUE
)
target_compile_options("${PROJECT_NAME}_external_hauserparabolicsmoother"
  PUBLIC ${AIKIDO_CXX_STANDARD_FLAGS}
)
