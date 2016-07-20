add_library("${PROJECT_NAME}_external_hauserparabolicsmoother" STATIC
  DynamicPath.cpp
  ParabolicRamp.cpp
  Timer.cpp
)
set_target_properties("${PROJECT_NAME}_external_hauserparabolicsmoother"
  PROPERTIES POSITION_INDEPENDENT_CODE TRUE
)

set(hauserparabolicsmoother_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}"
  CACHE PATH "Hauser ParabolicSmoother include directories")
set(hauserparabolicsmoother_LIBRARIES
  "${PROJECT_NAME}_external_hauserparabolicsmoother"
  CACHE STRING "Hauser ParabolicSmoother libraries")

