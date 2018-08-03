find_package(EIGEN3 REQUIRED)

add_library("${PROJECT_NAME}_external_kunzretimer" STATIC
  Path.cpp
  Trajectory.cpp
)

target_include_directories("${PROJECT_NAME}_external_kunzretimer"
  PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}"
          ${EIGEN3_INCLUDE_DIRS}
)
set_target_properties("${PROJECT_NAME}_external_kunzretimer"
  PROPERTIES POSITION_INDEPENDENT_CODE TRUE
)
target_compile_options("${PROJECT_NAME}_external_kunzretimer"
  PUBLIC ${AIKIDO_CXX_STANDARD_FLAGS}
  PRIVATE -w
)
