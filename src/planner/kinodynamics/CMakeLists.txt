set(sources 
  KinodynamicPlanner.cpp
  sampler/Sampler.cpp
  sampler/MonteCarloSampler.cpp
  sampler/RejectionSampler.cpp
  sampler/HitAndRunSampler.cpp
  ompl/MyOptimizationObjective.cpp
  ompl/MyInformedRRTstar.cpp
  ompl/MyBITstar.cpp
  ompl/OmplHelpers.cpp
  ompl/ValidityChecker.cpp
  dimt/DoubleIntegrator.h
  dimt/DoubleIntegratorMinimumTime.cpp
)

add_library("${PROJECT_NAME}_planner_kinodynamics" SHARED ${sources})
target_link_libraries("${PROJECT_NAME}_planner_kinodynamics"
  PUBLIC
    "${PROJECT_NAME}_common"
    "${PROJECT_NAME}_constraint"
    "${PROJECT_NAME}_distance"
    "${PROJECT_NAME}_trajectory"
    "${PROJECT_NAME}_statespace"
    "${PROJECT_NAME}_planner"
    ${OMPL_LIBRARIES}
    ${NLOPT_LIBRARIES}
    ${DART_LIBRARIES}
    ${Boost_LIBRARIES}
)
target_include_directories("${PROJECT_NAME}_planner_kinodynamics" SYSTEM
  PUBLIC
    ${OMPL_INCLUDE_DIRS}
    ${NLOPT_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${DART_INCLUDE_DIRS}
)
target_compile_options("${PROJECT_NAME}_planner_kinodynamics"
  PUBLIC ${AIKIDO_CXX_STANDARD_FLAGS}
)

