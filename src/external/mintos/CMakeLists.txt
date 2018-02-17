set (CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

add_library("${PROJECT_NAME}_external_mintos" STATIC
  include/mintos/math/math.h
  include/mintos/math/function.h
  Timer.cpp
  src/AABB.cpp
  src/ASCIIShade.cpp
  src/ConstrainedInterpolator.cpp
  src/ConvexHull2D.cpp
  src/DiagonalMatrix.cpp
  src/GLPKInterface.cpp
  src/GeneralizedBezierCurve.cpp
  src/Householder.cpp
  src/LDL.cpp
  src/LSQRInterface.cpp
  src/LinearProgram.cpp
  src/MatrixPrinter.cpp
  src/MatrixTemplate.cpp
  src/Mintos.cpp
  src/Newton.cpp
  src/PiecewisePolynomial.cpp
  src/Plane2D.cpp
  src/PolytopeProjection.cpp
  src/QRDecomposition.cpp
  src/SVDecomposition.cpp
  src/SignalHandler.cpp
  src/SparseMatrixTemplate.cpp
  src/SparseVectorTemplate.cpp
  src/SplineInterpolate.cpp
  src/TimeScaling.cpp
  src/UnboundedPolytope2D.cpp
  src/VectorPrinter.cpp
  src/VectorTemplate.cpp
  src/backsubstitute.cpp
  src/complex.cpp
  src/differentiation.cpp
  src/function.cpp
  src/infnan.cpp
  src/lsqr.cpp
  src/metric.cpp
  src/misc.cpp
  src/primitives.cpp
  src/stringutils.cpp
)
target_compile_definitions("${PROJECT_NAME}_external_mintos"
  PRIVATE
    -DHAVE_DECL_ISINF
    -DHAVE_DECL_ISNAN
    -DHAVE_DECL_ISFINITE
    -DHAVE_GLPK=1
)
target_include_directories("${PROJECT_NAME}_external_mintos"
  PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries("${PROJECT_NAME}_external_mintos"
  PRIVATE "${PROJECT_NAME}_external_glpk")
set_target_properties("${PROJECT_NAME}_external_mintos" PROPERTIES
  POSITION_INDEPENDENT_CODE TRUE)

# Our build of GLPK must occur BEFORE any system include paths to override the
# any system version of GLPK, which is likely too recent for MinTOS.
target_include_directories("${PROJECT_NAME}_external_mintos" BEFORE
  PRIVATE ${glpk_EXTERNAL_INCLUDE_DIRS})

target_compile_options("${PROJECT_NAME}_external_mintos"
  PUBLIC ${AIKIDO_CXX_STANDARD_FLAGS}
  PRIVATE -w
)

message(STATUS "INCLUDE_DIR = ${CMAKE_CURRENT_SOURCE_DIR}/include")
