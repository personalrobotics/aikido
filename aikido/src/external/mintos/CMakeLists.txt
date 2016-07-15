# Force our build of GLPK to override the system version, which is likely too
# new to work with MinTOS.
include_directories(BEFORE SYSTEM ${glpk_INCLUDE_DIRS})

include_directories("include")

add_definitions(
  -DHAVE_DECL_ISINF
  -DHAVE_DECL_ISNAN
  -DHAVE_DECL_ISFINITE
  -DHAVE_GLPK=1
)

aikido_add_library("${PROJECT_NAME}_mintos" EXTERNAL
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
target_link_libraries("${PROJECT_NAME}_mintos"
  ${glpk_LIBRARIES}
)

set(mintos_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/include"
  CACHE PATH "MinTOS include directories")
set(mintos_LIBRARIES "${PROJECT_NAME}_mintos"
  CACHE STRING "MinTOS libraries")
