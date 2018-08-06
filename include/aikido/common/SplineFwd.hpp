// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 20010-2011 Hauke Heibel <hauke.heibel@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef AIKIDO_COMMON_SPLINEFWD_HPP_
#define AIKIDO_COMMON_SPLINEFWD_HPP_

#include <Eigen/Core>

namespace aikido {
namespace common {

template <typename Scalar, int Dim, int Degree = Eigen::Dynamic>
class BSpline;

template <typename SplineType, int DerivativeOrder = Eigen::Dynamic>
struct SplineTraits
{
};

/**
 * \ingroup Splines_Module
 * \brief Compile-time attributes of the Spline class for Dynamic degree.
 **/
template <typename _Scalar, int _Dim, int _Degree>
struct SplineTraits<BSpline<_Scalar, _Dim, _Degree>, Eigen::Dynamic>
{
  typedef _Scalar Scalar; /*!< The spline curve's scalar type. */
  enum
  {
    Dimension = _Dim /*!< The spline curve's dimension. */
  };
  enum
  {
    Degree = _Degree /*!< The spline curve's degree. */
  };

  enum
  {
    OrderAtCompileTime
    = _Degree == Eigen::Dynamic
          ? Eigen::Dynamic
          : _Degree + 1 /*!< The spline curve's order at compile-time. */
  };
  enum
  {
    NumOfDerivativesAtCompileTime = OrderAtCompileTime /*!< The number of
                                                          derivatives defined
                                                          for the current
                                                          spline. */
  };

  enum
  {
    DerivativeMemoryLayout
    = Dimension == 1
          ? Eigen::RowMajor
          : Eigen::ColMajor /*!< The derivative type's memory layout. */
  };

  /** \brief The data type used to store non-zero basis functions. */
  typedef Eigen::Array<Scalar, 1, OrderAtCompileTime> BasisVectorType;

  /** \brief The data type used to store the values of the basis function
   * derivatives. */
  typedef Eigen::Array<Scalar,
                       Eigen::Dynamic,
                       Eigen::Dynamic,
                       Eigen::RowMajor,
                       NumOfDerivativesAtCompileTime,
                       OrderAtCompileTime>
      BasisDerivativeType;

  /** \brief The data type used to store the spline's derivative values. */
  typedef Eigen::Array<Scalar,
                       Dimension,
                       Eigen::Dynamic,
                       DerivativeMemoryLayout,
                       Dimension,
                       NumOfDerivativesAtCompileTime>
      DerivativeType;

  /** \brief The point type the spline is representing. */
  typedef Eigen::Array<Scalar, Dimension, 1> PointType;

  /** \brief The data type used to store knot vectors. */
  typedef Eigen::Array<Scalar, 1, Eigen::Dynamic> KnotVectorType;

  /** \brief The data type used to store parameter vectors. */
  typedef Eigen::Array<Scalar, 1, Eigen::Dynamic> ParameterVectorType;

  /** \brief The data type representing the spline's control points. */
  typedef Eigen::Array<Scalar, Dimension, Eigen::Dynamic>
      ControlPointVectorType;
};

/**
 * \ingroup Splines_Module
 * \brief Compile-time attributes of the Spline class for fixed degree.
 *
 * The traits class inherits all attributes from the SplineTraits of Dynamic
 *degree.
 **/
template <typename _Scalar, int _Dim, int _Degree, int _DerivativeOrder>
struct SplineTraits<BSpline<_Scalar, _Dim, _Degree>, _DerivativeOrder>
    : public SplineTraits<BSpline<_Scalar, _Dim, _Degree> >
{
  enum
  {
    OrderAtCompileTime
    = _Degree == Eigen::Dynamic
          ? Eigen::Dynamic
          : _Degree + 1 /*!< The spline curve's order at compile-time. */
  };
  enum
  {
    NumOfDerivativesAtCompileTime
    = _DerivativeOrder == Eigen::Dynamic
          ? Eigen::Dynamic
          : _DerivativeOrder + 1 /*!< The number of
                                    derivatives defined for
                                    the current spline. */
  };

  enum
  {
    DerivativeMemoryLayout
    = _Dim == 1 ? Eigen::RowMajor
                : Eigen::ColMajor /*!< The derivative type's memory layout. */
  };

  /** \brief The data type used to store the values of the basis function
   * derivatives. */
  typedef Eigen::Array<_Scalar,
                       Eigen::Dynamic,
                       Eigen::Dynamic,
                       Eigen::RowMajor,
                       NumOfDerivativesAtCompileTime,
                       OrderAtCompileTime>
      BasisDerivativeType;

  /** \brief The data type used to store the spline's derivative values. */
  typedef Eigen::Array<_Scalar,
                       _Dim,
                       Eigen::Dynamic,
                       DerivativeMemoryLayout,
                       _Dim,
                       NumOfDerivativesAtCompileTime>
      DerivativeType;
};

/** \brief 2D float B-spline with dynamic degree. */
typedef BSpline<float, 2> BSpline2f;

/** \brief 3D float B-spline with dynamic degree. */
typedef BSpline<float, 3> BSpline3f;

/** \brief 2D double B-spline with dynamic degree. */
typedef BSpline<double, 2> BSpline2d;

/** \brief 3D double B-spline with dynamic degree. */
typedef BSpline<double, 3> BSpline3d;

} // namespace common
} // namespace aikido

#endif // AIKIDO_COMMON_SPLINEFWD_HPP_
