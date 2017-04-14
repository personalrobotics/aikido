#ifndef AIKIDO_PERCEPTION_EIGEN_YAML_HPP_
#define AIKIDO_PERCEPTION_EIGEN_YAML_HPP_

#include <iostream>
#include <Eigen/Dense>
#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <yaml-cpp/yaml.h>

template <class _Scalar,
          int _Rows,
          int _Cols,
          int _Options,
          int _MaxRows,
          int _MaxCols>
inline void deserialize(
    YAML::Node const& node,
    Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& matrix)
{
  typedef Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>
      MatrixType;
  typedef typename MatrixType::Index Index;

  using boost::format;
  using boost::str;
  using std::runtime_error;

  if (node.Type() != YAML::NodeType::Sequence)
  {
    throw runtime_error("Matrix or vector must be a sequence.");
  }

  Index const rows = node.size();
  if (MatrixType::RowsAtCompileTime != Eigen::Dynamic
      && rows != MatrixType::RowsAtCompileTime)
  {
    throw runtime_error(
        str(format("Matrix has incorrect number of rows: expected %d; got %d.")
            % MatrixType::RowsAtCompileTime
            % rows));
  }

  if (node[0].Type() == YAML::NodeType::Scalar)
  {
    matrix.resize(rows, 1);

    for (Index i = 0; i < rows; ++i)
    {
      matrix(i, 0) = node[i].template as<_Scalar>();
    }
  }
  else if (node[0].Type() == YAML::NodeType::Sequence)
  {
    Index const cols = node[0].size();

    if (MatrixType::ColsAtCompileTime != Eigen::Dynamic
        && cols != MatrixType::ColsAtCompileTime)
    {
      throw runtime_error(
          str(format(
                  "Matrix has incorrect number of cols: expected %d; got %d.")
              % MatrixType::ColsAtCompileTime
              % cols));
    }

    matrix.resize(rows, cols);

    for (Index r = 0; r < node.size(); ++r)
    {
      if (node[r].Type() != YAML::NodeType::Sequence)
      {
        throw runtime_error(
            str(format("Row %d of the matrix must be a sequence.") % r));
      }
      else if (node[r].size() != cols)
      {
        throw runtime_error(
            boost::str(
                format("Expected row %d to have %d columns; got %d.") % r % cols
                % node[r].size()));
      }

      for (Index c = 0; c < cols; ++c)
      {
        matrix(r, c) = node[r][c].template as<_Scalar>();
      }
    }
  }
  else
  {
    throw runtime_error(str(format("Unknown type of matrix ")));
  }
}

template <class _Scalar, int Dim, int Mode, int _Options>
inline void deserialize(
    YAML::Node const& node,
    Eigen::Transform<_Scalar, Dim, Mode, _Options>& pose)
{
  deserialize(node, pose.matrix());
}

template <class T>
inline bool has_child(YAML::Node const& node, T const& key)
{
  return node[key].IsDefined();
}

namespace YAML {

namespace detail {

template <typename MatrixType, bool IsVectorAtCompileTime>
struct encode_impl {};

template <typename MatrixType>
struct encode_impl<MatrixType, true>
{
  static Node encode(MatrixType const& matrix)
  {
    typedef typename MatrixType::Index Index;

    Node node;
    node.SetTag("Vector");

    for (Index i = 0; i < matrix.size(); ++i)
    {
      node.push_back(Node(matrix[i]));
    }
    return node;
  }
};

template <typename MatrixType>
struct encode_impl<MatrixType, false>
{
  static Node encode(MatrixType const& matrix)
  {
    typedef typename MatrixType::Index Index;

    Node node;
    node.SetTag("Matrix");

    for (Index r = 0; r < matrix.rows(); ++r)
    {
      Node row(NodeType::Sequence);

      for (Index c = 0; c < matrix.cols(); ++c)
      {
        row.push_back(matrix(r, c));
      }

      node.push_back(row);
    }
    return node;
  }
};

} // namespace detail

template <typename _Scalar, int _Dim, int _Mode, int _Options>
struct convert<Eigen::Matrix<_Scalar, _Dim, _Mode, _Options> >
{
  typedef Eigen::Matrix<_Scalar, _Dim, _Mode, _Options> MatrixType;

  static Node encode(MatrixType const& matrix)
  {
    YAML::Node node(NodeType::Sequence);
    return detail::encode_impl<MatrixType, MatrixType::IsVectorAtCompileTime>::
        encode(matrix);
  }

  static bool decode(
      YAML::Node const& node,
      Eigen::Matrix<_Scalar, _Dim, _Mode, _Options>& matrix)
  {
    if (node[0].Type() == YAML::NodeType::Scalar
        || node[0].Type() == YAML::NodeType::Sequence)
    {
      deserialize(node, matrix);
      return true;
    }
    else
    {
      return false;
    }
  }
};

template <typename _Scalar, int _Dim, int _Mode, int _Options>
struct convert<Eigen::Transform<_Scalar, _Dim, _Mode, _Options> >
{
  typedef Eigen::Transform<_Scalar, _Dim, _Mode, _Options> TransformType;
  typedef typename TransformType::MatrixType MatrixType;

  static Node encode(TransformType const& transform)
  {
    return convert<MatrixType>::encode(transform.matrix());
  }

  static bool decode(Node const& node, TransformType& transform)
  {
    return convert<MatrixType>::decode(node, transform.matrix());
  }
};

template <class T>
inline void operator>>(Node const& node, T& value)
{
  value = node.as<T>();
}

} // namespace YAML

#endif // AIKIDO_PERCEPTION_EIGEN_YAML_HPP_
