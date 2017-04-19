#ifndef AIKIDO_UTIL_DETAIL_YAMLEIGENEXTENSION_HPP_
#define AIKIDO_UTIL_DETAIL_YAMLEIGENEXTENSION_HPP_

#include <sstream>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

namespace aikido {
namespace util {
namespace detail {

//==============================================================================
template <class _Scalar,
          int _Rows,
          int _Cols,
          int _Options,
          int _MaxRows,
          int _MaxCols>
void deserialize(
    const YAML::Node& node,
    Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& matrix)
{
  using MatrixType
      = Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>;
  using Index = typename MatrixType::Index;

  using std::runtime_error;

  if (node.Type() != YAML::NodeType::Sequence)
    throw runtime_error("Matrix or vector must be a sequence.");

  Index const rows = node.size();
  if (MatrixType::RowsAtCompileTime != Eigen::Dynamic
      && rows != MatrixType::RowsAtCompileTime)
  {
    std::stringstream ss;
    ss << "Matrix has incorrect number of rows: expected "
       << MatrixType::RowsAtCompileTime << "; got " << rows << ".";
    throw runtime_error(ss.str());
  }

  if (node.Tag() == "Vector" || node.Tag() == "!Vector")
  {
    matrix.resize(rows, 1);
    for (Index i = 0; i < rows; ++i)
      matrix(i, 0) = node[i].template as<_Scalar>();
  }
  else if (node.Tag() == "Matrix" || node.Tag() == "!Matrix")
  {
    Index const cols = node[0].size();

    if (MatrixType::ColsAtCompileTime != Eigen::Dynamic
        && cols != MatrixType::ColsAtCompileTime)
    {
      std::stringstream ss;
      ss << "Matrix has incorrect number of cols: expected "
         << MatrixType::ColsAtCompileTime << "; got " << cols << ".";
      throw runtime_error(ss.str());
    }

    matrix.resize(rows, cols);

    for (Index r = 0; r < node.size(); ++r)
    {
      if (node[r].Type() != YAML::NodeType::Sequence)
      {
        std::stringstream ss;
        ss << "Row " << r << " of the matrix must be a sequence.";
        throw runtime_error(ss.str());
      }
      else if (node[r].size() != cols)
      {
        std::stringstream ss;
        ss << "Expected row " << r << " to have " << cols << " columns; got "
           << node[r].size() << ".";
        throw runtime_error(ss.str());
      }

      for (Index c = 0; c < cols; ++c)
        matrix(r, c) = node[r][c].template as<_Scalar>();
    }
  }
  else
  {
    std::stringstream ss;
    ss << "Unknown type of matrix '" << node.Tag() << "'.";
    throw runtime_error(ss.str());
  }
}

//==============================================================================
template <class _Scalar, int Dim, int Mode, int _Options>
void deserialize(
    const YAML::Node& node,
    Eigen::Transform<_Scalar, Dim, Mode, _Options>& pose)
{
  deserialize(node, pose.matrix());
}

} // namespace detail
} // namespace util
} // namespace aikido

namespace YAML {
namespace detail {

//==============================================================================
template <typename MatrixType, bool IsVectorAtCompileTime>
struct encode_impl
{
  // Nothing defined. This class should be always specialized.
};

//==============================================================================
template <typename MatrixType>
struct encode_impl<MatrixType, true>
{
  static Node encode(const MatrixType& matrix)
  {
    using Index = typename MatrixType::Index;

    Node node;
    node.SetTag("Vector");

    for (Index i = 0; i < matrix.size(); ++i)
      node.push_back(Node(matrix[i]));

    return node;
  }
};

//==============================================================================
template <typename MatrixType>
struct encode_impl<MatrixType, false>
{
  static Node encode(const MatrixType& matrix)
  {
    using Index = typename MatrixType::Index;

    Node node;
    node.SetTag("Matrix");

    for (Index r = 0; r < matrix.rows(); ++r)
    {
      Node row(NodeType::Sequence);

      for (Index c = 0; c < matrix.cols(); ++c)
        row.push_back(matrix(r, c));

      node.push_back(row);
    }

    return node;
  }
};

} // namespace detail

//==============================================================================
template <typename _Scalar, int _Dim, int _Mode, int _Options>
struct convert<Eigen::Matrix<_Scalar, _Dim, _Mode, _Options> >
{
  using MatrixType = Eigen::Matrix<_Scalar, _Dim, _Mode, _Options>;

  static Node encode(const MatrixType& matrix)
  {
    return detail::encode_impl<MatrixType, MatrixType::IsVectorAtCompileTime>::
        encode(matrix);
  }

  static bool decode(
      const YAML::Node& node,
      Eigen::Matrix<_Scalar, _Dim, _Mode, _Options>& matrix)
  {
    aikido::util::detail::deserialize(node, matrix);

    if (node.Tag() == "Vector" || node.Tag() == "!Vector"
        || node.Tag() == "Matrix"
        || node.Tag() == "!Matrix")
    {
      aikido::util::detail::deserialize(node, matrix);
      return true;
    }
    else
    {
      return false;
    }
  }
};

//==============================================================================
template <typename _Scalar, int _Dim, int _Mode, int _Options>
struct convert<Eigen::Transform<_Scalar, _Dim, _Mode, _Options> >
{
  using TransformType = Eigen::Transform<_Scalar, _Dim, _Mode, _Options>;
  using MatrixType = typename TransformType::MatrixType;

  static Node encode(const TransformType& transform)
  {
    return convert<MatrixType>::encode(transform.matrix());
  }

  static bool decode(const Node& node, TransformType& transform)
  {
    return convert<MatrixType>::decode(node, transform.matrix());
  }
};

} // namespace YAML

#endif // AIKIDO_UTIL_YAMLUTILS_HPP_
