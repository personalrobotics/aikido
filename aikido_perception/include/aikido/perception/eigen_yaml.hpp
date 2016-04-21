#ifndef AIKIDO_PERCEPTION_EIGEN_YAML_HPP
#define AIKIDO_PERCEPTION_EIGEN_YAML_HPP

#include <iostream>
#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>


template <class _Scalar,
          int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
inline void deserialize(
    YAML::Node const &node,
    Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> &matrix)
{
    typedef Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> MatrixType;
    typedef typename MatrixType::Index Index;
    typedef typename MatrixType::Scalar Scalar;


    using boost::format;
    using boost::str;
    using std::runtime_error;

    if (node.Type() != YAML::NodeType::Sequence) {
        throw runtime_error("Matrix or vector must be a sequence.");
    }

    Index const rows = node.size();
    if (MatrixType::RowsAtCompileTime != Eigen::Dynamic 
     && rows != MatrixType::RowsAtCompileTime) {
        throw runtime_error(str(
            format("Matrix has incorrect number of rows: expected %d; got %d.")
            % MatrixType::RowsAtCompileTime % rows));
    }

    if (node[0].Type() == YAML::NodeType::Scalar) {
        matrix.resize(rows, 1);

        for (Index i = 0; i < rows; ++i) {
#ifdef YAMLCPP_NEWAPI
            matrix(i, 0) = node[i].template as< _Scalar >();
#else
            node[i] >> matrix(i, 0);
#endif
        }
    } else if (node[0].Type() == YAML::NodeType::Sequence) {
        Index const cols = node[0].size();

        if (MatrixType::ColsAtCompileTime != Eigen::Dynamic 
         && cols != MatrixType::ColsAtCompileTime) {
            throw runtime_error(str(
                format("Matrix has incorrect number of cols: expected %d; got %d.")
                % MatrixType::ColsAtCompileTime % cols));
        }

        matrix.resize(rows, cols);

        for (Index r = 0; r < node.size(); ++r) {
            if (node[r].Type() != YAML::NodeType::Sequence) {
                throw runtime_error(str(
                    format("Row %d of the matrix must be a sequence.") % r));
            } else if (node[r].size() != cols) {
                throw runtime_error(boost::str(
                    format("Expected row %d to have %d columns; got %d.")
                        % r % cols % node[r].size()));
            }

            for (Index c = 0; c < cols; ++c) {
#ifdef YAMLCPP_NEWAPI
                matrix(r, c) = node[r][c].template as<_Scalar>();
#else
                node[r][c] >> matrix(r, c);
#endif
            }
        }
    } else {
        throw runtime_error(str(
            format("Unknown type of matrix ")));
    }
}

template <class _Scalar, int Dim, int Mode, int _Options>
inline void deserialize(YAML::Node const &node,
                        Eigen::Transform<_Scalar, Dim, Mode, _Options> &pose)
{
    deserialize(node, pose.matrix());
}

template <class T>
inline bool has_child(YAML::Node const &node, T const &key)
{
#ifdef YAMLCPP_NEWAPI
    return node[key].IsDefined();
#else // ifdef YAMLCPP_NEWAPI
    return !!node.FindValue(key);
#endif // ifndef YAMLCPP_NEWAPI
}

namespace YAML {

#ifdef YAMLCPP_NEWAPI

namespace detail {

template <typename MatrixType, bool IsVectorAtCompileTime>
struct encode_impl {
  static Node encode(MatrixType const &matrix)
  {
    assert(false && "Unknown MatrixType.");
  }
};

template <typename MatrixType>
struct encode_impl<MatrixType, true> {
  static Node encode(MatrixType const &matrix)
  {
    typedef typename MatrixType::Index Index;

    Node node;
    node.SetTag("Vector");

    for (Index i = 0; i < matrix.size(); ++i) {
      node.push_back(Node(matrix[i]));
    }
    return node;
  }
};

template <typename MatrixType>
struct encode_impl<MatrixType, false> {
  static Node encode(MatrixType const &matrix)
  {
    typedef typename MatrixType::Index Index;

    Node node;
    node.SetTag("Matrix");

    for (Index r = 0; r < matrix.rows(); ++r) {
      Node row(NodeType::Sequence);

      for (Index c = 0; c < matrix.cols(); ++c) {
          row.push_back(matrix(r, c));
      }

      node.push_back(row);
    }
    return node;
  }
};

}

template <typename _Scalar, int _Dim, int _Mode, int _Options>
struct convert<Eigen::Matrix<_Scalar, _Dim, _Mode, _Options> > {
    typedef Eigen::Matrix<_Scalar, _Dim, _Mode, _Options> MatrixType;

    static Node encode(MatrixType const &matrix)
    {
        YAML::Node node(NodeType::Sequence);
        return detail::encode_impl<
          MatrixType, MatrixType::IsVectorAtCompileTime>::encode(matrix);
    }

    static bool decode(
        YAML::Node const &node,
        Eigen::Matrix<_Scalar, _Dim, _Mode, _Options> &matrix)
    {
        if (node[0].Type() == YAML::NodeType::Scalar || node[0].Type() == YAML::NodeType::Sequence) {
            deserialize(node, matrix);
            return true;
        } else {
            return false;
        }
    }
};

template <typename _Scalar, int _Dim, int _Mode, int _Options>
struct convert<Eigen::Transform<_Scalar, _Dim, _Mode, _Options> > {
    typedef Eigen::Transform<_Scalar, _Dim, _Mode, _Options> TransformType;
    typedef typename TransformType::MatrixType MatrixType;

    static Node encode(TransformType const &transform)
    {
        return convert<MatrixType>::encode(transform.matrix());
    }

    static bool decode(Node const &node, TransformType &transform)
    {
        return convert<MatrixType>::decode(node, transform.matrix());
    }
};

template <class T>
inline void operator>>(Node const &node, T &value)
{
    value = node.as<T>();
}

#else // ifdef YAMLCPP_NEWAPI

template <class _Scalar,
          int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
inline Emitter &operator<<(
    Emitter &emitter,
    Eigen::Matrix<_Scalar,
        _Rows, _Cols, _Options, _MaxRows, _MaxCols> const &matrix)
{
    typedef Eigen::Matrix<_Scalar, _Rows, _Cols, _Options,
                          _MaxRows, _MaxCols> MatrixType;
    typedef typename MatrixType::Index Index;

    if (MatrixType::IsVectorAtCompileTime) {
        emitter << LocalTag("Vector")
                << Flow
                << BeginSeq;

        for (Index i = 0; i < matrix.size(); ++i) {
            emitter << matrix(i, 0);
        }

        emitter << EndSeq;
    } else {
        emitter << LocalTag("Matrix")
                << BeginSeq;

        for (Index r = 0; r < matrix.rows(); ++r) {
            emitter << Flow
                    << BeginSeq;
            for (Index c = 0; c < matrix.cols(); ++c) {
                emitter << matrix(r, c);
            }

            emitter << EndSeq;
        }

        emitter << EndSeq;
    }
    return emitter;
}

template <class _Scalar, int Dim, int Mode, int _Options>
inline Emitter &operator<<(Emitter &emitter,
                           Eigen::Transform<_Scalar, Dim, Mode, _Options> &tf)
{
    return emitter << tf.matrix();
}

template <class _Scalar, int Dim, int Mode, int _Options>
inline void operator>>(
    Node const &node,
    Eigen::Transform<_Scalar, Dim, Mode, _Options> &tf)
{
    deserialize(node, tf.matrix());
}

template <class _Scalar,
          int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
inline void operator>>(
    Node const &node,
    Eigen::Matrix<_Scalar,
        _Rows, _Cols, _Options, _MaxRows, _MaxCols> &matrix)
{
    deserialize(node, matrix);
}

#endif // else ifdef YAMLCPP_NEWAPI

} // namespace YAML

#endif // ifndef YAML_UTILS_H_