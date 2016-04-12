#ifndef AIKIDO_PERCEPTION_EIGEN_YAML_HPP
#define AIKIDO_PERCEPTION_EIGEN_YAML_HPP
#include <Eigen/Sparse>
#include <yaml-cpp/yaml.h>

namespace YAML {

template <typename _Scalar, int _Flags, typename _Index>
struct convert<Eigen::SparseVector<_Scalar, _Flags, _Index> >
{
  using Type = Eigen::SparseVector<_Scalar, _Flags, _Index>;

  static Node encode(const Type &x)
  {
    Node node(NodeType::Map);
    node.SetTag("SparseVector");
    node["size"] = x.size();

    for (typename Type::InnerIterator it(x); it; ++it) {
      node["indices"].push_back(it.index());
      node["values"].push_back(it.value());
    }

    return node;
  }

  static bool decode(const Node &node, Type &x)
  {
    if (!node.IsMap() || node.Tag() != "SparseVector")
      return false;

    const Node &size_node = node["size"];
    const _Index size = size_node.as<_Index>();

    const Node &index_nodes = node["indices"];
    if (!index_nodes || !index_nodes.IsSequence())
      return false;

    const Node &value_nodes = node["values"];
    if (!value_nodes || !value_nodes.IsSequence())
      return false;

    if (index_nodes.size() != value_nodes.size())
      return false;

    x = Type(size);

    for (_Index i = 0; i < index_nodes.size(); ++i) {
      const _Index index = index_nodes[i].template as<_Index>();
      const _Scalar value = value_nodes[i].template as<_Scalar>();
      x.coeffRef(index) = value;
    }

    return true;
  }
};

} // namespace YAML

#endif // ifndef AIKIDO_PERCEPTION_EIGEN_YAML_HPP