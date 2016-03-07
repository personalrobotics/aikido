#ifndef AIKIDO_PERCEPTION_YAMLCONVERSION_H
#define AIKIDO_PERCEPTION_YAMLCONVERSION_H

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

namespace YAML {
//assume vectors for now..
    template<typename Derived>
        struct convert<Eigen::MatrixBase<Derived> > {
            static Node encode(const Eigen::MatrixBase<Derived>& rhs) {
                Node node;
                for(int i = 0; i < rhs.size(); ++i){
                    node.push_back(rhs(i));
                }
                return node;
            }

            static bool decode(const Node& node, const Eigen::MatrixBase<Derived>& rhs) {
                if(!node.IsSequence())
                    return false;
                //todo use intelligent inspection of RowsAtCompileTime etc to disambguate between fixed and dynamic matrices
                //if(!node.size() == 3)
                //    return false;
                for(int i = 0; i < rhs.size(); ++i){
                    rhs(i) = node.as<typename Eigen::MatrixBase<Derived>::Scalar>();
                }
                return true;
            }
        };
}

#endif 