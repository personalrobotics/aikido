#ifndef R3_OMPL_PLAN_H_
#define R3_OMPL_PLAN_H_
#include <vector>
#include <Eigen/Dense>
#include <dart/dynamics/dynamics.h>

namespace dart {
namespace simulation {

class World;

} // namespace simulation
} // namespace dart

namespace r3 {
namespace ompl {

::Eigen::MatrixXd Plan(
    ::dart::simulation::World *world,
    ::std::vector<::dart::dynamics::DegreeOfFreedom *> const &dofs,
    ::Eigen::VectorXd const &dof_weights,
    ::Eigen::VectorXd const &dof_resolutions,
    ::Eigen::MatrixXd const &start_configs,
    ::Eigen::VectorXd const &goal_config
);

} // namespace ompl
} // namespace r3

#endif
