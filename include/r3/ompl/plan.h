#ifndef R3_OMPL_PLAN_H_
#define R3_OMPL_PLAN_H_
#include <memory>
#include <vector>
#include <Eigen/Dense>
#include <dart/collision/collision.h>
#include <dart/dynamics/dynamics.h>

namespace dart {
namespace simulation {

class World;

} // namespace simulation
} // namespace dart

namespace r3 {
namespace ompl {

::Eigen::MatrixXd Plan(
    ::std::shared_ptr< ::dart::collision::CollisionDetector> const &collision_detector,
    ::std::vector<::dart::dynamics::DegreeOfFreedomPtr> const &dofs,
    ::Eigen::VectorXd const &dof_weights,
    ::Eigen::VectorXd const &dof_resolutions,
    ::Eigen::MatrixXd const &start_configs,
    ::Eigen::VectorXd const &goal_config
);

} // namespace ompl
} // namespace r3

#endif
