#include <aikido/planner/VectorFieldPlanner.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

aikido::trajectory::InterpolatedPtr planPathByVectorField(
  const dart::dynamics::MetaSkeletonPtr skeleton,
  double t,
  const VectorFieldCallback& vectorFiledCb,
  const VectorFieldStatusCallback& statusCb)
{
  return nullptr;
}

aikido::trajectory::InterpolatedPtr planStrightLine(
  const dart::dynamics::MetaSkeletonPtr skeleton,
  const Eigen::VectorXd& startPosition,
  const Eigen::VectorXd& goalPosition)
{

}

} // namespace vectorfield
} // namespace planner
} // namespace aikido


