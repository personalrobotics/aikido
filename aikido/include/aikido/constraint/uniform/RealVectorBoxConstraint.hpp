#ifndef AIKIDO_STATESPACE_REALVECTORSTATESPACESAMPLEABLECONSTRAINT_H_
#define AIKIDO_STATESPACE_REALVECTORSTATESPACESAMPLEABLECONSTRAINT_H_
#include "../../statespace/RealVectorStateSpace.hpp"
#include "../Sampleable.hpp"

namespace aikido {
namespace statespace {

class RealVectorBoxConstraint
  : public constraint::SampleableConstraint
{
public:
  RealVectorBoxConstraint(
    std::shared_ptr<statespace::RealVectorStateSpace> _space,
    std::unique_ptr<util::RNG> _rng,
    const Eigen::VectorXd& _lowerLimits,
    const Eigen::VectorXd& _upperLimits);

  // Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

  // Documentation inherited.
  std::unique_ptr<constraint::SampleGenerator>
    createSampleGenerator() const override;

private:
  std::shared_ptr<statespace::RealVectorStateSpace> mSpace;
  std::unique_ptr<util::RNG> mRng;
  Eigen::VectorXd mLowerLimits;
  Eigen::VectorXd mUpperLimits;
};


} // namespace statespace
} // namespace aikido

#endif // AIKIDO_STATESPACE_REALVECTORSTATESPACESAMPLEABLECONSTRAINT_H_
