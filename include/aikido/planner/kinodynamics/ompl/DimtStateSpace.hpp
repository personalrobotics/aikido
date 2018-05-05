#ifndef OMPL_BASE_SPACES_DIMT_STATE_SPACE_
#define OMPL_BASE_SPACES_DIMT_STATE_SPACE_

#include <Eigen/Dense>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <aikido/planner/kinodynamics/dimt/DoubleIntegratorMinimumTime.h>
#include <aikido/planner/kinodynamics/dimt/Params.h>

namespace ompl {
namespace base {
class DimtStateSpace : public RealVectorStateSpace
{
private:
  std::shared_ptr<DIMT> dimt_;
  mutable Eigen::VectorXd eig_from, eig_to;

public:
  DimtStateSpace(const std::shared_ptr<DIMT> dimt)
    : RealVectorStateSpace(param.dimensions)
    , eig_from(param.dimensions)
    , eig_to(param.dimensions)
  {
    dimt_ = dimt;
  }

  virtual double distance(const State* state1, const State* state2) const
  {
    double time = dimt_->getMinTime(state1, state2);
    return time;
  }

  virtual void interpolate(
      const State* from, const State* to, const double t, State* state) const
  {
    double minTime = dimt_->getMinTime(from, to);
    dimt_->interpolate(from, to, t * minTime, state);
    /*
    // TODO: Fix assert fail in steering function (using straight line for now)
    for (unsigned int i = 0; i < dimension_; ++i)
        rstate->values[i] = rfrom->values[i] + (rto->values[i] -
    rfrom->values[i]) * t;
    */
  }

  virtual bool isMetricSpace() const
  {
    return false;
  }
};

using DimtStateSpacePtr = std::shared_ptr<DimtStateSpace>;
}
}
#endif
