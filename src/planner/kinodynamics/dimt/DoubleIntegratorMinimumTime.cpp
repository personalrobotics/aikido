#include "DoubleIntegrator.h"
#include "aikido/planner/kinodynamics/dimt/DoubleIntegratorMinimumTime.h"

class DoubleIntegratorTobiasImpl : public DoubleIntegratorImpl
{
  using DI = DoubleIntegrator<param.dof>;

public:
  DI::StateVector newX1;
  DI::StateVector newX2;
  DI::StateVector newX;

  DoubleIntegratorTobiasImpl(
      std::vector<double>& maxAccelerations, std::vector<double>& maxVelocities)
    : maxAccelerations_(maxAccelerations), maxVelocities_(maxVelocities)
  {
    DI::Vector maxA;
    DI::Vector maxV;
    for (int i = 0; i < param.dof; ++i)
    {
      maxA[i] = maxAccelerations_[i];
      maxV[i] = maxVelocities_[i];
    }
    doubleIntegrator_ = std::make_shared<DI>(maxA, maxV);
  }

  void convertToTobiasFormat(const ompl::base::State* x, DI::StateVector& newX)
  {
    /*
    // convert [x1, v1, x2, v2] to [x1, x2, v1, v2]
    for(int i=0;i<param.dof;i++)
    {
        newX[i] =
    x->as<ompl::base::RealVectorStateSpace::StateType>()->values[2*i];
        newX[param.dof+i] =
    x->as<ompl::base::RealVectorStateSpace::StateType>()->values[2*i+1];
    }*/
    for (int i = 0; i < param.dimensions; i++)
    {
      newX[i] = x->as<ompl::base::RealVectorStateSpace::StateType>()->values[i];
    }

    /*
    std::cout << "COMP ";
    for(int i=0;i<param.dimensions;i++)
    {
        std::cout <<
    x->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] << " ";
        std::cout << "(" << newX[i] << ") ";
    }
    std::cout << std::endl;*/
    return;
  }

  void convertFromTobiasFormat(
      const DI::StateVector& x, ompl::base::State* newX)
  {
    /*
    // convert [x1, x2, v1, v2] to [x1, v1, x2, v2]
    for(int i=0;i<param.dof;i++)
    {
        newX->as<ompl::base::RealVectorStateSpace::StateType>()->values[2*i] =
    x[i];
        newX->as<ompl::base::RealVectorStateSpace::StateType>()->values[2*i+1] =
    x[param.dof+i];
    }*/
    for (int i = 0; i < param.dimensions; i++)
    {
      newX->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = x[i];
    }
    return;
  }

  void convertToTobiasFormatVec(const Eigen::VectorXd& x, DI::StateVector& newX)
  {
    for (int i = 0; i < param.dimensions; i++)
    {
      newX[i] = x[i];
    }
  }

  void convertFromTobiasFormatVec(
      const DI::StateVector& x, Eigen::VectorXd& newX)
  {
    for (int i = 0; i < param.dimensions; i++)
    {
      newX[i] = x[i];
    }
    return;
  }

  virtual double getMinTime(
      const ompl::base::State* x1, const ompl::base::State* x2)
  {
    convertToTobiasFormat(x1, newX1);
    convertToTobiasFormat(x2, newX2);
    return doubleIntegrator_->getMinTime(newX1, newX2);
  }

  virtual double getMinTimeIfSmallerThan(
      const ompl::base::State* x1,
      const ompl::base::State* x2,
      double timeThreshold)
  {
    convertToTobiasFormat(x1, newX1);
    convertToTobiasFormat(x2, newX2);

    return doubleIntegrator_->getMinTimeIfSmallerThan(
        newX1, newX2, timeThreshold);
  }

  virtual std::tuple<double, double, double> getMinTimeAndIntervals1Dof(
      const double x1,
      const double v1,
      const double x2,
      const double v2,
      int dof_index = 0)

  {
    double a1(0);
    double minTime = doubleIntegrator_->getMinTime1D(
        v1,
        v2,
        x2 - x1,
        maxAccelerations_[dof_index],
        maxVelocities_[dof_index],
        std::numeric_limits<double>::max(),
        a1);

    std::pair<double, double> infeasibleInterval
        = doubleIntegrator_->getInfeasibleInterval(
            v1,
            v2,
            x2 - x1,
            a1,
            maxAccelerations_[dof_index],
            maxVelocities_[dof_index]);

    return std::make_tuple(
        minTime, infeasibleInterval.first, infeasibleInterval.second);
  }

  virtual void interpolate(
      const ompl::base::State* x1,
      const ompl::base::State* x2,
      double t,
      ompl::base::State* x)
  {
    convertToTobiasFormat(x1, newX1);
    convertToTobiasFormat(x2, newX2);
    typename DI::Trajectory traj
        = doubleIntegrator_->getTrajectory(newX1, newX2);
    newX = traj.getState(t);
    convertFromTobiasFormat(newX, x);
    return;
  }

  virtual double getMinTime(
      const Eigen::VectorXd& x1, const Eigen::VectorXd& x2)
  {
    convertToTobiasFormatVec(x1, newX1);
    convertToTobiasFormatVec(x2, newX2);
    return doubleIntegrator_->getMinTime(newX1, newX2);
  }

  virtual Eigen::VectorXd interpolate(
      const Eigen::VectorXd& x1, const Eigen::VectorXd& x2, double t)
  {
    convertToTobiasFormatVec(x1, newX1);
    convertToTobiasFormatVec(x2, newX2);
    typename DI::Trajectory traj
        = doubleIntegrator_->getTrajectory(newX1, newX2);
    newX = traj.getState(t);
    Eigen::VectorXd x(param.dimensions);
    convertFromTobiasFormatVec(newX, x);
    return x;
  }

  virtual std::vector<Eigen::VectorXd> discretize(
      const Eigen::VectorXd& x1, const Eigen::VectorXd& x2, double step_t)
  {
    convertToTobiasFormatVec(x1, newX1);
    convertToTobiasFormatVec(x2, newX2);
    std::vector<Eigen::VectorXd> waypoints;
    double minTime = doubleIntegrator_->getMinTime(newX1, newX2);
    typename DI::Trajectory traj
        = doubleIntegrator_->getTrajectory(newX1, newX2);
    for (double t = 0.0; t < minTime; t += step_t)
    {
      newX = traj.getState(t);
      Eigen::VectorXd x(param.dimensions);
      convertFromTobiasFormatVec(newX, x);
      waypoints.push_back(x);
    }
    return waypoints;
  }

  virtual std::vector<Eigen::VectorXd> discretize(
      const ompl::base::State* x1, const ompl::base::State* x2, double step_t)
  {
    convertToTobiasFormat(x1, newX1);
    convertToTobiasFormat(x2, newX2);
    std::vector<Eigen::VectorXd> waypoints;
    double minTime = doubleIntegrator_->getMinTime(newX1, newX2);
    typename DI::Trajectory traj
        = doubleIntegrator_->getTrajectory(newX1, newX2);
    double t = 0.0;
    while (t < minTime)
    {
      newX = traj.getState(t);
      Eigen::VectorXd x(param.dimensions);
      convertFromTobiasFormatVec(newX, x);
      waypoints.push_back(x);
      t += step_t;
    }
    if (t >= minTime)
    {
      newX = traj.getState(minTime);
      Eigen::VectorXd x(param.dimensions);
      convertFromTobiasFormatVec(newX, x);
      waypoints.push_back(x);
    }
    return waypoints;
  }

protected:
  std::vector<double> maxAccelerations_;
  std::vector<double> maxVelocities_;
  std::shared_ptr<DI> doubleIntegrator_;
};

DoubleIntegratorMinimumTime::DoubleIntegratorMinimumTime(
    std::size_t _numDofs,
    std::vector<double>& _maxAccelerations,
    std::vector<double>& _maxVelocities)
  : mNumDofs(_numDofs)
{
  if (_numDofs != _maxVelocities.size())
  {
    throw std::invalid_argument("_numDofs not equal to _maxVelocities size");
  }
  if (_numDofs != _maxAccelerations.size())
  {
    throw std::invalid_argument("_numDofs not equal to _maxAccelerations size");
  }

  doubleIntegratorImpl_ = std::make_shared<DoubleIntegratorTobiasImpl>(
      _maxAccelerations, _maxVelocities);
}
