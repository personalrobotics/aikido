#ifndef DOUBLE_INTEGRATOR_MINIMUM_TIME_H_
#define DOUBLE_INTEGRATOR_MINIMUM_TIME_H_

// Standard libarary
#include <iostream>
#include <utility>    // std::pair
#include <limits>     // std::numeric_limits::infinity()
#include <algorithm>  // std::max and std::min
#include <vector>     // std::vector
#include <chrono>
#include <memory>
#include <Eigen/Dense>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include "aikido/planner/kinodynamics/dimt/Params.h"

class DoubleIntegratorImpl
{
public:
    virtual double getMinTime(const ompl::base::State* x1, const ompl::base::State* x2) = 0;

    virtual std::tuple<double, double, double>
    getMinTimeAndIntervals1Dof(const double x1, const double v1,
                               const double x2, const double v2,
                               int dof_index = 0) = 0;

    virtual void interpolate(const ompl::base::State* x1, const ompl::base::State* x2,
                             double t, ompl::base::State* x) = 0;

    virtual double getMinTimeIfSmallerThan(const ompl::base::State* x1, const ompl::base::State* x2,
                                    double timeThreshold)  = 0;

    virtual double getMinTime(const Eigen::VectorXd & x1, const Eigen::VectorXd & x2) = 0;

    virtual Eigen::VectorXd interpolate(const Eigen::VectorXd & x1, const Eigen::VectorXd & x2,
                                        double t) = 0;

    virtual std::vector<Eigen::VectorXd> discretize(const ompl::base::State* x1, const ompl::base::State* x2, double step_t);

    virtual std::vector<Eigen::VectorXd> discretize(const ompl::base::State* x1, const ompl::base::State* x2, double step_t, std::vector<double>& times) = 0;

    virtual std::vector<Eigen::VectorXd> discretize(const Eigen::VectorXd & x1, const Eigen::VectorXd & x2, double step_t);

    virtual std::vector<Eigen::VectorXd> discretize(const Eigen::VectorXd & x1, const Eigen::VectorXd & x2, double step_t, std::vector<double>& times) = 0;
};

class DoubleIntegratorMinimumTime
{
private:
    std::shared_ptr<DoubleIntegratorImpl> doubleIntegratorImpl_;
public:
    ///
    /// Constructor
    ///
    /// @param maxAccelerations Max accelerations of all DOFs
    /// @param maxVelocities Velocity limits of all DOFs
    ///
    DoubleIntegratorMinimumTime(std::size_t _numDofs,
                                std::vector<double>& _maxAccelerations,
                                std::vector<double>& _maxVelocities);

    /// This function calculates the maximum time given a set number of joints
    /// x = [x_1, x_1_dot,...,x_n,x_n_dot]
    /// @param x1 Initial state
    /// @param x2 Final state
    /// @return T Maximum time
    double getMinTime(const ompl::base::State* x1, const ompl::base::State* x2) const
    {
        return doubleIntegratorImpl_->getMinTime(x1, x2);
    }  

    std::tuple<double, double, double>
    getMinTimeAndIntervals1Dof(const double x1, const double v1,
                               const double x2, const double v2,
                               int dof_index = 0) const
    {
        return doubleIntegratorImpl_->getMinTimeAndIntervals1Dof(x1,v1,
                                                                 x2,v2,
                                                                 dof_index);
    }

    double getMinTimeIfSmallerThan(const ompl::base::State* x1, const ompl::base::State* x2, double timeThreshold) const
    {
        return doubleIntegratorImpl_->getMinTimeIfSmallerThan(x1, x2, timeThreshold);
    }

    void interpolate(const ompl::base::State* x1, const ompl::base::State* x2,
                     double t, ompl::base::State* x) const
    {
        return doubleIntegratorImpl_->interpolate(x1, x2, t, x);
    }

    std::vector<Eigen::VectorXd> discretize(const ompl::base::State* x1, const ompl::base::State* x2, double step_t)
    {
        return doubleIntegratorImpl_->discretize(x1, x2, step_t);
    }

    double getMinTime(const Eigen::VectorXd & x1, const Eigen::VectorXd & x2)
    {
        return doubleIntegratorImpl_->getMinTime(x1, x2);
    }

    Eigen::VectorXd interpolate(const Eigen::VectorXd & x1, const Eigen::VectorXd & x2,
                                double t)
    {
        return doubleIntegratorImpl_->interpolate(x1, x2, t);
    }

    std::vector<Eigen::VectorXd> discretize(const Eigen::VectorXd & x1, const Eigen::VectorXd & x2, double step_t)
    {
        return doubleIntegratorImpl_->discretize(x1, x2, step_t);
    }

    std::size_t getNumDofs() const
    {
      return mNumDofs;
    }

protected:
    std::size_t mNumDofs;
};

using DIMT = DoubleIntegratorMinimumTime;
using DIMTPtr = std::shared_ptr<DoubleIntegratorMinimumTime>;

#endif // DOUBLE_INTEGRATOR_MINIMUM_TIME_H_
