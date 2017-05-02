#include <chrono>
#include <cmath>
#include "HauserParabolicSmoother.hpp"
#include <aikido/planner/parabolic/detail/ParabolicUtil.hpp>
#include "Config.h"

using namespace ParabolicRamp;

namespace aikido {
namespace planner {
namespace parabolic {

SmootherFeasibilityChecker::SmootherFeasibilityChecker(aikido::constraint::TestablePtr testable)
    : testable_(testable)
{
}

bool SmootherFeasibilityChecker::ConfigFeasible(ParabolicRamp::Vector const &x)
{
    statespace::StateSpacePtr stateSpace = testable_->getStateSpace();
    Eigen::VectorXd eigX = toEigen(x);
    auto state = stateSpace->createState();
    stateSpace->expMap(eigX, state);
    return testable_->isSatisfied(state);
}

bool SmootherFeasibilityChecker::SegmentFeasible(ParabolicRamp::Vector const &a,
                                                 ParabolicRamp::Vector const &b)
{
   statespace::StateSpacePtr stateSpace = testable_->getStateSpace();
   //TODO: confirm implementation
   Eigen::VectorXd eigA = toEigen(a);
   Eigen::VectorXd eigB = toEigen(b);
   Eigen::VectorXd deltaVec = eigA - eigB;
   double resolution = 1e-2;
   double stepNum = deltaVec.norm() / resolution;
   double stepLen = 1.0/stepNum;
   for(double i=0.0;i<=1.;i+=stepLen)
   {
       Eigen::VectorXd newVec = eigA + i * deltaVec;
       auto newState = stateSpace->createState();
       stateSpace->expMap(newVec, newState);
       if(!testable_->isSatisfied(newState))
       {
           return false;
       }
   }
   return true;
}

HauserParabolicSmoother::HauserParabolicSmoother(aikido::constraint::TestablePtr testable,
                                                 double timelimit,
                                                 bool useVelocity,
                                                 double blendRadius,
                                                 int blendIterations,
                                                 double tolerance)
    : blendRadius_(blendRadius),
      blendIterations_(blendIterations),
      timelimit_(timelimit),
      useVelocity_(useVelocity),
      tolerance_(tolerance),
      feasibilityChecker_(new SmootherFeasibilityChecker(testable),
                          static_cast<ParabolicRamp::Real>(tolerance_))
{
}


bool HauserParabolicSmoother::doShortcut(ParabolicRamp::DynamicPath* dynamicPath)
{
    std::chrono::time_point<std::chrono::system_clock> startTime, currentTime;
    startTime = std::chrono::system_clock::now();
    double elapsedTime = -1;

    while (elapsedTime < timelimit_ && dynamicPath->ramps.size() > 3)
    {
        dynamicPath->Shortcut(1, feasibilityChecker_);

        currentTime = std::chrono::system_clock::now();
        elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
            currentTime - startTime).count();
    }

    return true;
}


bool HauserParabolicSmoother::doBlend(ParabolicRamp::DynamicPath* dynamicPath)
{
    // Mark all of the ramps in the initial trajectory as "original". We'll
    // only try to blend transitions between these ramps.
    for(unsigned int i=0;i<dynamicPath->ramps.size();++i)
    {
        dynamicPath->ramps[i].blendAttempts = 0;
    }

    double dtShortcut = blendRadius_;

    for (int attempt = 0; attempt < blendIterations_; ++attempt)
    {
        while (tryBlend(dynamicPath, attempt, dtShortcut));

        dtShortcut /= 2.;
    }
    return true;
}

bool HauserParabolicSmoother::tryBlend(ParabolicRamp::DynamicPath* dynamicPath,
                                       int attempt, double dtShortcut)
{
    size_t const numRamps = dynamicPath->ramps.size();
    double const tMax = dynamicPath->GetTotalTime();
    double t = 0;

    for (size_t iwaypoint = 0; iwaypoint < numRamps - 1; ++iwaypoint)
    {
        ParabolicRamp::ParabolicRampND &rampNd
                = dynamicPath->ramps[iwaypoint];
        t += rampNd.endTime;

        if (needsBlend(rampNd) && rampNd.blendAttempts == attempt)
        {
            double const t1 = std::max(t - dtShortcut, - ParabolicRamp::EpsilonT);
            double const t2 = std::min(t + dtShortcut, tMax + ParabolicRamp::EpsilonT);

            bool const success = dynamicPath->TryShortcut(t1, t2, feasibilityChecker_);

            if (success)
            {
                return true;
            } else {
                rampNd.blendAttempts++;
            }
        }
    }

    return false;
}

bool HauserParabolicSmoother::needsBlend(ParabolicRamp::ParabolicRampND const &rampNd)
{
    for (size_t idof = 0; idof < rampNd.dx1.size(); ++idof)
    {
        if (std::fabs(rampNd.dx1[idof]) > ParabolicRamp::EpsilonV)
        {
            return false;
        }
    }
    return true;
}


} // namespace parabolic
} // namespace planner
} // namespace aikido
