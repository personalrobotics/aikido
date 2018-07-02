#include "KinodynamicUtil.hpp"
#include <fstream>
#include <iostream>
#include "aikido/common/Spline.hpp"

namespace aikido {
namespace planner {
namespace kinodynamics {

void printStateWithTime(
    double t,
    std::size_t dimension,
    Eigen::VectorXd& stateVec,
    Eigen::VectorXd& velocityVec,
    std::ofstream& cout)
{
  cout << t << " ";
  for (std::size_t i = 0; i < dimension; i++)
  {
    cout << stateVec[i] << " " << velocityVec[i] << " ";
  }
  cout << std::endl;
  return;
}

void dumpSplinePhasePlot(
    const aikido::trajectory::Spline& spline,
    const std::string& filename,
    double timeStep)
{
  std::ofstream phasePlotFile;
  phasePlotFile.open(filename);
  auto stateSpace = spline.getStateSpace();
  std::size_t dim = stateSpace->getDimension();

  auto state = stateSpace->createState();
  Eigen::VectorXd stateVec(dim);
  Eigen::VectorXd velocityVec(dim);
  double t = spline.getStartTime();
  while (t + timeStep < spline.getEndTime())
  {
    spline.evaluate(t, state);
    spline.evaluateDerivative(t, 1, velocityVec);
    stateSpace->logMap(state, stateVec);
    printStateWithTime(t, dim, stateVec, velocityVec, phasePlotFile);
    t += timeStep;
  }
  spline.evaluate(spline.getEndTime(), state);
  spline.evaluateDerivative(spline.getEndTime(), 1, velocityVec);
  stateSpace->logMap(state, stateVec);
  printStateWithTime(t, dim, stateVec, velocityVec, phasePlotFile);

  phasePlotFile.close();
  return;
}

bool convertPathToSequentialStates(
    ::ompl::geometric::PathGeometric* path,
    const DIMTPtr& dimt,
    double interpolateStepSize,
    std::vector<Eigen::VectorXd>& points,
    std::vector<double>& times)
{
  if (path)
  {
    points.clear();
    times.clear();

    std::size_t node_num = path->getStateCount();
    double last_time = 0.0;
    for (size_t idx = 0; idx < node_num - 1; idx++)
    {
      ::ompl::base::State* state1 = path->getState(idx);
      ::ompl::base::State* state2 = path->getState(idx + 1);
      std::vector<double> deltaTimes;
      std::vector<Eigen::VectorXd> deltaPoints
          = dimt->discretize(state1, state2, interpolateStepSize, deltaTimes);

      std::transform(
          deltaTimes.begin(),
          deltaTimes.end(),
          deltaTimes.begin(),
          std::bind2nd(std::plus<double>(), last_time));

      if (idx == 0)
      {
        points.insert(points.end(), deltaPoints.begin(), deltaPoints.end());
        times.insert(times.end(), deltaTimes.begin(), deltaTimes.end());
      }
      else
      {
        points.insert(points.end(), deltaPoints.begin() + 1, deltaPoints.end());
        times.insert(times.end(), deltaTimes.begin() + 1, deltaTimes.end());
      }

      // update last time
      last_time = times.back();
    }

    return true;
  }
  return false;
}

std::unique_ptr<aikido::trajectory::Spline> convertSequentialStatesToSpline(
    const statespace::dart::MetaSkeletonStateSpacePtr& metaSkeletonStateSpace,
    std::vector<Eigen::VectorXd>& points,
    std::vector<double>& times)
{
  std::size_t dimension = metaSkeletonStateSpace->getDimension();
  using CubicSplineProblem
      = aikido::common::SplineProblem<double, int, 4, Eigen::Dynamic, 2>;

  auto outputTrajectory = dart::common::make_unique<aikido::trajectory::Spline>(
      metaSkeletonStateSpace);
  auto segmentStartState = metaSkeletonStateSpace->createState();

  Eigen::VectorXd zeroPosition = Eigen::VectorXd::Zero(dimension);
  for (std::size_t i = 0; i < points.size() - 1; i++)
  {
    Eigen::VectorXd positionCurr = points[i].head(dimension);
    Eigen::VectorXd velocityCurr = points[i].tail(dimension);
    Eigen::VectorXd positionNext = points[i + 1].head(dimension);
    Eigen::VectorXd velocityNext = points[i + 1].tail(dimension);

    double timeCurr = times[i];
    double timeNext = times[i + 1];

    CubicSplineProblem problem(
        Eigen::Vector2d(0.0, timeNext - timeCurr), 4, dimension);
    problem.addConstantConstraint(0, 0, zeroPosition);
    problem.addConstantConstraint(0, 1, velocityCurr);
    problem.addConstantConstraint(1, 0, positionNext - positionCurr);
    problem.addConstantConstraint(1, 1, velocityNext);
    const auto spline = problem.fit();

    metaSkeletonStateSpace->expMap(positionCurr, segmentStartState);

    // Add the ramp to the output trajectory.
    assert(spline.getCoefficients().size() == 1);
    const auto& coefficients = spline.getCoefficients().front();
    outputTrajectory->addSegment(
        coefficients, timeNext - timeCurr, segmentStartState);
  }

  return outputTrajectory;
}

void saveSuboptimalSolutions(
    const std::vector<::ompl::geometric::PathGeometric*>& paths,
    const DIMTPtr& dimt,
    const statespace::dart::MetaSkeletonStateSpacePtr& metaSkeletonStateSpace, 
    std::string filenamePrex,
    double interpolateStepSize)
{
  std::stringstream ss;
  for(std::size_t i=0; i<paths.size(); i++)
  {
    ss.clear();
    ss << filenamePrex << "_" << i << ".txt";
    
    std::vector<Eigen::VectorXd> points;
    std::vector<double> times;
    convertPathToSequentialStates(paths[i], dimt,
                                  interpolateStepSize,
                                  points, times);
    auto traj = convertSequentialStatesToSpline(
        metaSkeletonStateSpace, points, times);

    std::string filename = ss.str();
    std::cout << "WRITING TO " << filename << std::endl;
    dumpSplinePhasePlot(*traj.get(), filename, interpolateStepSize);
  }

}

} // namespace kinodynamics
} // namespace planner
} // namespace aikido
