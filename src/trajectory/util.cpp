#include <aikido/trajectory/util.hpp>
#include <algorithm>
#include <aikido/common/Spline.hpp>
#include <aikido/common/StepSequence.hpp>
#include <dart/common/StlHelpers.hpp>
#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/planner/parabolic/ParabolicTimer.hpp>

namespace po = boost::program_options;

namespace aikido {
namespace trajectory {

inline int sgn(double x)
{
  return (x < 0) ? -1 : (x > 0);
}

//==============================================================================
int quadraticRootFinder(
    double a, double b, double c, double& r1, double& r2, double epsilon)
{
  if (a == 0)
  {
    // 1st order
    if (b == 0)
    {
      return -1;
    }
    else
    {
      r1 = -c / b;
      return 1;
    }
  }

  if (c == 0)
  {
    r1 = 0;
    r2 = -b / a;
    return 2;
  }

  double det = b * b - 4.0 * a * c;
  if (det < 0.0)
  {
    return -1;
  }
  else if (det == 0.0)
  {
    r1 = -b / (2.0 * a);
    return 1;
  }
  else
  {
    double sqrt_det = sqrt(det);
    if (abs(-b - sqrt_det) < abs(a))
    {
      r1 = 0.5 * (-b + sqrt_det) / a;
    }
    else
    {
      r1 = 2.0 * c / (-b - sqrt_det);
    }

    if (abs(-b + sqrt_det) < abs(a))
    {
      r1 = 0.5 * (-b - sqrt_det) / a;
    }
    else
    {
      r2 = 2.0 * c / (-b + sqrt_det);
    }

    if (r1 < 0 && r1 > -epsilon)
      r1 = 0;
    if (r2 < 0 && r2 > -epsilon)
      r2 = 0;
    return 2;
  }

  return -1;
}

//==============================================================================
double calcSwitchTime(
    double x0, double x1, double dx0, double dx1, double accel)
{
  int res = -1;
  double t1 = 0.0, t2 = 0.0;
  double epsilon = 1e-6;

  double a = 0.0, b = 0.0, c = 0.0;
  if (abs(accel) > 1.0)
  {
    a = accel;
    b = 2.0 * dx0;
    c = 0.5 * (sqrt(dx0) - sqrt(dx1)) / a + x0 - x1;
  }
  else
  {
    a = accel * accel;
    b = 2.0 * accel * dx0;
    c = 0.5 * (sqrt(dx0) - sqrt(dx1)) + (x0 - x1) * accel;
  }

  res = quadraticRootFinder(a, b, c, t1, t2, epsilon);

  if (res == 1)
  {
    if (t1 < 0)
      return 0.0;
    return t1;
  }
  else if (res == 2)
  {
    if (t1 < 0 || t1 * abs(accel) < (dx1 - dx0) * sgn(accel))
    {
      if (t2 < 0 || t2 * abs(accel) < (dx1 - dx0) * sgn(accel))
        return 0.0;

      t1 = t2;
      return t1;
    }

    return std::min(t1, t2);
  }
  return 0.0;
}

void from_vector(const Eigen::VectorXd& e_vector, std::vector<double>& s_vector)
{
  s_vector = std::vector<double>(
      e_vector.data(), e_vector.data() + e_vector.rows() * e_vector.cols());
}

void to_vector(const std::vector<double>& s_vector, Eigen::VectorXd& e_vector)
{
  for (std::size_t i = 0; i < s_vector.size(); i++)
  {
    e_vector[i] = s_vector[i];
  }
}

//==============================================================================
void printStateWithTime(
    double t,
    std::size_t dimension,
    Eigen::VectorXd& stateVec,
    Eigen::VectorXd& velocityVec,
    std::ofstream& cout)
{
  cout << t << ",";
  for (std::size_t i = 0; i < dimension; i++)
  {
    cout << stateVec[i] << "," << velocityVec[i];
    if (i < dimension - 1)
    {
      cout << ",";
    }
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

  aikido::common::StepSequence sequence(
      timeStep, true, true, spline.getStartTime(), spline.getEndTime());
  auto state = stateSpace->createState();
  Eigen::VectorXd stateVec(dim);
  Eigen::VectorXd velocityVec(dim);

  for (std::size_t i = 0; i < sequence.getLength(); i++)
  {
    double t = sequence[i];
    spline.evaluate(t, state);
    spline.evaluateDerivative(t, 1, velocityVec);
    stateSpace->logMap(state, stateVec);
    printStateWithTime(t, dim, stateVec, velocityVec, phasePlotFile);
  }

  phasePlotFile.close();
  return;
}

double findClosetStateOnTrajectory(
    const aikido::trajectory::Trajectory* traj,
    const Eigen::VectorXd& config,
    double timeStep)
{
  if (traj == nullptr)
    throw std::runtime_error("Traj is nullptr");
  auto stateSpace = traj->getStateSpace();
  if (config.size() != stateSpace->getDimension())
    throw std::runtime_error("Dimension mismatch");

  double findTime = traj->getStartTime();
  double minDist = std::numeric_limits<double>::max();

  aikido::common::StepSequence sequence(
      timeStep, true, true, traj->getStartTime(), traj->getEndTime());

  auto currState = stateSpace->createState();
  Eigen::VectorXd currPos(stateSpace->getDimension());
  for (std::size_t i = 0; i < sequence.getLength(); i++)
  {
    double currTime = sequence[i];
    traj->evaluate(currTime, currState);
    stateSpace->logMap(currState, currPos);

    double currDist = (config - currPos).norm();
    if (currDist < minDist)
    {
      findTime = currTime;
      minDist = currDist;
    }
  }
  ROS_INFO_STREAM("findClosestStateOnTrajectory minDist: " << minDist);

  return findTime;
}

std::unique_ptr<aikido::trajectory::Spline> createPartialTrajectory(
    const aikido::trajectory::Spline& traj, double partialStartTime)
{
  if (partialStartTime < traj.getStartTime()
      || partialStartTime > traj.getEndTime())
    throw std::runtime_error("Wrong partial start time");

  using dart::common::make_unique;
  using CubicSplineProblem = aikido::common::
      SplineProblem<double, int, 4, Eigen::Dynamic, Eigen::Dynamic>;

  auto stateSpace = traj.getStateSpace();
  std::size_t dimension = stateSpace->getDimension();
  auto outputTrajectory = make_unique<aikido::trajectory::Spline>(
      stateSpace, traj.getStartTime());

  double currSegmentStartTime = traj.getStartTime();
  double currSegmentEndTime = currSegmentStartTime;
  std::size_t currSegmentIdx = 0;

  auto segmentStartState = stateSpace->createState();
  auto segmentEndState = stateSpace->createState();
  Eigen::VectorXd segStartPos(dimension), segEndPos(dimension),
      segStartVel(dimension), segEndVel(dimension);
  const Eigen::VectorXd zeroPos = Eigen::VectorXd::Zero(dimension);
  traj.evaluate(partialStartTime, segmentStartState);
  stateSpace->logMap(segmentStartState, segStartPos);
  traj.evaluateDerivative(partialStartTime, 1, segStartVel);

  while (currSegmentIdx < traj.getNumSegments())
  {
    currSegmentEndTime += traj.getSegmentDuration(currSegmentIdx);
    if (partialStartTime >= currSegmentStartTime
        && partialStartTime <= currSegmentEndTime)
    {
      std::cout << "FIND " << partialStartTime << " IN " << currSegmentIdx
                << "-th [" << currSegmentStartTime;
      std::cout << " , " << currSegmentEndTime << "]" << std::endl;
      // create new segment
      traj.evaluate(currSegmentEndTime, segmentEndState);
      stateSpace->logMap(segmentEndState, segEndPos);
      traj.evaluateDerivative(currSegmentEndTime, 1, segEndVel);

      double segmentDuration = currSegmentEndTime - partialStartTime;

      if (segmentDuration > 0.0)
      {
        CubicSplineProblem problem(
            Eigen::Vector2d{0., segmentDuration}, 4, dimension);
        problem.addConstantConstraint(0, 0, zeroPos);
        problem.addConstantConstraint(0, 1, segStartVel);
        problem.addConstantConstraint(1, 0, segEndPos - segStartPos);
        problem.addConstantConstraint(1, 1, segEndVel);
        const auto solution = problem.fit();
        const auto coefficients = solution.getCoefficients().front();

        outputTrajectory->addSegment(
            coefficients, segmentDuration, segmentStartState);
      }
      break;
    }

    currSegmentIdx++;
    currSegmentStartTime = currSegmentEndTime;
  }

  for (std::size_t i = currSegmentIdx + 1; i < traj.getNumSegments(); i++)
  {
    // std::cout << "CONTINUE ADDING " << i << "-th SEGMENT" << std::endl;
    outputTrajectory->addSegment(
        traj.getSegmentCoefficients(i),
        traj.getSegmentDuration(i),
        traj.getSegmentState(i));
  }

  return outputTrajectory;
}

std::unique_ptr<aikido::trajectory::Interpolated> convertToInterpolated(const aikido::trajectory::Spline& traj)
{
  using dart::common::make_unique;
  auto stateSpace = traj.getStateSpace();
  auto dim = stateSpace->getDimension();
   
  auto interpolator = std::make_shared<aikido::statespace::GeodesicInterpolator>(stateSpace);

  auto outputTrajectory
      = make_unique<aikido::trajectory::Interpolated>(stateSpace, interpolator);

  auto state = stateSpace->createState();
  double t = 0.0;
  for(std::size_t i=0; i<traj.getNumWaypoints();i++)
  {
    traj.getWaypoint(i, state);
    t = traj.getWaypointTime(i);
    outputTrajectory->addWaypoint(t, state);
  }

  return outputTrajectory;
}

std::unique_ptr<aikido::trajectory::Interpolated> concatenate(const aikido::trajectory::Interpolated& traj1,
                                                              const aikido::trajectory::Interpolated& traj2)
{
  auto statespace1 = traj1.getStateSpace();
  auto dim1 = statespace1->getDimension();
  auto statespace2 = traj2.getStateSpace();
  auto dim2 = statespace2->getDimension();
  if(traj1.getStateSpace()->getDimension()!=traj2.getStateSpace()->getDimension())
    throw std::runtime_error("Dimension mismatch");

  using dart::common::make_unique;
  auto stateSpace = traj1.getStateSpace();
  std::size_t dimension = stateSpace->getDimension();

  auto outputTrajectory
      = make_unique<aikido::trajectory::Interpolated>(stateSpace, traj1.getInterpolator());

  for(std::size_t i=0; i<traj1.getNumWaypoints()-1;i++)
  {
    outputTrajectory->addWaypoint(traj1.getWaypointTime(i), traj1.getWaypoint(i));
  }
  outputTrajectory->addWaypoint(traj1.getEndTime(), traj2.getWaypoint(0));
  double endTimeOfTraj1 = traj1.getEndTime();
  for(std::size_t i=1; i<traj2.getNumWaypoints(); i++)
  {
    outputTrajectory->addWaypoint(endTimeOfTraj1+traj2.getWaypointTime(i), traj2.getWaypoint(i));
  }
  return outputTrajectory;
}

std::unique_ptr<aikido::trajectory::Spline> concatenate(
    const aikido::trajectory::Spline& traj1,
    const aikido::trajectory::Spline& traj2)
{
  auto interpolated1 = convertToInterpolated(traj1);
  auto interpolated2 = convertToInterpolated(traj2);
  auto concatenatedInterpolated = concatenate(*interpolated1, *interpolated2);
  return aikido::planner::parabolic::convertToSpline(*concatenatedInterpolated);
}

}
}
