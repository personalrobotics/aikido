#include "aikido/io/trajectory.hpp"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <boost/program_options.hpp>
#include "aikido/common/Spline.hpp"
#include "aikido/io/yaml.hpp"
#include "aikido/io/detail/yaml_extension.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"
#include "aikido/trajectory/Interpolated.hpp"

using aikido::statespace::ConstStateSpacePtr;
using aikido::statespace::StateSpacePtr;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr;
using aikido::trajectory::Spline;
using aikido::trajectory::UniqueSplinePtr;

namespace aikido {
namespace io {

void saveTrajectory(const aikido::trajectory::Spline& trajectory,
    const std::string& savePath)
{
  std::ofstream file(savePath);
  YAML::Emitter emitter;

  auto skelSpace = std::dynamic_pointer_cast<const MetaSkeletonStateSpace>
      (trajectory.getStateSpace());
  if (!skelSpace)
    throw std::runtime_error("Trajectory state space is not MetaSkeletonStateSpace");

  emitter << YAML::BeginMap;
  emitter << YAML::Key << "configuration";
  emitter << YAML::BeginMap;
  emitter << YAML::Key << "start_time" << YAML::Value << trajectory.getStartTime();
  emitter << YAML::Key << "dofs" << YAML::Flow << skelSpace->getProperties().getDofNames();
  emitter << YAML::Key << "type" << YAML::Value << "spline";
  emitter << YAML::Key << "spline";
  emitter << YAML::BeginMap;
  emitter << YAML::Key << "order" << YAML::Value << trajectory.getNumDerivatives();
  emitter << YAML::EndMap;
  emitter << YAML::EndMap;

  Eigen::VectorXd position(skelSpace->getDimension());
  aikido::io::detail::encode_impl<Eigen::MatrixXd, false> convertMatrix;
  aikido::io::detail::encode_impl<Eigen::VectorXd, false> convertVector;

  emitter << YAML::Key << "data";
  emitter << YAML::BeginSeq;
  for (std::size_t i = 0; i < trajectory.getNumSegments(); ++i)
  {
    auto startState = trajectory.getSegmentStartState(i);
    double duration = trajectory.getSegmentDuration(i);
    auto segmentCoeff = trajectory.getSegmentCoefficients(i);

    // Convert start state to Eigen vector
    skelSpace->logMap(startState, position);

    emitter << YAML::BeginMap;
    emitter << YAML::Key << "coefficients" << YAML::Flow
        << convertMatrix.encode(segmentCoeff);
    emitter << YAML::Key << "duration" << YAML::Value << duration;
    emitter << YAML::Key << "start_state" << YAML::Flow
        << convertVector.encode(position);
    emitter << YAML::EndMap;
  }

  emitter << YAML::EndSeq << YAML::EndMap;
  file << emitter.c_str();
}

//==============================================================================
UniqueSplinePtr loadSplineTrajectory(
    const std::string& trajPath,
    const ConstMetaSkeletonStateSpacePtr& metaSkeletonStateSpace)
{
  YAML::Node trajFile = YAML::LoadFile(trajPath);
  const YAML::Node& config = trajFile["configuration"];

  double startTime = config["start_time"].as<double>();
  std::vector<std::string> dofs = config["dofs"].as<std::vector<std::string>>();
  auto paramDofs = metaSkeletonStateSpace->getProperties().getDofNames();
  if (!std::equal(dofs.begin(), dofs.end(), paramDofs.begin()))
  {
    throw std::runtime_error("Dof names should be same");
  }

  std::string trajType = config["type"].as<std::string>();
  if (trajType.compare("spline"))
  {
    throw std::runtime_error("Trajectory type should be spline");
  }

  auto trajectory
      = ::aikido::common::make_unique<Spline>(metaSkeletonStateSpace, startTime);
  aikido::statespace::StateSpace::State* startState
        = metaSkeletonStateSpace->allocateState();

  const YAML::Node& segments = trajFile["data"];
  for (YAML::const_iterator it = segments.begin(); it != segments.end(); ++it)
  {
    const YAML::Node& segment = *it;
    Eigen::MatrixXd coefficients = segment["coefficients"].as<Eigen::MatrixXd>();
    double duration = segment["duration"].as<double>();
    Eigen::VectorXd position = segment["start_state"].as<Eigen::VectorXd>();

    // Convert position Eigen vector to StateSpace::State*
    metaSkeletonStateSpace->expMap(position, startState);

    trajectory->addSegment(coefficients, duration, startState);
  }

  metaSkeletonStateSpace->freeState(startState);
  return trajectory;
}

} // namespace io
} // namespace aikido
