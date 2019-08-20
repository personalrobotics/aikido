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
using aikido::statespace::dart::MetaSkeletonStateSpacePtr;
using aikido::trajectory::Spline;
using aikido::trajectory::UniqueSplinePtr;

namespace aikido {
namespace io {

void saveTrajectory(const aikido::trajectory::Spline& trajectory,
    const MetaSkeletonStateSpacePtr& skelSpace, const std::string& savePath)
{
  std::ofstream file(savePath);
  YAML::Emitter emitter;

  statespace::ConstStateSpacePtr trajSpace = trajectory.getStateSpace();
  emitter << YAML::BeginMap;
  emitter << YAML::Key << "configuration" << YAML::BeginMap;
  emitter << YAML::Key << "start_time" << YAML::Value << trajectory.getStartTime();
  emitter << YAML::Key << "dofs" << YAML::Flow << skelSpace->getProperties().getDofNames();
  emitter << YAML::Key << "type" << YAML::Value << "spline";
  emitter << YAML::Key << "spline";
  emitter << YAML::BeginMap;
  emitter << YAML::Key << "order" << YAML::Value << trajectory.getNumDerivatives();
  emitter << YAML::EndMap;
  emitter << YAML::EndMap << YAML::Key << "data" << YAML::BeginSeq;

  Eigen::VectorXd position(trajSpace->getDimension());
  aikido::io::detail::encode_impl<Eigen::MatrixXd, false> convertMatrix;
  aikido::io::detail::encode_impl<Eigen::VectorXd, false> convertVector;

  for (std::size_t i = 0; i < trajectory.getNumSegments(); ++i)
  {
    auto startState = trajectory.getSegmentStartState(i);
    double duration = trajectory.getSegmentDuration(i);
    auto segmentCoeff = trajectory.getSegmentCoefficients(i);

    // Convert start state to position
    trajSpace->logMap(startState, position);
    
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
    const std::string& trajPath, const MetaSkeletonStateSpacePtr& stateSpace)
{
  YAML::Node trajFile = YAML::LoadFile(trajPath);
  const YAML::Node& config = trajFile["configuration"];

  double startTime = config["start_time"].as<double>();
  std::vector<std::string> dofs = config["dofs"].as<std::vector<std::string>>();
  std::string trajType = config["type"].as<std::string>();

  auto Trajectory
      = ::aikido::common::make_unique<Spline>(stateSpace, startTime);

  const YAML::Node& segments = trajFile["data"];
  for (std::size_t i = 0; i < segments.size(); i++) {
    const YAML::Node& segment = segments[i];
    Eigen::MatrixXd coefficients = segment["coefficients"].as<Eigen::MatrixXd>();
    double duration = segment["duration"].as<double>();
    Eigen::VectorXd position = segment["start_state"].as<Eigen::VectorXd>();

    // Convert position Eigen vector to StateSpace::State*
    aikido::statespace::StateSpace::State* startState
        = stateSpace->allocateState();
    stateSpace->expMap(position, startState);

    Trajectory->addSegment(coefficients, duration, startState);
  }
  return Trajectory;
}

} // namespace io
} // namespace aikido