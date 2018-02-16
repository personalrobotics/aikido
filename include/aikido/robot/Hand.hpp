#ifndef AIKIDO_ROBOT_HAND_HPP_
#define AIKIDO_ROBOT_HAND_HPP_

#include <string>
#include <unordered_map>
#include <set>
#include <dart/dart.hpp>
#include "aikido/robot/GrabMetadata.hpp"
#include "aikido/io/yaml.hpp"
#include "aikido/control/PositionCommandExecutor.hpp"

namespace aikido {
namespace robot {

/// A base class for a hand used by manipulators
class Hand
{
public:

  Hand(const std::string &name,
      dart::dynamics::BranchPtr hand,
      bool simulation,
      dart::dynamics::BodyNode* endEffectorBodyNode,
      std::shared_ptr<aikido::control::PositionCommandExecutor> executor,
      std::unordered_map<std::string, size_t> fingerJointNameToPositionIndexMap);

  virtual ~Hand() = default;

  /// \param newName New name for this robot
  virtual std::unique_ptr<Hand> clone(const std::string& newName) = 0;

  /// Returns the end-effector body node.
  /// \return DART body node of end-effector
  dart::dynamics::BodyNode* getBodyNode() const;

  /// Returns the hand skeleton.
  /// \return DART Branch rooted at the palm
  dart::dynamics::BranchPtr getMetaSkeleton() const;

  /// Returns the hand's offset from a TSR of a specific object type
  /// (as registered in \c tsrEndEffectorTransformsUri).
  /// Each object's TSR.mTw_e must be right-multiplied with this.
  /// \param objectType Type of the object (e.g. "cylinder")
  /// \return hand's transform if it exists, boost::none if not
  boost::optional<Eigen::Isometry3d> getEndEffectorTransform(
      const std::string& objectType) const;

  /// Grabs an object. Immediately executes.
  /// \param bodyToGrab The object to grab
  /// \return bool for success
  void grab(const dart::dynamics::SkeletonPtr& bodyToGrab);

  /// Ungrabs an object. Immediately ungrabs.
  /// Throws a runtime_error if fails.
  void ungrab();

  /// Sets the hand to the corresponding preshape (from \c preshapesUri).
  /// \param preshapeName Name of preshape (e.g. "open")
  /// \throw a runtime_error if execution fails.
  void executePreshape(const std::string& preshapeName);

  /// Executes the preshape trajectory upto timepoint..
  // \param timepoint Time to simulate to.
  virtual void step(const std::chrono::system_clock::time_point& timepoint);

private:
  // Preshapes and end-effector transforms are read from YAML files
  using PreshapeMap = std::unordered_map<std::string, Eigen::VectorXd>;
  using EndEffectorTransformMap = std::
      unordered_map<std::string,
                    Eigen::Isometry3d,
                    std::hash<std::string>,
                    std::equal_to<std::string>,
                    Eigen::aligned_allocator<std::pair<const std::string,
                                                       Eigen::Isometry3d>>>;

  /// Loads preshapes from YAML file (retrieved from \c preshapesUri).
  /// \param node The YAML node to read from
  void parseYAMLToPreshapes(const YAML::Node& node);

  /// Loads end-effector transforms from YAML file (retrieved from
  /// \c tsrEndEffectorTransformsUri).
  /// \param node The YAML node to read from
  void parseYAMLToEndEffectorTransforms(const YAML::Node& node);

  /// Returns the corresponding preshape (from \c preshapesUri).
  /// \param preshapeName Name of preshape (e.g. "open")
  /// \return preshape if it exists, boost::none if not
  boost::optional<Eigen::VectorXd> getPreshape(
      const std::string& preshapeName);

  const std::string mName;

  dart::dynamics::BranchPtr mHand;

  const bool mSimulation;

  dart::dynamics::BodyNodePtr mEndEffectorBodyNode;
  std::shared_ptr<dart::collision::BodyNodeCollisionFilter>
      mSelfCollisionFilter;

  std::shared_ptr<aikido::control::PositionCommandExecutor> mExecutor;

  // Used to find the final pose before actual execution
  std::shared_ptr<aikido::control::PositionCommandExecutor> mSimExecutor;
  PreshapeMap mPreshapeConfigurations;
  EndEffectorTransformMap mEndEffectorTransforms;

  std::unordered_map<std::string, size_t> mFingerJointNameToPositionIndexMap;

  // TODO: change this to grab multiple objects
  std::unique_ptr<GrabMetadata> mGrabMetadata;

};

using HandPtr = std::shared_ptr<Hand>;

} // namespace robot
} // namespace aikido

#endif

