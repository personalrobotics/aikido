#ifndef AIKIDO_ROBOT_HAND_HPP_
#define AIKIDO_ROBOT_HAND_HPP_

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <dart/dart.hpp>
#include "aikido/robot/GrabInfo.hpp"
#include "aikido/io/yaml.hpp"
#include "aikido/control/PositionCommandExecutor.hpp"

namespace aikido {
namespace robot {

/// A base class for a hand used by manipulators
class Hand
{
public:

  /// \param newName New name for this robot
  virtual std::unique_ptr<Hand> clone(const std::string& newName) override;

  /// Get the end-effector body node.
  /// \return DART body node of end-effector
  dart::dynamics::BodyNode* getBodyNode() const;

  /// Get the hand skeleton.
  /// \return DART Branch rooted at palm of a BarrettHand
  dart::dynamics::BranchPtr getHand();

  /// Get the hand's offset from a TSR of a specific object type (as registered
  /// in \c tsrEndEffectorTransformsUri). Each object's TSR.mTw_e must be
  /// right-multiplied with this.
  /// \param[in] _objectType Type of the object (e.g. "cylinder")
  /// \return hand's transform if it exists, boost::none if not
  boost::optional<Eigen::Isometry3d> getEndEffectorTransform(
      const std::string& _objectType) const;

  /// Grab an object. Immediately executes.
  /// \param[in] bodyToGrab The object to grab
  /// \return bool for success
  void grab(const dart::dynamics::SkeletonPtr& bodyToGrab);

  /// Ungrab an object. Immediately ungrabs.
  /// Throws a runtime_error if fails.
  void ungrab();


  /// Set the hand to the corresponding preshape (from \c preshapesUri).
  /// \param[in] preshapeName Name of preshape (e.g. "open")
  /// \throw a runtime_error if execution fails.
  void executePreshape(const std::string& preshapeName);

  /// Execute one step of the preshape trajectory.
  void step();

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

  /// Load preshapes from YAML file (retrieved from \c preshapesUri).
  /// \param[in] node The YAML node to read from
  void parseYAMLToPreshapes(const YAML::Node& node);

  /// Load end-effector transforms from YAML file (retrieved from
  /// \c tsrEndEffectorTransformsUri).
  /// \param[in] node The YAML node to read from
  void parseYAMLToEndEffectorTransforms(const YAML::Node& node);

  /// Return the corresponding preshape (from \c preshapesUri).
  /// \param[in] _preshapeName Name of preshape (e.g. "open")
  /// \return preshape if it exists, boost::none if not
  boost::optional<Eigen::VectorXd> getPreshape(
      const std::string& _preshapeName);

  // Controllers
  std::shared_ptr<aikido::control::PositionCommandExecutor>
  createHandPositionExecutor(dart::dynamics::SkeletonPtr robot);

  const std::string mName;
  const bool mSimulation;
  dart::dynamics::BodyNodePtr mEndEffectorBodyNode;
  dart::dynamics::BranchPtr mHand;
  std::shared_ptr<dart::collision::BodyNodeCollisionFilter>
      mSelfCollisionFilter;

  std::unique_ptr<::ros::NodeHandle> mNode;
  std::shared_ptr<aikido::control::PositionCommandExecutor> mExecutor;

  PreshapeMap mPreshapeConfigurations;
  EndEffectorTransformMap mEndEffectorTransforms;

  std::unordered_set<GrabInfo> mGrabInfos;

  static const std::unordered_map<std::string, size_t>
      fingerJointNameToPositionIndexMap;
};

using HandPtr = std::shared_ptr<Hand>;

} // namespace robot
} // namespace aikido
