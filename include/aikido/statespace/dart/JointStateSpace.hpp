#ifndef AIKIDO_STATESPACE_DART_JOINTSTATESPACE_HPP_
#define AIKIDO_STATESPACE_DART_JOINTSTATESPACE_HPP_

#include <dart/dynamics/dynamics.hpp>

#include "aikido/statespace/StateSpace.hpp"

namespace aikido {
namespace statespace {
namespace dart {

/// \c StateSpace of a DART \c Joint. This is a base class that is inherited by
/// concrete implementations for DART's various \c Joint subclasses. This class
/// provides functions for converting between \c State objects and vectors of
/// DART joint positions.
///
/// Since this class does not keep a reference to the underlying \c Joint,
/// changes made to the \c Joint after the state space is constructed will not
/// be reflected in the \c JointStateSpace.
class JointStateSpace : public virtual StateSpace
{
public:
  /// Static properties from the DART \c Joint.
  /// These will not update if the \c Joint is updated.
  class Properties
  {
  public:
    /// Constructs the joint properties for \c joint.
    ///
    /// \param joint \c Joint to create properties for
    explicit Properties(const ::dart::dynamics::Joint* joint);

    /// Return the name of the joint.
    const std::string& getName() const;

    /// Return the type of the joint.
    const std::string& getType() const;

    /// Return the number of DOFs in the joint.
    std::size_t getNumDofs() const;

    /// Return the names of DOFs in the joint.
    const std::vector<std::string>& getDofNames() const;

    /// Return whether the index DOF is position-limited.
    bool hasPositionLimit(std::size_t index) const;

    /// Return whether any DOF is position-limited.
    bool isPositionLimited() const;

    /// Return the vector of position lower limits.
    const Eigen::VectorXd& getPositionLowerLimits() const;

    /// Return the vector of position upper limits.
    const Eigen::VectorXd& getPositionUpperLimits() const;

    /// Return the vector of velocity lower limits.
    const Eigen::VectorXd& getVelocityLowerLimits() const;

    /// Return the vector of velocity upper limits.
    const Eigen::VectorXd& getVelocityUpperLimits() const;

    /// Return whether two JointStateSpace::Properties are identical.
    /// \param otherProperties Other Properties to compare against
    bool operator==(const Properties& otherProperties) const;

    /// Return whether two JointStateSpace::Properties are different.
    /// \param otherProperties Other Properties to compare against
    bool operator!=(const Properties& otherProperties) const;

  protected:
    /// Name of the joint
    std::string mName;

    /// Name of the joint type
    std::string mType;

    /// Names of DOFs in the Joint
    std::vector<std::string> mDofNames;

    /// The joint's position lower limits
    Eigen::VectorXd mPositionLowerLimits;

    /// The joint's position upper limits
    Eigen::VectorXd mPositionUpperLimits;

    /// The joint's position limits
    Eigen::Matrix<bool, Eigen::Dynamic, 1> mPositionHasLimits;

    /// The joint's velocity lower limits
    Eigen::VectorXd mVelocityLowerLimits;

    /// The joint's velocity upper limits
    Eigen::VectorXd mVelocityUpperLimits;

    /// The joint's velocity limits
    // TODO: why doesn't this exist in DART??
    // Eigen::Matrix<bool, Eigen::Dynamic, 1> mVelocityHasLimits;
  };

  /// Constructs a state space for \c joint.
  ///
  /// \param joint joint to create a \c StateSpace from
  explicit JointStateSpace(const ::dart::dynamics::Joint* joint);

  /// Destructor
  virtual ~JointStateSpace() = default;

  /// Gets the joint properties associated with this state space.
  ///
  /// \return joint properties associated with this state space
  const Properties& getProperties() const;

  /// Returns whether the Joint can be used with this state space.
  ///
  /// \param joint Joint to check
  bool isCompatible(const ::dart::dynamics::Joint* joint) const;

  /// Throws an error if the Joint cannot be used with this state space.
  ///
  /// \param joint Joint to check
  /// \throws invalid_argument if the Joint does not match the state space
  void checkCompatibility(const ::dart::dynamics::Joint* joint) const;

  /// Converts DART \c Joint positions, e.g. those returned by
  /// \c getPositions, to a \c State in this state space.
  ///
  /// \param positions input DART \c Joint positions
  /// \param[out] state output state
  virtual void convertPositionsToState(
      const Eigen::VectorXd& positions, StateSpace::State* state) const = 0;

  /// Converts a \c State in this state space to DART \c Joint positions, e.g.
  /// that may be passed to \c setPositions.
  ///
  /// \param state input state
  /// \param[out] positions output DART \c Joint positions
  virtual void convertStateToPositions(
      const StateSpace::State* state, Eigen::VectorXd& positions) const = 0;

  /// Gets the positions of the \c joint and store them in \c state.
  ///
  /// \param joint \c Joint to get position from
  /// \param[out] state output state
  virtual void getState(
      const ::dart::dynamics::Joint* joint, StateSpace::State* state) const;

  /// Sets the positions of the \c joint to \c state.
  ///
  /// \param joint \c Joint to set position for
  /// \param state input state
  virtual void setState(
      ::dart::dynamics::Joint* joint, const StateSpace::State* state) const;

protected:
  Properties mProperties;
};

} // namespace dart
} // namespace statespace
} // namespace aikido

#endif // ifndef AIKIDO_STATESPACE_DART_JOINTSTATESPACE_HPP_
