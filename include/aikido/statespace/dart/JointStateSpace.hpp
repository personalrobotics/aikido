#ifndef AIKIDO_STATESPACE_DART_JOINTSTATESPACE_HPP_
#define AIKIDO_STATESPACE_DART_JOINTSTATESPACE_HPP_
#include <dart/dynamics/dynamics.hpp>
#include "../StateSpace.hpp"

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
    /// Constructs the joint properties for \c _joint.
    ///
    /// \param _joint \c Joint to create properties for
    explicit Properties(::dart::dynamics::Joint* _joint);

    /// Return the name of the joint.
    const std::string getName() const;

    /// Return the type of the joint.
    const std::string getType() const;

    /// Return the number of DOFs in the joint.
    std::size_t getNumDofs() const;

    /// Return whether the index DOF is position-limited.
    bool hasPositionLimit(std::size_t index) const;

    /// Return whether any DOF is position-limited.
    bool isLimited() const;

    /// Return the vector of position lower limits.
    const Eigen::VectorXd getPositionLowerLimits() const;

    /// Return the vector of position upper limits.
    const Eigen::VectorXd getPositionUpperLimits() const;

  protected:
    /// Name of the joint
    std::string mName;

    /// Name of the joint type
    std::string mType;

    /// Number of DOFs in the joint
    std::size_t mNumDofs;

    /// The joint's position lower limits
    Eigen::VectorXd mPositionLowerLimits;

    /// The joint's position upper limits
    Eigen::VectorXd mPositionUpperLimits;

    /// The joint's position limits
    Eigen::Matrix<bool, Eigen::Dynamic, 1> mPositionHasLimits;
  };

  /// Constructs a state space for \c _joint.
  ///
  /// \param _joint joint to create a \c StateSpace for
  explicit JointStateSpace(::dart::dynamics::Joint* _joint);

  virtual ~JointStateSpace() = default;

  /// Gets the joint properties associated with this state space.
  ///
  /// \return joint properties associated with this state space
  const Properties getProperties() const;

  /// Converts DART \c Joint positions, e.g. those returned by
  /// \c getPositions, to a \c State in this state space.
  ///
  /// \param _positions input DART \c Joint positions
  /// \param[out] _state output state
  virtual void convertPositionsToState(
      const Eigen::VectorXd& _positions, StateSpace::State* _state) const = 0;

  /// Converts a \c State in this state space to DART \c Joint positions, e.g.
  /// that may be passed to \c setPositions.
  ///
  /// \param _state input state
  /// \param[out] _positions output DART \c Joint positions
  virtual void convertStateToPositions(
      const StateSpace::State* _state, Eigen::VectorXd& _positions) const = 0;

  /// Gets the positions of the \c _joint and store them in \c _state.
  ///
  /// \param _joint \c Joint to get position from
  /// \param[out] _state output state
  virtual void getState(
      const ::dart::dynamics::Joint* _joint, StateSpace::State* _state) const;

  /// Sets the positions of the \c _joint to \c _state.
  ///
  /// \param _joint \c Joint to get position from
  /// \param _state input state
  virtual void setState(
      ::dart::dynamics::Joint* _joint, const StateSpace::State* _state) const;

protected:
  Properties mProperties;
};

} // namespace dart
} // namespace statespace
} // namespace aikido

#endif // ifndef AIKIDO_STATESPACE_DART_JOINTSTATESPACE_HPP_
