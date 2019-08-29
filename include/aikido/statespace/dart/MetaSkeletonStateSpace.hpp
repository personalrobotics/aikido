#ifndef AIKIDO_STATESPACE_DART_METASKELETONSTATESPACE_HPP_
#define AIKIDO_STATESPACE_DART_METASKELETONSTATESPACE_HPP_

#include <unordered_map>
#include <dart/dynamics/dynamics.hpp>
#include "aikido/common/pair.hpp"
#include "aikido/common/pointers.hpp"
#include "aikido/statespace/CartesianProduct.hpp"
#include "aikido/statespace/dart/JointStateSpace.hpp"

namespace aikido {
namespace statespace {
namespace dart {

AIKIDO_DECLARE_POINTERS(MetaSkeletonStateSpace)

/// \c StateSpace of a DART \c MetaSkeleton. This is a \c CartesianProduct,
/// where the i-th subspace is a \c JointStateSpace for the i-th \c Joint of
/// the \c MetaSkeleton. This class provides functions for converting between
/// \c State objects and vectors of DART joint positions.
///
/// The behavior of this class is undefined if you modify the structure of the
/// \c MetaSkeleton or its position limits after construction.
class MetaSkeletonStateSpace : public CartesianProduct
{
public:
  /// Static properties from the DART \c MetaSkeleton.
  /// These will not update if the \c MetaSkeleton is updated.
  class Properties
  {
  public:
    /// Constructs the MetaSkeleton properties for \c _metaskeleton.
    ///
    /// \param metaskeleton \c MetaSkeleton to create properties for
    explicit Properties(const ::dart::dynamics::MetaSkeleton* metaskeleton);

    /// Return the name of the MetaSkeleton.
    const std::string& getName() const;

    /// Return the number of joints in the MetaSkeleton.
    std::size_t getNumJoints() const;

    /// Return the number of DOFs in the MetaSkeleton.
    std::size_t getNumDofs() const;

    /// Return the names of DOFs in the MetaSkeleton.
    const std::vector<std::string>& getDofNames() const;

    /// Return the MetaSkeleton DOF index.
    std::size_t getDofIndex(std::size_t ijoint, std::size_t ijointdof) const;

    /// Return the MetaSkeleton DOF index.
    std::size_t getDofIndex(const std::string& dofName) const;

    /// Return the vector of position lower limits.
    const Eigen::VectorXd& getPositionLowerLimits() const;

    /// Return the vector of position upper limits.
    const Eigen::VectorXd& getPositionUpperLimits() const;

    /// Return the vector of velocity lower limits.
    const Eigen::VectorXd& getVelocityLowerLimits() const;

    /// Return the vector of velocity upper limits.
    const Eigen::VectorXd& getVelocityUpperLimits() const;

    /// Return whether two MetaSkeletonStateSpace::Properties are identical.
    /// \param otherProperties Other Properties to compare against
    bool operator==(const Properties& otherProperties) const;

    /// Return whether two MetaSkeletonStateSpace::Properties are different.
    /// \param otherProperties Other Properties to compare against
    bool operator!=(const Properties& otherProperties) const;

  protected:
    /// Name of the MetaSkeleton
    std::string mName;

    /// Number of joints in the MetaSkeleton
    std::size_t mNumJoints;

    /// Names of DOFs in the MetaSkeleton
    std::vector<std::string> mDofNames;

    /// Mapping from Joint index and Joint DOF index to MetaSkeleton DOF index
    std::unordered_map<
        std::pair<std::size_t, std::size_t>,
        std::size_t,
        aikido::common::PairHash>
        mIndexMap;

    /// The metaskeleton's position lower limits
    Eigen::VectorXd mPositionLowerLimits;

    /// The metaskeleton's position upper limits
    Eigen::VectorXd mPositionUpperLimits;

    /// The metaskeleton's velocity lower limits
    Eigen::VectorXd mVelocityLowerLimits;

    /// The metaskeleton's velocity upper limits
    Eigen::VectorXd mVelocityUpperLimits;
  };

  using CartesianProduct::ScopedState;
  using CartesianProduct::State;

  /// Constructs a state space for a DART \c MetaSkeleton.
  ///
  /// \param metaskeleton target \c MetaSkeleton
  explicit MetaSkeletonStateSpace(
      const ::dart::dynamics::MetaSkeleton* metaskeleton);

  /// Gets the MetaSkeleton properties associated with this state space.
  ///
  /// \return MetaSkeleton properties associated with this state space
  const Properties& getProperties() const;

  /// Returns whether the MetaSkeleton can be used with this state space.
  ///
  /// \param metaskeleton MetaSkeleton to check
  /// \return true if MetaSkeleton is compatible
  bool isCompatible(const ::dart::dynamics::MetaSkeleton* metaskeleton) const;

  /// Throws an error if the MetaSkeleton cannot be used with this state space.
  ///
  /// \param metaskeleton MetaSkeleton to check
  /// \throws invalid_argument if the MetaSkeleton does not match the state
  /// space
  void checkCompatibility(
      const ::dart::dynamics::MetaSkeleton* metaskeleton) const;

  /// Checks whether this \c skeleton contains all dofs defined
  /// in this state space.
  ///
  /// \throws invalid_argument if \c skeleton does not contain
  ///  all dofs if this state space.
  void checkIfContained(const ::dart::dynamics::Skeleton* skeleton) const;

  /// Gets the subspace corresponding to \c _joint in \c _metaskeleton.
  ///
  /// \tparam Space type of \c StateSpace to return
  /// \param _metaskeleton MetaSkeleton containing \c _joint
  /// \param _joint joint in \c _metaskeleton
  /// \return state space corresponding to \c _joint
  template <class Space = JointStateSpace>
  std::shared_ptr<Space> getJointSpace(
      const ::dart::dynamics::MetaSkeleton* _metaskeleton,
      const ::dart::dynamics::Joint* _joint) const;

  /// Gets the subspace corresponding to joint with index \c _index.
  ///
  /// \tparam Space type of \c StateSpace to return
  /// \param _index index of a joint in the \c MetaSkeleton
  /// \return state space corresponding to \c _joint
  template <class Space = JointStateSpace>
  std::shared_ptr<const Space> getJointSpace(std::size_t _index) const;

  /// Converts DART \c MetaSkeleton positions, e.g. those returned by
  /// \c getPositions, to a \c State in this state space.
  ///
  /// \param _positions input DART \c MetaSkeleton positions
  /// \param[out] _state output state
  void convertPositionsToState(
      const Eigen::VectorXd& _positions, State* _state) const;

  /// Converts a \c State in this state space to DART \c MetaSkeleton
  /// positions, e.g. that may be passed to \c setPositions.
  ///
  /// \param _state input state
  /// \param[out] _positions output DART \c MetaSkeleton positions
  void convertStateToPositions(
      const State* _state, Eigen::VectorXd& _positions) const;

  /// Gets the positions of the \c _metaskeleton and store them in \c _state.
  ///
  /// \param _metaskeleton \c MetaSkeleton to get position from
  /// \param[out] _state output state
  void getState(
      const ::dart::dynamics::MetaSkeleton* _metaskeleton, State* _state) const;

  /// Sets the positions of the \c _metaskeleton to \c _state.
  ///
  /// \param _metaskeleton \c MetaSkeleton to set position for
  /// \param _state input state
  void setState(
      ::dart::dynamics::MetaSkeleton* _metaskeleton, const State* _state) const;

  /// Wrapper for \c getStateFromMetaSkeleton that returns a ScopedState.
  ///
  /// \param _metaskeleton \c MetaSkeleton to get state from
  /// \return current state of the \c _metaskeleton
  ScopedState getScopedStateFromMetaSkeleton(
      const ::dart::dynamics::MetaSkeleton* _metaskeleton) const;

  /// Returns MetaSkeleton this space operates on.
  /// \param _skeleton \c Skeleton to create MetaSkeleton from.
  ::dart::dynamics::MetaSkeletonPtr getControlledMetaSkeleton(
      const ::dart::dynamics::SkeletonPtr& _skeleton) const;

private:
  Properties mProperties;
};

} // namespace dart
} // namespace statespace
} // namespace aikido

#include "detail/MetaSkeletonStateSpace-impl.hpp"

#endif // ifndef AIKIDO_STATESPACE_DART_METASKELETONSTATESPACE_HPP_
