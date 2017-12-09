#ifndef AIKIDO_PLANNER_VECTORFIELD_BODYNODEPOSEVECTORFIELDCONSTRAINT_HPP_
#define AIKIDO_PLANNER_VECTORFIELD_BODYNODEPOSEVECTORFIELDCONSTRAINT_HPP_

#include <aikido/constraint/Testable.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {
namespace detail {

/// This class implements a constraint for body node pose vector field.
/// It checks a given state constraint and a bound constraint of state space.
///
class BodyNodePoseVectorFieldConstraint : public aikido::constraint::Testable
{
public:
    /// Constructor.
    ///
    /// \param[in] stateSpace State space.
    /// \param[in] stateConstraint State constraint.
    BodyNodePoseVectorFieldConstraint(
            aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace,
            aikido::constraint::TestablePtr stateConstraint);

    // Documentation inherited.
    virtual bool isSatisfied(
        const statespace::StateSpace::State* _state) const;

    // Documentation inherited.
    virtual statespace::StateSpacePtr getStateSpace() const;

protected:
    aikido::statespace::StateSpacePtr mStateSpace;
    aikido::constraint::TestablePtr mStateConstraint;
    aikido::constraint::TestablePtr mStateSpaceBoundConstraint;
};

} // namespace detail
} // namespace vectorfield
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_VECTORFIELD_BODYNODEPOSEVECTORFIELDCONSTRAINT_HPP_
