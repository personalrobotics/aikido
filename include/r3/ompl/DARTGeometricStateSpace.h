#ifndef R3_OMPL_DARTGEOMETRICSTATESPACE_H_
#define R3_OMPL_DARTGEOMETRICSTATESPACE_H_
#include <vector>
#include <Eigen/Dense>
#include <boost/container/vector.hpp>
#include <boost/unordered_set.hpp>
#include <boost/shared_ptr.hpp>
#include <ompl/base/StateSpace.h>

namespace dart {

namespace collision {
    class CollisionDetector;
} // namespace dart::collision

namespace dynamics {
    class DegreeOfFreedom;
    class Skeleton;
} // namespace dart::dynamics

} // namespace dart


namespace r3 {
namespace ompl {

class DARTGeometricStateSpace : public ::ompl::base::CompoundStateSpace {
public:
    DARTGeometricStateSpace(
        ::std::vector<::dart::dynamics::DegreeOfFreedom *> const &dofs,
        ::Eigen::VectorXd const &weights,
        ::Eigen::VectorXd const &resolutions,
        ::dart::collision::CollisionDetector *collision_detector);

    void SetState(StateType const *state);
    void GetState(StateType *state) const;

    void CreateState(Eigen::VectorXd const &dof_values,
                     StateType *state) const;

    bool IsInCollision();

private:
    ::std::vector<::dart::dynamics::DegreeOfFreedom *> dofs_;
    ::boost::container::vector<bool> is_circular_;
    ::boost::unordered_set<::dart::dynamics::Skeleton *> skeletons_;
    ::dart::collision::CollisionDetector *collision_detector_;

    static bool IsDOFCircular(::dart::dynamics::DegreeOfFreedom const *dof);
};

typedef boost::shared_ptr<DARTGeometricStateSpace> DARTGeometricStateSpacePtr;

} // namespace r3::ompl
} // namespace r3

#endif
