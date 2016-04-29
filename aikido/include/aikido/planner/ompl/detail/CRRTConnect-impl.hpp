namespace aikido {
namespace planner {
namespace ompl {

//=============================================================================
CRRTConnect::Motion::Motion()
    : root(nullptr), state(nullptr), parent(nullptr)
{

}

//=============================================================================
CRRTConnect::Motion::Motion(const ::ompl::base::SpaceInformationPtr &si)
    : root(nullptr), state(si->allocState()), parent(nullptr)
{

}

//=============================================================================
CRRTConnect::Motion::~Motion()
{

}

//=============================================================================
template <template <typename T> class NN>
void CRRTConnect::setNearestNeighbors(void)
{
  mStartTree.reset(new NN<Motion *>());
  mGoalTree.reset(new NN<Motion *>());
}

}
}
}
