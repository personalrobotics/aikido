namespace aikido {
namespace planner {
namespace ompl {

//=============================================================================
template <template <typename T> class NN>
void CRRTConnect::setNearestNeighbors(void)
{
  mStartTree.reset(new NN<CRRT::Motion *>());
  mGoalTree.reset(new NN<CRRT::Motion *>());
}

}
}
}
