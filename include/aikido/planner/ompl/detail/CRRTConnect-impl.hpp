namespace aikido {
namespace planner {
namespace ompl {

//==============================================================================
template <template <typename T> class NN>
void CRRTConnect::setNearestNeighbors()
{
  mStartTree.reset(new NN<CRRT::Motion*>());
  mGoalTree.reset(new NN<CRRT::Motion*>());
}

} // namespace ompl
} // namespace planner
} // namespace aikido
