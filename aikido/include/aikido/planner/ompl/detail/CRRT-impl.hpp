namespace aikido {
namespace planner {
namespace ompl {

//=============================================================================
template <template <typename T> class NN> 
void CRRT::setNearestNeighbors() 
{
  mNN.reset(new NN<Motion *>());
}

//=============================================================================
CRRT::Motion::Motion() : state(nullptr), parent(nullptr) 
{

}

//=============================================================================
CRRT::Motion::Motion(const ::ompl::base::SpaceInformationPtr &_si)
    : state(_si->allocState()), parent(nullptr) {}

//=============================================================================
CRRT::Motion::~Motion(void) {}

} // namespace ompl
} // namespace planner
} // namespace aikido
