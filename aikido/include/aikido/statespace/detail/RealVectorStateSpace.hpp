#include <type_traits>

namespace aikido {
namespace statespace {

template <class T, bool Condition>
struct add_const_if {};

template <class T>
struct add_const_if<T, false>
{
  using type = T;
};

template <class T>
struct add_const_if<T, true>
{
  using type = typename std::add_const<T>::type;
};


template <class _QualifiedState>
class RealVectorStateHandle 
  : public statespace::StateHandle<RealVectorStateSpace, RealVectorStateSpace::State>
{
public:
  using typename statespace::StateHandle<
    RealVectorStateSpace, _QualifiedState>::State;
  using typename statespace::StateHandle<
    RealVectorStateSpace, _QualifiedState>::StateSpace;
  using typename statespace::StateHandle<
    RealVectorStateSpace, _QualifiedState>::QualifiedState;

  using ValueType = typename add_const_if<
    Eigen::VectorXd, std::is_const<QualifiedState>::value>::type;

  RealVectorStateHandle()
  {
  }

  RealVectorStateHandle(const StateSpace* _space, State* _state)
    : statespace::StateHandle<StateSpace, QualifiedState>(_space, _state)
  {
  }

  Eigen::Map<ValueType> getValue() 
  {
    return getStateSpace()->getValue(**this);
  }
};

} // namespace statespace
} // namespace aikido
