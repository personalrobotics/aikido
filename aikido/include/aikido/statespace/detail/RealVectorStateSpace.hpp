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
  : public statespace::StateHandle<RealVectorStateSpace, _QualifiedState>
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

  RealVectorStateHandle(const StateSpace* _space, QualifiedState* _state)
    : statespace::StateHandle<StateSpace, QualifiedState>(_space, _state)
  {
  }

  Eigen::Map<const Eigen::VectorXd> getValue() 
  {
    return this->getStateSpace()->getValue(this->getState());
  }

  void setValue(const Eigen::VectorXd& _value)
  {
    return this->getStateSpace()->setValue(this->getState(), _value);
  }
};

} // namespace statespace
} // namespace aikido
