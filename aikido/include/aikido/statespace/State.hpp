#ifndef AIKIDO_STATESPACE_STATE_H
#define AIKIDO_STATESPACE_STATE_H
#include <Eigen/Dense>
#include <memory>

namespace aikido {
namespace statespace {

class State{
public:
  virtual ~State() = default;
};

class UtilState : public State
{
public:
  UtilState(const Eigen::VectorXd& _q)
    : mQ(_q)
  {
  }

  UtilState(int _dim)
    : mQ(Eigen::VectorXd(_dim))
  {
  }

  UtilState(const UtilState& other) = default;
  UtilState(UtilState&& other) = default;

  UtilState& operator=(const UtilState& other) = default;
  UtilState& operator=(UtilState&& other) = default;

  virtual ~UtilState() = default;

  Eigen::VectorXd mQ;
};

} // namespace statespace
} // namespace aikido

#endif
