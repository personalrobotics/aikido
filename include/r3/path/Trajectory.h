#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_
#include <string>
#include <Eigen/Dense>

namespace r3 {
namespace path {

class Trajectory {
public:
  virtual ~Trajectory() {}

  virtual size_t order() const = 0;
  virtual double duration() const = 0;
  virtual std::string const &type() const = 0;

  virtual Eigen::VectorXd sample(double t, size_t order) const = 0;
};

}
}

#endif
