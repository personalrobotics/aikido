#ifndef AIKIDO_CONSTRAINT_COLLISIONFREEOUTCOME_HPP_
#define AIKIDO_CONSTRAINT_COLLISIONFREEOUTCOME_HPP_

#include <sstream>
#include <vector>
#include "TestableOutcome.hpp"

namespace aikido {
namespace constraint {

class CollisionFreeOutcome : public TestableOutcome {
public:
  bool isSatisfied() const;
  std::string toString() const;
  void markCollisionBodyNode(const std::string& bodyNodeName);
  void markSelfCollisionBodyNode(const std::string& bodyNodeName);

private:
  std::vector<std::string> mCollisionBodyNodes;
  std::vector<std::string> mSelfCollisionBodyNodes;
};

} // namespace constraint
} // namespace aikido

#endif // ifndef AIKIDO_CONSTRAINT_COLLISIONFREEOUTCOME_HPP_
