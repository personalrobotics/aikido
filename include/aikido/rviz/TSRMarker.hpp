#ifndef AIKIDO_RVIZ_TSRMARKER_HPP_
#define AIKIDO_RVIZ_TSRMARKER_HPP_

#include <memory>

#include <dart/dynamics/SimpleFrame.hpp>
#include "aikido/common/pointers.hpp"

namespace aikido {
namespace rviz {

AIKIDO_DECLARE_POINTERS(TSRMarker)

class TSRMarker
{
public:
  explicit TSRMarker(
      std::vector<std::unique_ptr<dart::dynamics::SimpleFrame>> tsrFrames);
  virtual ~TSRMarker() = default;

private:
  std::vector<std::unique_ptr<dart::dynamics::SimpleFrame>> mTsrFrames;
};

} // namespace rviz
} // namespace aikido

#endif // ifndef AIKIDO_RVIZ_TSRMARKER_HPP_
