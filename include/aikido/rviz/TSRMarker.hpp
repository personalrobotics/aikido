#ifndef AIKIDO_RVIZ_TSRMARKER_H_
#define AIKIDO_RVIZ_TSRMARKER_H_

#include <set>
#include "FrameMarker.hpp"
#include "SmartPointers.hpp"
#include <dart/dynamics/Frame.hpp>
#include <dart/dynamics/SmartPointer.hpp>


namespace aikido {
namespace rviz {

class TSRMarker {
public:
  explicit TSRMarker(
    std::vector<std::shared_ptr<dart::dynamics::SimpleFrame>> tsrFrames);
  virtual ~TSRMarker() = default;

private:
  std::vector<std::shared_ptr<dart::dynamics::SimpleFrame>> mTsrFrames;

};

} // namespace rviz
} // namespace aikido

#endif // ifndef AIKIDO_RVIZ_TSRMARKER_H_
