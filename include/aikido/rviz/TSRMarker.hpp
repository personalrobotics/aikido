#ifndef AIKIDO_RVIZ_TSRMARKER_H_
#define AIKIDO_RVIZ_TSRMARKER_H_

#include <dart/dynamics/SimpleFrame.hpp>
#include <memory>

namespace aikido {
namespace rviz {

class TSRMarker {
public:
  explicit TSRMarker(
    std::vector<std::unique_ptr<dart::dynamics::SimpleFrame>> tsrFrames);
  virtual ~TSRMarker() = default;

private:
  std::vector<std::unique_ptr<dart::dynamics::SimpleFrame>> mTsrFrames;

};

} // namespace rviz
} // namespace aikido

#endif // ifndef AIKIDO_RVIZ_TSRMARKER_H_
