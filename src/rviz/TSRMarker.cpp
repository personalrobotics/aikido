#include "aikido/rviz/TSRMarker.hpp"

using dart::dynamics::SimpleFrame;

namespace aikido {
namespace rviz {

//==============================================================================
TSRMarker::TSRMarker(std::vector<std::unique_ptr<SimpleFrame>> tsrFrames)
  : mTsrFrames(std::move(tsrFrames))
{
  // Do nothing
}

} // namespace rviz
} // namespace aikido
