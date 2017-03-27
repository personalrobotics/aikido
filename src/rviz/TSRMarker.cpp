#include <aikido/rviz/TSRMarker.hpp>

using boost::format;
using boost::str;

namespace aikido {
namespace rviz {

TSRMarker::TSRMarker()
{

}

void TSRMarker::addFrameMarker(FrameMarkerPtr const &marker)
{
  mFrameMarkers.insert(marker);
}

} // namespace rviz
} // namespace aikido
