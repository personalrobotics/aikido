#include <aikido/sensor/EyeTracker.hpp>

namespace aikido {
namespace sensor {

//==============================================================================
EyeTracker::EyeTracker(
    ::ros::NodeHandle _nodeHandle,
    const std::string& _topicName)
  : mCallbackQueue{} // Must be after mNodeHandle for order of destruction.
  , mNodeHandle{std::move(_nodeHandle)}
{
  mNodeHandle.setCallbackQueue(&mCallbackQueue);
  mSubscriber = mNodeHandle.subscribe(
      _topicName, 1, &EyeTracker::gazeDataCallback, this);
}

//==============================================================================
EyeTracker::GazeData EyeTracker::getCurGazeData()
{
  ros::spinOnce();
  return mGazeData;
}

//==============================================================================
void EyeTracker::gazeDataCallback(
  const eyerectoo_stream::Journal& _journal
) {
  // TODO: Handle concurrency with Mutex?

  // TODO: Some form of sanity check on data that comes in.
  uint curSyncTimeStamp = _journal.sync_timestamp;
  mGazeData.syncTimestamp = curSyncTimeStamp;
}

} // namespace sensor
} // namespace aikido