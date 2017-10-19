#ifndef AIKIDO_SENSOR_GAZETRACKER_HPP_
#define AIKIDO_SENSOR_GAZETRACKER_HPP_

#include <string>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <eyerectoo_stream/Journal.h>

namespace aikido {
namespace sensor {

/// Represent sensor for EyeTracker data. Wraps a ROS subscriber that reads
/// EyeRecToo data vector from publisher.
class EyeTracker
{
public:
  /// Constructor.
  /// \param _nodeHandle ROS node to wrap.
  /// \param _topicName Name of topic to subscribe to for gaze updates.
  EyeTracker(
      ::ros::NodeHandle _nodeHandle,
      const std::string& _topicName);

  struct GazeData
  {
    uint syncTimestamp;
    float gazeX;
    float gazeY;
    bool validGaze;
    uint fieldWidth;
    uint fieldHeight;
    bool arucoMarkersPresent;
    float leftPupilX;
    float leftPupilY;
    float leftPupilWidth;
    float leftPupilHeight;
    float leftPupilAngle;
    bool leftPupilValid;
    float rightPupilX;
    float rightPupilY;
    float rightPupilWidth;
    float rightPupilHeight;
    float rightPupilAngle;
    bool rightPupilValid;

    std::vector<int> arucoIDs;
    std::vector<float> arcuoXCoords;
    std::vector<float> arcuoYCoords;
  };

  GazeData getCurGazeData();

private:
  //mutable std::mutex mMutex;

  GazeData mGazeData;

  /// Callback to add a new JointState to mBuffer
  /// \param _jointState New JointState to add to mBuffer
  void gazeDataCallback(const eyerectoo_stream::Journal& _journal);

  ::ros::CallbackQueue mCallbackQueue;
  ::ros::NodeHandle mNodeHandle;
  ::ros::Subscriber mSubscriber;
};

} // namespace sensor
} // namespace aikido

#endif // ifndef AIKIDO_SENSOR_GAZETRACKER_HPP_