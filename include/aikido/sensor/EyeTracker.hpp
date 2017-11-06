#ifndef AIKIDO_SENSOR_GAZETRACKER_HPP_
#define AIKIDO_SENSOR_GAZETRACKER_HPP_

#include <string>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <eyerectoo_stream/Journal.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

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
      ::ros::NodeHandle _nodeHandleData,
      ::ros::NodeHandle _nodeHandleImage,
      const std::string& _dataTopicName,
      const std::string& _imageTopicName);

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

  cv::Mat getCurGazeImage();


private:
  //mutable std::mutex mMutex;

  GazeData mGazeData;
  cv::Mat mGazeImage;

  void gazeDataCallback(const eyerectoo_stream::Journal& _journal);

  void gazeImageCallback(const sensor_msgs::ImageConstPtr& msg);

  ::ros::CallbackQueue mDataCallbackQueue;
  ::ros::CallbackQueue mImageCallbackQueue;

  ::ros::NodeHandle mDataNodeHandle;
  ::ros::Subscriber mDataSubscriber;
  ::ros::NodeHandle mImageNodeHandle;
  ::ros::Subscriber mImageSubscriber;
};

} // namespace sensor
} // namespace aikido

#endif // ifndef AIKIDO_SENSOR_GAZETRACKER_HPP_