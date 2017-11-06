#include <aikido/sensor/EyeTracker.hpp>

namespace aikido {
namespace sensor {

//==============================================================================
EyeTracker::EyeTracker(
    ::ros::NodeHandle _nodeHandleData,
      ::ros::NodeHandle _nodeHandleImage,
      const std::string& _dataTopicName,
      const std::string& _imageTopicName)
  : mDataNodeHandle{std::move(_nodeHandleData)},
    mImageNodeHandle{std::move(_nodeHandleImage)}
{
  mDataNodeHandle.setCallbackQueue(&mDataCallbackQueue);
  mDataSubscriber = mDataNodeHandle.subscribe(
      _dataTopicName, 1, &EyeTracker::gazeDataCallback, this);

  mImageNodeHandle.setCallbackQueue(&mImageCallbackQueue);
  mImageSubscriber = mImageNodeHandle.subscribe(
      _imageTopicName, 1, &EyeTracker::gazeImageCallback, this);
}

//==============================================================================
EyeTracker::GazeData EyeTracker::getCurGazeData()
{
  while (mDataCallbackQueue.empty()) {
    // Wait
  }
  mDataCallbackQueue.callAvailable();
  return mGazeData;
}

//==============================================================================
cv::Mat EyeTracker::getCurGazeImage()
{
  while (mImageCallbackQueue.empty()) {
    // Wait
  }
  mImageCallbackQueue.callAvailable();
  return mGazeImage;
}


//==============================================================================
void EyeTracker::gazeDataCallback(
  const eyerectoo_stream::Journal& _journal
) {
  // TODO: Remove all these ugly print statements once debugging is done.
  // TODO: Some form of sanity check on data that comes in.
  mGazeData.syncTimestamp = _journal.sync_timestamp;
  mGazeData.gazeX = _journal.gaze_x;
  mGazeData.gazeY = _journal.gaze_y;
  mGazeData.validGaze = _journal.valid_gaze;
  mGazeData.fieldWidth = _journal.field_width;
  mGazeData.fieldHeight = _journal.field_height;
  mGazeData.arucoMarkersPresent = _journal.aruco_markers_present;
  mGazeData.leftPupilX = _journal.left_pupil_x;
  mGazeData.leftPupilY = _journal.left_pupil_y;
  mGazeData.leftPupilWidth = _journal.left_pupil_width;
  mGazeData.leftPupilHeight = _journal.left_pupil_height;
  mGazeData.leftPupilAngle = _journal.left_pupil_angle;
  mGazeData.leftPupilValid = _journal.left_pupil_valid;
  mGazeData.rightPupilX = _journal.right_pupil_x;
  mGazeData.rightPupilY = _journal.right_pupil_y;
  mGazeData.rightPupilWidth = _journal.right_pupil_width;
  mGazeData.rightPupilHeight = _journal.right_pupil_height;
  mGazeData.rightPupilAngle = _journal.right_pupil_angle;
  mGazeData.rightPupilValid = _journal.right_pupil_valid;

  std::cout << "" << std::endl;
  std::cout << mGazeData.syncTimestamp << std::endl;
  std::cout << mGazeData.gazeX << std::endl;
  std::cout << mGazeData.gazeY << std::endl;
  std::cout << mGazeData.validGaze << std::endl;
  std::cout << mGazeData.fieldWidth << std::endl;
  std::cout << mGazeData.fieldHeight << std::endl;
  std::cout << mGazeData.arucoMarkersPresent << std::endl;
  std::cout << mGazeData.leftPupilX << std::endl;
  std::cout << mGazeData.leftPupilY << std::endl;
  std::cout << mGazeData.leftPupilWidth << std::endl;
  std::cout << mGazeData.leftPupilHeight << std::endl;
  std::cout << mGazeData.leftPupilAngle << std::endl;
  std::cout << mGazeData.leftPupilValid << std::endl;
  std::cout << mGazeData.rightPupilX << std::endl;
  std::cout << mGazeData.rightPupilY << std::endl;
  std::cout << mGazeData.rightPupilWidth << std::endl;
  std::cout << mGazeData.rightPupilHeight << std::endl;
  std::cout << mGazeData.rightPupilAngle << std::endl;
  std::cout << mGazeData.rightPupilValid << std::endl;

  if (mGazeData.arucoMarkersPresent) {
    mGazeData.arucoIDs.clear();
    mGazeData.arcuoXCoords.clear();
    mGazeData.arcuoYCoords.clear();

    // Parse the string representing gaze markers that were
    // found in the scene..
    std::string arucoMarkers = _journal.aruco_markers;
    std::stringstream arucoStream(arucoMarkers);
    std::string segment;
    std::vector<std::string> markerStrings;
    // ";" delimits individual markers.
    while(std::getline(arucoStream, segment, ';'))
    {
       markerStrings.push_back(segment);
    }

    // ":" delimits aruco marker ID from coordinates.
    for (std::string curMarkerString : markerStrings) {
      std::stringstream curMarkerStream(curMarkerString);
      std::string markerCoordsString;

      int i = 0;
      while (std::getline(curMarkerStream, segment, ':')) {
        if (i == 0) {
          int curmarkerID = std::stoi(segment);
          mGazeData.arucoIDs.push_back(curmarkerID);
        } else {
          markerCoordsString = segment;
        }
        i++;
      }

      // "x" delimits coordinates of the marker. 
      // NOTE: only grab the first 2 (x and y).
      std::stringstream markerCoordsStream(markerCoordsString);
      int j = 0;
      while (std::getline(markerCoordsStream, segment, 'x')) {
        float curCoord = std::stof(segment);
        if (j == 0) {
          mGazeData.arcuoXCoords.push_back(curCoord);
        } else if (j == 1) {
          mGazeData.arcuoYCoords.push_back(curCoord);
        } else {
          break;
        }
        j++;
      }

    }

    std::cout << "EYE COORDS" << std::endl;
    std::cout << mGazeData.arucoIDs[0] << std::endl;
    std::cout << mGazeData.arcuoXCoords[0] << std::endl;
    std::cout << mGazeData.arcuoYCoords[0] << std::endl;
  }


  std::cout << "" << std::endl;

  
}

//==============================================================================
void EyeTracker::gazeImageCallback(
  const sensor_msgs::ImageConstPtr& msg
) {

  // TODO: Instead of just one data point, keep queues of them over time.
  mGazeImage = cv_bridge::toCvShare(msg, "bgr8")->image;

  }





} // namespace sensor
} // namespace aikido


int main(int argc, char** argv)
{
  ros::init(argc, argv, "EyeTracker");

  ros::NodeHandle n_data;
  ros::NodeHandle n_image;
  std::string dataTopicName = "eyerectoo_journal";
  std::string imageTopicName = "EyeRecTooImage";
  aikido::sensor::EyeTracker testTracker(n_data, n_image, dataTopicName, imageTopicName);

  usleep(5000000);
  uint curStamp = testTracker.getCurGazeData().syncTimestamp;
  uint curStamp2 = testTracker.getCurGazeData().syncTimestamp;
  uint curStamp3 = testTracker.getCurGazeData().syncTimestamp;
  std::cout << "CUR STAMP" << std::endl;
  std::cout << curStamp << std::endl;

  cv::Mat curGazeImage = testTracker.getCurGazeImage();

  std::cout << "CUR WIDTH" << std::endl;
  std::cout << curGazeImage.cols << std::endl;

  std::cout << "CUR HEIGHT" << std::endl;
  std::cout << curGazeImage.rows << std::endl;
}

