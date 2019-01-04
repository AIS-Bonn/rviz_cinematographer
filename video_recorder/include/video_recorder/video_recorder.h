/** @file
 *
 * Subscribes to images and generates a video.
 *
 * @author Jan Razlaw
 */

#ifndef VIDEO_RECORDER_H
#define VIDEO_RECORDER_H

#include <queue>

#include <nodelet/nodelet.h>

#include <ros/subscriber.h>
#include <ros/ros.h>
#include <ros/package.h>

#include <rviz_cinematographer_msgs/Record.h>
#include <rviz_cinematographer_msgs/Finished.h>
#include <rviz_cinematographer_msgs/Wait.h>

#include <sensor_msgs/Image.h>

#include <boost/thread.hpp>

#include <cv.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

namespace video_recorder
{

class VideoRecorderNodelet : public nodelet::Nodelet
{
public:

  VideoRecorderNodelet();
  ~VideoRecorderNodelet() = default;

protected:

  /**
   * @brief Nodelet initialization.
   */
  virtual void onInit();
  
  /** @brief Sets parameters requested with service call and initializes a thread to process incoming images.
   *
   * @params[in] record_params  specifies that a record should be made and the parameters that should be used.
   */
  void recordParamsCallback(const rviz_cinematographer_msgs::Record::ConstPtr& record_params);

  //TODO
  void renderingFinishedCallback(const rviz_cinematographer_msgs::Finished::ConstPtr& finished);

  //TODO
  void imageCallback(const sensor_msgs::ImageConstPtr& input_image);

  //TODO
  void processImages();

  // TODO resize watermark to be at most half as wide as the image
  void resizeWatermark(cv::Mat& watermark, int image_width);
  
  // TODO
  void addWatermark(cv::Mat& image, cv::Mat& watermark);

protected:

  ros::NodeHandle nh_;

  ros::Subscriber record_params_sub_;
  ros::Subscriber rendering_finished_sub_;
  
  image_transport::Subscriber image_sub_;
  std::queue<cv_bridge::CvImagePtr> image_queue_;
  int max_queue_size_;
  ros::WallDuration process_one_image_duration_;
  
  boost::shared_ptr<boost::thread> process_images_thread_;
  
  ros::Publisher record_finished_pub_;
  ros::Publisher wait_pub_;

  cv::VideoWriter output_video_;
  std::string path_to_output_;
  int codec_;
  int target_fps_;
  int recorded_frames_counter_;
  bool add_watermark_;
  cv::Mat watermark_;
  bool resized_watermark_;
};

}  // namespace video_recorder

#endif // VIDEO_RECORDER_H
