/** @file
 *
 * Subscribes to images and generates a video.
 *
 * @author Jan Razlaw
 */

#ifndef VIDEO_RECORDER_H
#define VIDEO_RECORDER_H

#include <queue>
#include <unistd.h>

#include <nodelet/nodelet.h>

#include <ros/subscriber.h>
#include <ros/ros.h>
#include <ros/package.h>

#include <rviz_cinematographer_msgs/Record.h>
#include <rviz_cinematographer_msgs/Finished.h>
#include <rviz_cinematographer_msgs/Wait.h>

#include <sensor_msgs/Image.h>

#include <boost/thread.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio/videoio.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

namespace video_recorder
{

class VideoRecorderNodelet : public nodelet::Nodelet
{
public:

  VideoRecorderNodelet();

protected:

  /**
   * @brief Nodelet initialization.
   */
  virtual void onInit();

  /** @brief Sets requested recording parameters and initializes a thread to process incoming images.
   *
   * @params[in] record_params  specifies that a record should be made and the parameters that should be used.
   */
  void recordParamsCallback(const rviz_cinematographer_msgs::Record::ConstPtr& record_params);

  /** @brief Awaits a message indicating that the image stream ended to stop recording.
   * 
   * Waits until image_queue is processed, releases video writer and publishes that the recording is finished.
   *
   * @params[in] rendering_finished  true if image stream ended.
   */
  void renderingFinishedCallback(const rviz_cinematographer_msgs::Finished::ConstPtr& rendering_finished);

  /** @brief Stores subscribed images in queue and publishes a message if queue is too large.
   * 
   * If queue's size exceeds max_queue_size, the duration it takes to process most of the queue is computed and 
   * published. This message can be used by the source of the image stream to wait for the estimated duration.
   *
   * @params[in] input_image  subscribed image.
   */
  void imageCallback(const sensor_msgs::ImageConstPtr& input_image);

  /** @brief Feeds images from queue to video writer, optionally adding a watermark. */
  void processImages();

  /** @brief Resizes watermark to be at most half as wide as the input images.
   * 
   * @params[in,out]    watermark       the watermark being resized.
   * @params[in]        image_width     the width of the input images.
   */
  void resizeWatermark(cv::Mat& watermark, const int image_width);

  /** @brief Adds watermark to the image.
   * 
   * @params[in,out]    image       the image being watermarked.
   * @params[in]        watermark   the watermark.
   */
  void addWatermark(cv::Mat& image, const cv::Mat& watermark);

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
  cv::Mat original_watermark_;
  cv::Mat resized_watermark_;
  bool is_watermark_resized_;
};

}  // namespace video_recorder

#endif // VIDEO_RECORDER_H
