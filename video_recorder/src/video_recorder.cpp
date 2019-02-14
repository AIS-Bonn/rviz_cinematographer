/** @file
 *
 * Subscribes to images and generates a video.
 *
 * @author Jan Razlaw
 */

#include "video_recorder/video_recorder.h"

namespace video_recorder
{

VideoRecorderNodelet::VideoRecorderNodelet()
  : nh_("")
    , max_queue_size_(50)
    , path_to_output_("")
    , codec_(cv::VideoWriter::fourcc('D', 'I', 'V', 'X'))
    , target_fps_(60)
    , recorded_frames_counter_(0)
    , add_watermark_(true)
    , is_watermark_resized_(false)
{
}

void VideoRecorderNodelet::onInit()
{
  record_finished_pub_ = nh_.advertise<rviz_cinematographer_msgs::Finished>("/video_recorder/record_finished", 1);
  wait_pub_ = nh_.advertise<rviz_cinematographer_msgs::Wait>("/video_recorder/wait_duration", 1);

  record_params_sub_ = nh_.subscribe("/rviz/record", 1, &VideoRecorderNodelet::recordParamsCallback, this);
  rendering_finished_sub_ = nh_.subscribe("/rviz/finished_rendering_trajectory", 1,
                                          &VideoRecorderNodelet::renderingFinishedCallback, this);

  image_transport::ImageTransport it(nh_);
  image_sub_ = it.subscribe("/rviz/view_image", 1, &VideoRecorderNodelet::imageCallback, this);
}

void VideoRecorderNodelet::recordParamsCallback(const rviz_cinematographer_msgs::Record::ConstPtr& record_params)
{
  int max_fps = 120;
  if(record_params->compress > 0)
    codec_ = cv::VideoWriter::fourcc('D', 'I', 'V', 'X');
  else
  {
    codec_ = cv::VideoWriter::fourcc('P', 'I', 'M', '1');
    max_fps = 60;
  }

  target_fps_ = std::max(1, std::min(max_fps, (int)record_params->frames_per_second));

  path_to_output_ = record_params->path_to_output;
  add_watermark_ = record_params->add_watermark > 0;

  if(add_watermark_)
  {
    // load watermark 
    std::string path_to_watermark = ros::package::getPath("video_recorder");
    if(path_to_watermark.empty())
      NODELET_ERROR("Can't find path to video_recorder to load watermark.");
    else
    {
      path_to_watermark += "/watermark/watermark.png";
      original_watermark_ = cv::imread(path_to_watermark, cv::IMREAD_UNCHANGED);
      is_watermark_resized_ = false;
    }
  }

  // init thread to process images if not already existing
  if(!process_images_thread_)
    process_images_thread_ = boost::shared_ptr<boost::thread>(
      new boost::thread(boost::bind(&VideoRecorderNodelet::processImages, this)));
}

void
VideoRecorderNodelet::renderingFinishedCallback(const rviz_cinematographer_msgs::Finished::ConstPtr& rendering_finished)
{
  ros::Rate r(30); // 30Hz
  if(rendering_finished->is_finished)
  {
    // wait until images in queue are processed 
    while(!image_queue_.empty())
      r.sleep();

    if(output_video_.isOpened())
      output_video_.release();

    // publish that recording is finished 
    rviz_cinematographer_msgs::Finished record_finished;
    record_finished.is_finished = true;
    record_finished_pub_.publish(record_finished);
  }
}

void VideoRecorderNodelet::imageCallback(const sensor_msgs::ImageConstPtr& input_image)
{
  cv_bridge::CvImagePtr cv_image;
  try
  {
    cv_image = cv_bridge::toCvCopy(input_image, sensor_msgs::image_encodings::BGR8);
  }
  catch(cv_bridge::Exception& e)
  {
    NODELET_ERROR("Failed to convert sensor_msgs::Image to cv_bridge::CvImage : cv_bridge exception: %s", e.what());
    return;
  }

  image_queue_.push(cv_image);

  if((int)image_queue_.size() >= max_queue_size_)
  {
    NODELET_DEBUG("Max queue size exceeded. Sending wait message.");
    // publish that input has to wait until some images are processed 
    ros::WallDuration wait_duration = process_one_image_duration_ * (max_queue_size_ - (max_queue_size_ / 5));
    rviz_cinematographer_msgs::Wait wait_duration_msg;
    wait_duration_msg.seconds = static_cast<float>(wait_duration.toSec());
    wait_pub_.publish(wait_duration_msg);
  }
}

void VideoRecorderNodelet::processImages()
{
  ros::Rate r(30); // 30 hz
  while(ros::ok())
  {
    if(!image_queue_.empty())
    {
      ros::WallTime start = ros::WallTime::now();

      cv_bridge::CvImagePtr cv_ptr = image_queue_.front();

      cv::Size img_size(cv_ptr->image.cols, cv_ptr->image.rows);

      if(!output_video_.isOpened())
        if(!output_video_.open(path_to_output_, codec_, target_fps_, img_size, true))
          NODELET_ERROR_STREAM("Could not open the output video to write file in : " << path_to_output_);

      if(output_video_.isOpened())
      {
        if(add_watermark_)
        {
          // resize watermark only once per recording to better fit the video image size 
          if(!is_watermark_resized_)
          {
            original_watermark_.copyTo(resized_watermark_);
            resizeWatermark(resized_watermark_, cv_ptr->image.cols);
            is_watermark_resized_ = true;
          }

          // add watermark 
          addWatermark(cv_ptr->image, resized_watermark_);
        }
        output_video_.write(cv_ptr->image);
      }

      image_queue_.pop();

      process_one_image_duration_ = ros::WallTime::now() - start;
    }
    else
    {
      r.sleep();
    }
  }
}

void VideoRecorderNodelet::resizeWatermark(cv::Mat& watermark, const int image_width)
{
  float watermark_resize_factor = (0.5f * image_width) / watermark.cols;
  if(watermark_resize_factor < 1.f)
    cv::resize(watermark, watermark, cv::Size(), watermark_resize_factor, watermark_resize_factor);
}

void VideoRecorderNodelet::addWatermark(cv::Mat& image, const cv::Mat& watermark)
{
  int origin_watermark_row = image.rows - watermark.rows;
  int origin_watermark_col = image.cols - watermark.cols;
  int image_row = origin_watermark_row;
  int image_col = origin_watermark_col;
  float alpha = 0.8f;
  for(int watermark_row = 0; watermark_row < watermark.rows; watermark_row++, image_row++)
  {
    image_col = origin_watermark_col;
    for(int watermark_col = 0; watermark_col < watermark.cols; watermark_col++, image_col++)
    {
      // overlay if pixel in watermark is not transparent
      unsigned char pixel_alpha = watermark.at<cv::Vec4b>(watermark_row, watermark_col)[3];
      if(pixel_alpha != 0)
        for(int i = 0; i < 3; ++i)
          image.at<cv::Vec3b>(image_row, image_col)[i] = cv::saturate_cast<uchar>(
            alpha * image.at<cv::Vec3b>(image_row, image_col)[i] +
            (1.f - alpha) * watermark.at<cv::Vec4b>(watermark_row, watermark_col)[i]);

    }
  }
}

}

#include <pluginlib/class_list_macros.h>
// Register this plugin with pluginlib.  Names must match nodelet_plugin.xml.
PLUGINLIB_EXPORT_CLASS(video_recorder::VideoRecorderNodelet, nodelet::Nodelet)
