/** @file
 *
 * This ROS nodelet detects potential objects in point clouds
 *
 * @author Jan Razlaw
 */

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <object_detection/detector.h>

namespace object_detection
{
  /**
   * @brief Detector in a nodelet.
   *
   * Start the detector in a nodelet.
   *
   * @see Detector
   */
  class ObjectDetectionNodelet: public nodelet::Nodelet
  {
  public:

     ObjectDetectionNodelet() {}
    ~ObjectDetectionNodelet() {}

  private:

    virtual void onInit();
    boost::shared_ptr<Detector> detector_;
  };

  /**
   * @brief Nodelet initialization.
   */
  void ObjectDetectionNodelet::onInit()
  {
    detector_.reset(new Detector(getNodeHandle(), getPrivateNodeHandle()));
  }

} // namespace object_detection


// Register this plugin with pluginlib.  Names must match nodelet_plugin.xml.
//
// parameters: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(object_detection, ObjectDetectionNodelet,
                        object_detection::ObjectDetectionNodelet, nodelet::Nodelet);
