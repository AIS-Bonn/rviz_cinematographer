/** @file
 *
 * This ROS nodelet stores and maintains a representation of the dynamics in the environment
 *
 * @author Jan Razlaw
 */

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <dynamics_mapping/dynamics_mapper.h>

namespace dynamics_mapping
{
  /**
   * @brief DynamicsMapper in a nodelet.
   *
   * Start the DynamicsMapper in a nodelet.
   *
   * @see DynamicsMapper
   */
  class DynamicsMappingNodelet: public nodelet::Nodelet
  {
  public:

     DynamicsMappingNodelet() {}
    ~DynamicsMappingNodelet() {}

  private:

    virtual void onInit();
    boost::shared_ptr<DynamicsMapper> dynamics_mapper_;
  };

  /**
   * @brief Nodelet initialization.
   */
  void DynamicsMappingNodelet::onInit()
  {
    dynamics_mapper_.reset(new DynamicsMapper(getNodeHandle(), getPrivateNodeHandle()));
  }

} // namespace dynamics_mapping


// Register this plugin with pluginlib.  Names must match nodelet_plugin.xml.
//
// parameters: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(dynamics_mapping, DynamicsMappingNodelet,
                        dynamics_mapping::DynamicsMappingNodelet, nodelet::Nodelet);
