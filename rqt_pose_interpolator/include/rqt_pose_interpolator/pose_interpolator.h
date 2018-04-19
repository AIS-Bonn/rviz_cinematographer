//
// Created by razlaw on 19.04.18.
//

#ifndef RQT_POSE_INTERPOLATOR_POSE_INTERPOLATOR_H
#define RQT_POSE_INTERPOLATOR_POSE_INTERPOLATOR_H

#include <rqt_gui_cpp/plugin.h>
#include <QWidget>
#include "ui_pose_interpolator.h"

namespace pose_interpolator {

class PoseInterpolator
        : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  PoseInterpolator();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

  // Comment in to signal that the plugin has a way to configure it
  //bool hasConfiguration() const;
  //void triggerConfiguration();
private:
  Ui::pose_interpolator ui_;
  QWidget* widget_;
};
} // namespace

#endif //RQT_POSE_INTERPOLATOR_POSE_INTERPOLATOR_H
