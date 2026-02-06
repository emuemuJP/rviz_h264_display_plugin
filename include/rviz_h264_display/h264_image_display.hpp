#ifndef RVIZ_H264_DISPLAY__H264_IMAGE_DISPLAY_HPP_
#define RVIZ_H264_DISPLAY__H264_IMAGE_DISPLAY_HPP_

#ifndef Q_MOC_RUN

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <QObject>

#include <OgreMaterial.h>
#include <OgreSharedPtr.h>
#include <OgreTexture.h>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/ros_topic_display.hpp"
#include "rviz_common/render_panel.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>

#endif

namespace Ogre
{
class Rectangle2D;
}

namespace rviz_h264_display
{

class H264ImageDisplay
  : public rviz_common::RosTopicDisplay<sensor_msgs::msg::CompressedImage>
{
  Q_OBJECT

public:
  H264ImageDisplay();
  ~H264ImageDisplay() override;

  void onInitialize() override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;

protected:
  void processMessage(sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) override;
  void subscribe() override;
  void unsubscribe() override;
  void onEnable() override;
  void onDisable() override;

private:
  void initPipeline();
  void destroyPipeline();
  void decodeThread();
  void setupScreenRectangle();
  void setupRenderPanel();

  // GStreamer
  GstElement * pipeline_ = nullptr;
  GstElement * appsrc_ = nullptr;
  GstElement * appsink_ = nullptr;
  std::thread decode_thread_;
  std::atomic<bool> stop_{false};
  std::atomic<bool> pipeline_ready_{false};

  // Decoded frame buffer (decode thread -> render thread)
  std::mutex frame_mutex_;
  std::vector<uint8_t> decoded_frame_;
  int frame_width_ = 0;
  int frame_height_ = 0;
  bool new_frame_ = false;

  // Ogre rendering
  Ogre::TexturePtr texture_;
  Ogre::MaterialPtr material_;
  std::unique_ptr<Ogre::Rectangle2D> screen_rect_;
  std::unique_ptr<rviz_common::RenderPanel> render_panel_;
  bool screen_rect_initialized_ = false;

  static int instance_count_;
};

}  // namespace rviz_h264_display

#endif  // RVIZ_H264_DISPLAY__H264_IMAGE_DISPLAY_HPP_
