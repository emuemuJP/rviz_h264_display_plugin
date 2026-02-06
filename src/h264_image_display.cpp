#include "rviz_h264_display/h264_image_display.hpp"

#include <algorithm>
#include <string>

#include <OgreHardwarePixelBuffer.h>
#include <OgreMaterialManager.h>
#include <OgreRectangle2D.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>

#include "rviz_common/display_context.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_rendering/render_window.hpp"

#include "rclcpp/rclcpp.hpp"

#include <pluginlib/class_list_macros.hpp>

static rclcpp::Logger get_logger()
{
  return rclcpp::get_logger("rviz_h264_display");
}

namespace rviz_h264_display
{

int H264ImageDisplay::instance_count_ = 0;

H264ImageDisplay::H264ImageDisplay()
{
}

H264ImageDisplay::~H264ImageDisplay()
{
  destroyPipeline();

  screen_rect_.reset();

  if (material_) {
    Ogre::MaterialManager::getSingleton().remove(material_);
    material_.reset();
  }
  if (texture_) {
    Ogre::TextureManager::getSingleton().remove(texture_);
    texture_.reset();
  }
}

void H264ImageDisplay::onInitialize()
{
  rviz_common::RosTopicDisplay<sensor_msgs::msg::CompressedImage>::onInitialize();

  instance_count_++;
  std::string mat_name = "H264ImageMaterial_" + std::to_string(instance_count_);
  std::string tex_name = "H264ImageTexture_" + std::to_string(instance_count_);

  // Create a default 8x8 texture
  texture_ = Ogre::TextureManager::getSingleton().createManual(
    tex_name,
    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
    Ogre::TEX_TYPE_2D,
    8, 8, 0,
    Ogre::PF_BYTE_RGBA,
    Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);

  // Create material
  material_ = Ogre::MaterialManager::getSingleton().create(
    mat_name,
    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  material_->setReceiveShadows(false);
  material_->getTechnique(0)->setLightingEnabled(false);
  material_->setDepthWriteEnabled(false);
  Ogre::TextureUnitState * tu =
    material_->getTechnique(0)->getPass(0)->createTextureUnitState();
  tu->setTextureName(texture_->getName());
  tu->setTextureFiltering(Ogre::TFO_NONE);

  setupRenderPanel();
  setupScreenRectangle();
}

void H264ImageDisplay::setupRenderPanel()
{
  render_panel_ = std::make_unique<rviz_common::RenderPanel>();
  render_panel_->resize(640, 480);
  render_panel_->initialize(context_);
  setAssociatedWidget(render_panel_.get());
}

void H264ImageDisplay::setupScreenRectangle()
{
  if (screen_rect_initialized_) {
    return;
  }

  auto render_window = render_panel_->getRenderWindow();
  if (!render_window) {
    RCLCPP_WARN_ONCE(get_logger(), "RenderWindow not ready yet, will retry in update()");
    return;
  }

  auto scene_manager = rviz_rendering::RenderWindowOgreAdapter::getSceneManager(render_window);
  if (!scene_manager) {
    RCLCPP_WARN_ONCE(get_logger(), "SceneManager not ready yet, will retry in update()");
    return;
  }

  screen_rect_ = std::make_unique<Ogre::Rectangle2D>(true);
  screen_rect_->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);
  screen_rect_->setMaterial(material_);
  screen_rect_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);

  Ogre::AxisAlignedBox aabb;
  aabb.setInfinite();
  screen_rect_->setBoundingBox(aabb);

  auto scene_node = scene_manager->getRootSceneNode()->createChildSceneNode();
  scene_node->attachObject(screen_rect_.get());

  screen_rect_initialized_ = true;
  RCLCPP_INFO(get_logger(), "Screen rectangle initialized successfully");
}

void H264ImageDisplay::initPipeline()
{
  if (pipeline_) {
    return;
  }

  gst_init(nullptr, nullptr);

  std::string pipeline_str =
    "appsrc name=src is-live=true block=false format=3"
    " ! h264parse"
    " ! nvv4l2decoder enable-max-performance=true disable-dpb=true"
    " ! nvvidconv"
    " ! video/x-raw,format=RGBA"
    " ! appsink name=sink max-buffers=1 drop=true sync=false emit-signals=false";

  GError * error = nullptr;
  pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);
  if (!pipeline_ || error) {
    std::string err_msg = error ? error->message : "unknown error";
    if (error) {
      g_error_free(error);
    }
    setStatus(
      rviz_common::properties::StatusProperty::Error,
      "GStreamer",
      QString::fromStdString("Pipeline creation failed: " + err_msg));
    pipeline_ = nullptr;
    return;
  }

  appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "src");
  appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");

  if (!appsrc_ || !appsink_) {
    setStatus(
      rviz_common::properties::StatusProperty::Error,
      "GStreamer",
      "Failed to get appsrc/appsink elements");
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
    return;
  }

  // Set appsrc caps
  GstCaps * caps = gst_caps_new_simple(
    "video/x-h264",
    "stream-format", G_TYPE_STRING, "byte-stream",
    "alignment", G_TYPE_STRING, "au",
    nullptr);
  gst_app_src_set_caps(GST_APP_SRC(appsrc_), caps);
  gst_caps_unref(caps);

  g_object_set(
    appsrc_,
    "stream-type", 0,   // GST_APP_STREAM_TYPE_STREAM
    "max-bytes", static_cast<guint64>(2 * 1024 * 1024),
    nullptr);

  // Set pipeline to PAUSED
  GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PAUSED);
  if (ret == GST_STATE_CHANGE_FAILURE) {
    setStatus(
      rviz_common::properties::StatusProperty::Error,
      "GStreamer",
      "Failed to set pipeline to PAUSED");
    gst_object_unref(appsrc_);
    gst_object_unref(appsink_);
    gst_object_unref(pipeline_);
    appsrc_ = nullptr;
    appsink_ = nullptr;
    pipeline_ = nullptr;
    return;
  }

  // Wait for PAUSED state (up to 5 seconds)
  GstState state;
  ret = gst_element_get_state(pipeline_, &state, nullptr, 5 * GST_SECOND);
  if (ret == GST_STATE_CHANGE_FAILURE) {
    setStatus(
      rviz_common::properties::StatusProperty::Error,
      "GStreamer",
      "Pipeline failed to reach PAUSED state");
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    gst_object_unref(appsrc_);
    gst_object_unref(appsink_);
    gst_object_unref(pipeline_);
    appsrc_ = nullptr;
    appsink_ = nullptr;
    pipeline_ = nullptr;
    return;
  }

  // Start decode thread
  stop_ = false;
  decode_thread_ = std::thread(&H264ImageDisplay::decodeThread, this);

  // Set pipeline to PLAYING
  gst_element_set_state(pipeline_, GST_STATE_PLAYING);
  pipeline_ready_ = true;

  setStatus(
    rviz_common::properties::StatusProperty::Ok,
    "GStreamer",
    "Pipeline running");
}

void H264ImageDisplay::destroyPipeline()
{
  pipeline_ready_ = false;

  if (pipeline_) {
    stop_ = true;

    // Send EOS to unblock pull_sample
    gst_element_send_event(pipeline_, gst_event_new_eos());

    if (decode_thread_.joinable()) {
      decode_thread_.join();
    }

    gst_element_set_state(pipeline_, GST_STATE_NULL);

    if (appsrc_) {
      gst_object_unref(appsrc_);
      appsrc_ = nullptr;
    }
    if (appsink_) {
      gst_object_unref(appsink_);
      appsink_ = nullptr;
    }
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
  }

  std::lock_guard<std::mutex> lock(frame_mutex_);
  decoded_frame_.clear();
  frame_width_ = 0;
  frame_height_ = 0;
  new_frame_ = false;
}

void H264ImageDisplay::decodeThread()
{
  bool first_frame = true;
  RCLCPP_INFO(get_logger(), "Decode thread started");

  while (!stop_.load()) {
    GstSample * sample = gst_app_sink_try_pull_sample(
      GST_APP_SINK(appsink_), GST_SECOND);

    if (!sample) {
      if (gst_app_sink_is_eos(GST_APP_SINK(appsink_))) {
        RCLCPP_INFO(get_logger(), "Decode thread: EOS received");
        break;
      }
      continue;
    }

    GstBuffer * buf = gst_sample_get_buffer(sample);
    if (!buf) {
      gst_sample_unref(sample);
      continue;
    }

    // Get width/height from caps on first frame
    if (first_frame) {
      GstCaps * caps = gst_sample_get_caps(sample);
      if (caps) {
        GstStructure * s = gst_caps_get_structure(caps, 0);
        int w = 0, h = 0;
        gst_structure_get_int(s, "width", &w);
        gst_structure_get_int(s, "height", &h);
        if (w > 0 && h > 0) {
          RCLCPP_INFO(get_logger(), "Decode thread: first decoded frame %dx%d", w, h);
          std::lock_guard<std::mutex> lock(frame_mutex_);
          frame_width_ = w;
          frame_height_ = h;
          first_frame = false;
        }
      }
    }

    GstMapInfo info;
    if (!gst_buffer_map(buf, &info, GST_MAP_READ)) {
      gst_sample_unref(sample);
      continue;
    }

    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      if (frame_width_ > 0 && frame_height_ > 0) {
        size_t expected = static_cast<size_t>(frame_width_) * frame_height_ * 4;  // RGBA
        size_t copy_size = std::min(info.size, expected);
        decoded_frame_.resize(expected);
        std::memcpy(decoded_frame_.data(), info.data, copy_size);
        new_frame_ = true;
      }
    }

    gst_buffer_unmap(buf, &info);
    gst_sample_unref(sample);
  }
}

void H264ImageDisplay::subscribe()
{
  if (!isEnabled()) {
    return;
  }

  if (topic_property_->isEmpty()) {
    setStatus(
      rviz_common::properties::StatusProperty::Error,
      "Topic",
      "Error subscribing: Empty topic name");
    return;
  }

  try {
    // Override QoS to match publisher: best_effort, volatile, depth=1
    rclcpp::QoS qos(1);
    qos.best_effort();
    qos.durability_volatile();

    subscription_ =
      rviz_ros_node_.lock()->get_raw_node()
      ->create_subscription<sensor_msgs::msg::CompressedImage>(
      topic_property_->getTopicStd(),
      qos,
      [this](const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {
        incomingMessage(msg);
      });
    setStatus(rviz_common::properties::StatusProperty::Ok, "Topic", "OK");
  } catch (rclcpp::exceptions::InvalidTopicNameError & e) {
    setStatus(
      rviz_common::properties::StatusProperty::Error,
      "Topic",
      QString("Error subscribing: ") + e.what());
  }
}

void H264ImageDisplay::unsubscribe()
{
  rviz_common::RosTopicDisplay<sensor_msgs::msg::CompressedImage>::unsubscribe();
  destroyPipeline();
}

void H264ImageDisplay::onEnable()
{
  initPipeline();
  subscribe();
}

void H264ImageDisplay::onDisable()
{
  unsubscribe();
  reset();
}

void H264ImageDisplay::processMessage(
  sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
{
  if (!pipeline_ready_ || !appsrc_) {
    // Try to init pipeline if not yet ready
    if (!pipeline_) {
      initPipeline();
    }
    if (!pipeline_ready_) {
      return;
    }
  }

  RCLCPP_INFO_ONCE(get_logger(),
    "processMessage: first message received, format='%s', size=%zu",
    msg->format.c_str(), msg->data.size());

  if (msg->format.find("h264") == std::string::npos) {
    setStatus(
      rviz_common::properties::StatusProperty::Warn,
      "Format",
      QString::fromStdString("Unexpected format: " + msg->format + " (expected h264)"));
    return;
  }

  if (msg->data.empty()) {
    return;
  }

  // Create GstBuffer and push into appsrc
  GstBuffer * buffer = gst_buffer_new_allocate(nullptr, msg->data.size(), nullptr);
  GstMapInfo map;
  if (gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
    std::memcpy(map.data, msg->data.data(), msg->data.size());
    gst_buffer_unmap(buffer, &map);
  }

  GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(appsrc_), buffer);
  if (ret != GST_FLOW_OK) {
    // buffer ownership is taken by push_buffer even on failure
  }
}

void H264ImageDisplay::update(float wall_dt, float ros_dt)
{
  (void)wall_dt;
  (void)ros_dt;

  // Retry screen rectangle setup if it failed during onInitialize
  if (!screen_rect_initialized_) {
    setupScreenRectangle();
  }

  std::lock_guard<std::mutex> lock(frame_mutex_);

  if (!new_frame_ || frame_width_ <= 0 || frame_height_ <= 0) {
    return;
  }
  new_frame_ = false;

  uint32_t w = static_cast<uint32_t>(frame_width_);
  uint32_t h = static_cast<uint32_t>(frame_height_);

  RCLCPP_INFO_ONCE(get_logger(), "First frame received: %ux%u", w, h);

  // Recreate texture if size changed
  if (texture_->getWidth() != w || texture_->getHeight() != h) {
    RCLCPP_INFO(get_logger(), "Resizing texture to %ux%u", w, h);
    std::string tex_name = texture_->getName();
    Ogre::TextureManager::getSingleton().remove(texture_);

    texture_ = Ogre::TextureManager::getSingleton().createManual(
      tex_name,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      Ogre::TEX_TYPE_2D,
      w, h, 0,
      Ogre::PF_BYTE_RGBA,
      Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);

    material_->getTechnique(0)->getPass(0)->getTextureUnitState(0)
    ->setTextureName(texture_->getName());
  }

  // Copy decoded frame to Ogre texture
  Ogre::HardwarePixelBufferSharedPtr pixel_buffer = texture_->getBuffer();
  pixel_buffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);
  const Ogre::PixelBox & pb = pixel_buffer->getCurrentLock();

  size_t row_bytes = w * 4;
  uint8_t * dst = static_cast<uint8_t *>(pb.data);
  const uint8_t * src = decoded_frame_.data();

  if (static_cast<size_t>(pb.rowPitch) * 4 == row_bytes) {
    // No padding, direct copy
    std::memcpy(dst, src, row_bytes * h);
  } else {
    // Row-by-row copy with pitch
    for (uint32_t y = 0; y < h; ++y) {
      std::memcpy(
        dst + y * pb.rowPitch * 4,
        src + y * row_bytes,
        row_bytes);
    }
  }

  pixel_buffer->unlock();

  context_->queueRender();
  render_panel_->update();
}

void H264ImageDisplay::reset()
{
  rviz_common::RosTopicDisplay<sensor_msgs::msg::CompressedImage>::reset();
  destroyPipeline();
}

}  // namespace rviz_h264_display

PLUGINLIB_EXPORT_CLASS(rviz_h264_display::H264ImageDisplay, rviz_common::Display)
