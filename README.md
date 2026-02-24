# rviz_compressed_display

RViz2 display plugin for compressed images (JPEG/H.264). Decodes `sensor_msgs/msg/CompressedImage` using NVIDIA hardware decoder via GStreamer and renders directly in RViz without intermediate Image topic publishing.

## Requirements

- ROS 2 Foxy
- NVIDIA Jetson (or other platform with `nvv4l2decoder` / `nvjpegdec` support)
- GStreamer 1.0
- Qt5

### Dependencies

**Build dependencies:**

- `ament_cmake`
- `rclcpp`
- `sensor_msgs`
- `rviz_common`, `rviz_rendering`, `rviz_ogre_vendor`
- `pluginlib`
- `qtbase5-dev`
- `libgstreamer1.0-dev`
- `libgstreamer-plugins-base1.0-dev`

**Runtime dependencies:**

- `gstreamer1.0-plugins-base`
- `gstreamer1.0-plugins-bad` (provides `nvv4l2decoder`)

## Build

```bash
cd ~/ros2_foxy_ws
colcon build --packages-select rviz_compressed_display
source install/setup.bash
```

## Usage

1. Launch RViz2:
   ```bash
   rviz2
   ```

2. Add display: **Add** -> **By display type** -> `rviz_compressed_display / CompressedImage`

3. Set the **Topic** to a `sensor_msgs/msg/CompressedImage` topic publishing JPEG or H.264 encoded frames (e.g., `/cam0/jpeg`).

The plugin subscribes with **Best Effort** QoS (depth=1, volatile durability) to match typical camera publishers.

## GStreamer Pipeline

The internal decode pipeline is auto-selected based on the `format` field:

### JPEG (HW)
```
appsrc ! jpegparse ! nvjpegdec ! nvvidconv ! video/x-raw,format=RGBA ! appsink
```

### H.264 (HW)
```
appsrc ! h264parse ! nvv4l2decoder ! nvvidconv ! video/x-raw,format=RGBA ! appsink
```

Decoded RGBA frames are uploaded to an Ogre texture and rendered in a dedicated `RenderPanel`.

## License

BSD
