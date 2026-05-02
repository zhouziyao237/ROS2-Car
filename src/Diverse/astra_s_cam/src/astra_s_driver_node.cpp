#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <OpenNI.h>

#include <atomic>
#include <chrono>
#include <cstring>
#include <memory>
#include <string>

using namespace std::chrono_literals;

class AstraSDriverNode : public rclcpp::Node
{
public:
  AstraSDriverNode() : Node("astra_s_driver_node")
  {
    camera_name_ = declare_parameter<std::string>("camera_name", "Astra_Orbbec");

    rgb_topic_ = declare_parameter<std::string>("rgb_topic", "/camera/rgb/image_raw");
    depth_topic_ = declare_parameter<std::string>("depth_topic", "/camera/depth/image_raw");
    rgb_info_topic_ = declare_parameter<std::string>("rgb_camera_info_topic", "/camera/rgb/camera_info");
    depth_info_topic_ = declare_parameter<std::string>("depth_camera_info_topic", "/camera/depth/camera_info");

    camera_link_frame_ = declare_parameter<std::string>("camera_link_frame", "camera_link");
    rgb_frame_ = declare_parameter<std::string>("rgb_frame", "camera_rgb_optical_frame");
    depth_frame_ = declare_parameter<std::string>("depth_frame", "camera_depth_optical_frame");

    enable_rgb_ = declare_parameter<bool>("enable_rgb", true);
    enable_depth_ = declare_parameter<bool>("enable_depth", true);

    rgb_width_ = declare_parameter<int>("rgb_width", 640);
    rgb_height_ = declare_parameter<int>("rgb_height", 480);
    rgb_fps_ = declare_parameter<int>("rgb_fps", 30);

    depth_width_ = declare_parameter<int>("depth_width", 640);
    depth_height_ = declare_parameter<int>("depth_height", 480);
    depth_fps_ = declare_parameter<int>("depth_fps", 30);

    fx_ = declare_parameter<double>("fx", 570.342);
    fy_ = declare_parameter<double>("fy", 570.342);
    cx_ = declare_parameter<double>("cx", 319.5);
    cy_ = declare_parameter<double>("cy", 239.5);

    depth_fx_ = declare_parameter<double>("depth_fx", 570.342);
    depth_fy_ = declare_parameter<double>("depth_fy", 570.342);
    depth_cx_ = declare_parameter<double>("depth_cx", 319.5);
    depth_cy_ = declare_parameter<double>("depth_cy", 239.5);

    double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 30.0);
    if (publish_rate_hz <= 0.0) {
      publish_rate_hz = 30.0;
    }

    auto qos = rclcpp::SensorDataQoS();

    rgb_pub_ = create_publisher<sensor_msgs::msg::Image>(rgb_topic_, qos);
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>(depth_topic_, qos);
    rgb_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(rgb_info_topic_, qos);
    depth_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(depth_info_topic_, qos);

    initOpenNI();

    auto period = std::chrono::duration<double>(1.0 / publish_rate_hz);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&AstraSDriverNode::captureAndPublish, this));

    RCLCPP_INFO(get_logger(), "astra_s_cam driver started.");
    RCLCPP_INFO(get_logger(), "RGB topic: %s", rgb_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Depth topic: %s", depth_topic_.c_str());
  }

  ~AstraSDriverNode()
  {
    RCLCPP_INFO(get_logger(), "Stopping Astra S driver...");
    if (color_stream_.isValid()) {
      color_stream_.stop();
      color_stream_.destroy();
    }
    if (depth_stream_.isValid()) {
      depth_stream_.stop();
      depth_stream_.destroy();
    }
    if (device_.isValid()) {
      device_.close();
    }
    openni::OpenNI::shutdown();
  }

private:
  void checkStatus(openni::Status status, const std::string & action)
  {
    if (status != openni::STATUS_OK) {
      std::string err = openni::OpenNI::getExtendedError();
      throw std::runtime_error(action + " failed: " + err);
    }
  }

  void initOpenNI()
  {
    checkStatus(openni::OpenNI::initialize(), "OpenNI initialize");

    openni::Array<openni::DeviceInfo> device_list;
    openni::OpenNI::enumerateDevices(&device_list);

    if (device_list.getSize() <= 0) {
      RCLCPP_WARN(get_logger(), "OpenNI2 reports 0 devices. Trying ANY_DEVICE anyway.");
    } else {
      for (int i = 0; i < device_list.getSize(); ++i) {
        const auto & info = device_list[i];
        RCLCPP_INFO(
          get_logger(),
          "OpenNI device[%d]: name=%s uri=%s vendor=%s",
          i, info.getName(), info.getUri(), info.getVendor());
      }
    }

    checkStatus(device_.open(openni::ANY_DEVICE), "Open device");
    RCLCPP_INFO(get_logger(), "Opened Astra S/OpenNI2 device.");

    if (enable_depth_) {
      checkStatus(depth_stream_.create(device_, openni::SENSOR_DEPTH), "Create depth stream");

      openni::VideoMode mode = depth_stream_.getVideoMode();
      mode.setResolution(depth_width_, depth_height_);
      mode.setFps(depth_fps_);
      mode.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);

      auto s = depth_stream_.setVideoMode(mode);
      if (s != openni::STATUS_OK) {
        RCLCPP_WARN(
          get_logger(),
          "Set depth mode %dx%d@%d failed, using default mode. Error: %s",
          depth_width_, depth_height_, depth_fps_, openni::OpenNI::getExtendedError());
      }

      checkStatus(depth_stream_.start(), "Start depth stream");
      openni::VideoMode actual = depth_stream_.getVideoMode();
      RCLCPP_INFO(
        get_logger(),
        "Depth stream started: %dx%d@%d, pixel_format=%d",
        actual.getResolutionX(), actual.getResolutionY(), actual.getFps(), actual.getPixelFormat());
    }

    if (enable_rgb_) {
      checkStatus(color_stream_.create(device_, openni::SENSOR_COLOR), "Create color stream");

      openni::VideoMode mode = color_stream_.getVideoMode();
      mode.setResolution(rgb_width_, rgb_height_);
      mode.setFps(rgb_fps_);
      mode.setPixelFormat(openni::PIXEL_FORMAT_RGB888);

      auto s = color_stream_.setVideoMode(mode);
      if (s != openni::STATUS_OK) {
        RCLCPP_WARN(
          get_logger(),
          "Set RGB mode %dx%d@%d failed, using default mode. Error: %s",
          rgb_width_, rgb_height_, rgb_fps_, openni::OpenNI::getExtendedError());
      }

      checkStatus(color_stream_.start(), "Start color stream");
      openni::VideoMode actual = color_stream_.getVideoMode();
      RCLCPP_INFO(
        get_logger(),
        "Color stream started: %dx%d@%d, pixel_format=%d",
        actual.getResolutionX(), actual.getResolutionY(), actual.getFps(), actual.getPixelFormat());
    }

    // Try to enable image registration if supported. If unsupported, keep running.
    if (enable_rgb_ && enable_depth_ &&
        device_.isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR)) {
      auto s = device_.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
      if (s == openni::STATUS_OK) {
        RCLCPP_INFO(get_logger(), "Depth-to-color registration enabled.");
      } else {
        RCLCPP_WARN(get_logger(), "Depth-to-color registration not enabled: %s", openni::OpenNI::getExtendedError());
      }
    }
  }

  sensor_msgs::msg::CameraInfo makeCameraInfo(
    const rclcpp::Time & stamp,
    const std::string & frame_id,
    int width,
    int height,
    double fx,
    double fy,
    double cx,
    double cy)
  {
    sensor_msgs::msg::CameraInfo info;
    info.header.stamp = stamp;
    info.header.frame_id = frame_id;
    info.width = width;
    info.height = height;
    info.distortion_model = "plumb_bob";
    info.d = {0.0, 0.0, 0.0, 0.0, 0.0};

    info.k = {
      fx, 0.0, cx,
      0.0, fy, cy,
      0.0, 0.0, 1.0
    };

    info.r = {
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0
    };

    info.p = {
      fx, 0.0, cx, 0.0,
      0.0, fy, cy, 0.0,
      0.0, 0.0, 1.0, 0.0
    };

    return info;
  }

  void captureAndPublish()
  {
    auto stamp = now();

    if (enable_depth_ && depth_stream_.isValid()) {
      openni::VideoFrameRef frame;
      openni::Status s = depth_stream_.readFrame(&frame);

      if (s == openni::STATUS_OK && frame.isValid()) {
        int width = frame.getWidth();
        int height = frame.getHeight();

        sensor_msgs::msg::Image msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = depth_frame_;
        msg.height = height;
        msg.width = width;
        msg.encoding = "16UC1";
        msg.is_bigendian = false;
        msg.step = width * sizeof(uint16_t);
        msg.data.resize(msg.step * height);

        const void * data = frame.getData();
        std::memcpy(msg.data.data(), data, msg.data.size());

        depth_pub_->publish(msg);
        depth_info_pub_->publish(makeCameraInfo(stamp, depth_frame_, width, height, depth_fx_, depth_fy_, depth_cx_, depth_cy_));
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Failed to read depth frame: %s", openni::OpenNI::getExtendedError());
      }
    }

    if (enable_rgb_ && color_stream_.isValid()) {
      openni::VideoFrameRef frame;
      openni::Status s = color_stream_.readFrame(&frame);

      if (s == openni::STATUS_OK && frame.isValid()) {
        int width = frame.getWidth();
        int height = frame.getHeight();

        sensor_msgs::msg::Image msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = rgb_frame_;
        msg.height = height;
        msg.width = width;
        msg.encoding = "rgb8";
        msg.is_bigendian = false;
        msg.step = width * 3;
        msg.data.resize(msg.step * height);

        const void * data = frame.getData();
        std::memcpy(msg.data.data(), data, msg.data.size());

        rgb_pub_->publish(msg);
        rgb_info_pub_->publish(makeCameraInfo(stamp, rgb_frame_, width, height, fx_, fy_, cx_, cy_));
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Failed to read RGB frame: %s", openni::OpenNI::getExtendedError());
      }
    }
  }

private:
  std::string camera_name_;
  std::string rgb_topic_;
  std::string depth_topic_;
  std::string rgb_info_topic_;
  std::string depth_info_topic_;
  std::string camera_link_frame_;
  std::string rgb_frame_;
  std::string depth_frame_;

  bool enable_rgb_{true};
  bool enable_depth_{true};

  int rgb_width_{640};
  int rgb_height_{480};
  int rgb_fps_{30};

  int depth_width_{640};
  int depth_height_{480};
  int depth_fps_{30};

  double fx_{570.342};
  double fy_{570.342};
  double cx_{319.5};
  double cy_{239.5};

  double depth_fx_{570.342};
  double depth_fy_{570.342};
  double depth_cx_{319.5};
  double depth_cy_{239.5};

  openni::Device device_;
  openni::VideoStream color_stream_;
  openni::VideoStream depth_stream_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rgb_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<AstraSDriverNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    fprintf(stderr, "astra_s_driver_node exception: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
