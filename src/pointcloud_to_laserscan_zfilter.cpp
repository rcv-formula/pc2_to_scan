#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <limits>
#include <cmath>
#include <string>

class PointCloudToLaserScanZFilter : public rclcpp::Node
{
public:
  PointCloudToLaserScanZFilter()
  : Node("pointcloud_to_laserscan_zfilter")
  {
    // ---- Parameters ----
    input_topic_  = this->declare_parameter<std::string>("input_topic", "/livox/lidar");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/scan");

    min_z_ = this->declare_parameter<double>("min_z", -0.2);
    max_z_ = this->declare_parameter<double>("max_z",  0.2);

    angle_min_ = this->declare_parameter<double>("angle_min", -M_PI);
    angle_max_ = this->declare_parameter<double>("angle_max",  M_PI);
    angle_increment_ = this->declare_parameter<double>("angle_increment", 0.005); // rad

    range_min_ = this->declare_parameter<double>("range_min", 0.05);
    range_max_ = this->declare_parameter<double>("range_max", 50.0);

    use_inf_ = this->declare_parameter<bool>("use_inf", true);
    inf_epsilon_ = this->declare_parameter<double>("inf_epsilon", 1.0);

    target_frame_ = this->declare_parameter<std::string>("target_frame", "");

    // ---- Pub/Sub ----
    rclcpp::QoS qos{rclcpp::SensorDataQoS()};
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, qos,
      std::bind(&PointCloudToLaserScanZFilter::cloudCallback, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, qos);

    on_set_params_cb_ = this->add_on_set_parameters_callback(
      std::bind(&PointCloudToLaserScanZFilter::onSetParams, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
      "Subscribed: %s -> Publishing: %s", input_topic_.c_str(), output_topic_.c_str());
  }

private:
  rcl_interfaces::msg::SetParametersResult
  onSetParams(const std::vector<rclcpp::Parameter> & params)
  {
    for (const auto & p : params) {
      const auto & name = p.get_name();
      if      (name == "min_z") min_z_ = p.as_double();
      else if (name == "max_z") max_z_ = p.as_double();
      else if (name == "angle_min") angle_min_ = p.as_double();
      else if (name == "angle_max") angle_max_ = p.as_double();
      else if (name == "angle_increment") angle_increment_ = p.as_double();
      else if (name == "range_min") range_min_ = p.as_double();
      else if (name == "range_max") range_max_ = p.as_double();
      else if (name == "use_inf") use_inf_ = p.as_bool();
      else if (name == "inf_epsilon") inf_epsilon_ = p.as_double();
      else if (name == "target_frame") target_frame_ = p.as_string();
    }

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "ok";
    return result;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (angle_increment_ <= 0.0 || angle_max_ <= angle_min_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Invalid angle params: angle_min=%f angle_max=%f angle_increment=%f",
        angle_min_, angle_max_, angle_increment_);
      return;
    }

    const int beam_count = static_cast<int>(std::ceil((angle_max_ - angle_min_) / angle_increment_));
    if (beam_count <= 0) return;

    sensor_msgs::msg::LaserScan scan;
    scan.header = msg->header;
    if (!target_frame_.empty()) scan.header.frame_id = target_frame_;

    scan.angle_min = angle_min_;
    scan.angle_max = angle_min_ + (beam_count - 1) * angle_increment_;
    scan.angle_increment = angle_increment_;
    scan.time_increment = 0.0;
    scan.scan_time = 0.0;
    scan.range_min = range_min_;
    scan.range_max = range_max_;

    scan.ranges.resize(beam_count);

    const float inf_value = use_inf_
      ? std::numeric_limits<float>::infinity()
      : static_cast<float>(range_max_ + inf_epsilon_);

    std::fill(scan.ranges.begin(), scan.ranges.end(), inf_value);

    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");

    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
      const float x = *it_x;
      const float y = *it_y;
      const float z = *it_z;

      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
      if (z < min_z_ || z > max_z_) continue;

      const float r = std::hypot(x, y);
      if (!std::isfinite(r)) continue;
      if (r < range_min_ || r > range_max_) continue;

      const float ang = std::atan2(y, x);
      if (ang < angle_min_ || ang > angle_max_) continue;

      const int idx = static_cast<int>((ang - angle_min_) / angle_increment_);
      if (idx < 0 || idx >= beam_count) continue;

      // 같은 각도 bin에 여러 점이 들어오면 가장 가까운 점을 채택
      if (r < scan.ranges[idx]) {
        scan.ranges[idx] = r;
      }
    }

    pub_->publish(scan);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_cb_;

  std::string input_topic_;
  std::string output_topic_;
  std::string target_frame_;

  double min_z_{-0.2}, max_z_{0.2};
  double angle_min_{-M_PI}, angle_max_{M_PI}, angle_increment_{0.005};
  double range_min_{0.05}, range_max_{50.0};
  bool use_inf_{true};
  double inf_epsilon_{1.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudToLaserScanZFilter>());
  rclcpp::shutdown();
  return 0;
}
