/**
 * @file      fastlio_frame_corrector.cpp
 * @brief     Convert FAST-LIO output into the robot/planner coordinate frame.
 *
 * FAST-LIO's extrinsic_R is the LiDAR-to-IMU extrinsic inside MID360.  Do not
 * use it to compensate how the whole sensor is mounted on the robot.  This
 * node applies that mounting/display/planning-frame correction after FAST-LIO
 * has already estimated odometry and registered cloud.
 */

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

namespace
{

tf2::Vector3 apply_matrix(const tf2::Matrix3x3& matrix, const tf2::Vector3& vector)
{
  return matrix * vector;
}

tf2::Quaternion transform_orientation(
  const tf2::Matrix3x3& correction,
  const tf2::Matrix3x3& correction_inverse,
  const geometry_msgs::msg::Quaternion& input)
{
  tf2::Quaternion q(input.x, input.y, input.z, input.w);
  q.normalize();

  tf2::Matrix3x3 input_rotation(q);
  tf2::Matrix3x3 output_rotation = correction * input_rotation * correction_inverse;

  tf2::Quaternion output;
  output_rotation.getRotation(output);
  output.normalize();
  return output;
}

geometry_msgs::msg::Quaternion to_msg(const tf2::Quaternion& q)
{
  geometry_msgs::msg::Quaternion msg;
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  msg.w = q.w();
  return msg;
}

}  // namespace

class FastlioFrameCorrector : public rclcpp::Node
{
public:
  FastlioFrameCorrector()
  : Node("fastlio_frame_corrector")
  {
    input_cloud_topic_ = declare_parameter<std::string>("input_cloud_topic", "/cloud_registered");
    output_cloud_topic_ = declare_parameter<std::string>("output_cloud_topic", "/scan/cloud_registered");
    input_odom_topic_ = declare_parameter<std::string>("input_odom_topic", "/Odometry");
    output_odom_topic_ = declare_parameter<std::string>("output_odom_topic", "/scan/Odometry");
    input_path_topic_ = declare_parameter<std::string>("input_path_topic", "/path");
    output_path_topic_ = declare_parameter<std::string>("output_path_topic", "/scan/path");
    output_frame_ = declare_parameter<std::string>("output_frame", "scan_map");
    output_child_frame_ = declare_parameter<std::string>("output_child_frame", "base_link");

    const std::vector<double> default_matrix{
      0.0, 1.0, 0.0,
      1.0, 0.0, 0.0,
      0.0, 0.0, -1.0};
    const auto matrix = declare_parameter<std::vector<double>>("correction_matrix", default_matrix);
    if (matrix.size() != 9) {
      throw std::runtime_error("correction_matrix must contain exactly 9 values");
    }

    correction_ = tf2::Matrix3x3(
      matrix[0], matrix[1], matrix[2],
      matrix[3], matrix[4], matrix[5],
      matrix[6], matrix[7], matrix[8]);
    correction_inverse_ = correction_.transpose();

    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_cloud_topic_, rclcpp::SensorDataQoS());
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      output_odom_topic_, rclcpp::SensorDataQoS());
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      output_path_topic_, rclcpp::QoS(10));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&FastlioFrameCorrector::cloud_callback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      input_odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&FastlioFrameCorrector::odom_callback, this, std::placeholders::_1));
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      input_path_topic_, rclcpp::QoS(10),
      std::bind(&FastlioFrameCorrector::path_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "FAST-LIO frame corrector: cloud %s -> %s, odom %s -> %s, frame=%s, child=%s",
      input_cloud_topic_.c_str(), output_cloud_topic_.c_str(),
      input_odom_topic_.c_str(), output_odom_topic_.c_str(),
      output_frame_.c_str(), output_child_frame_.c_str());
    RCLCPP_INFO(get_logger(), "Correction: x'=y, y'=x, z'=-z");
  }

private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    auto output = *msg;
    output.header.frame_id = output_frame_;

    try {
      sensor_msgs::PointCloud2Iterator<float> iter_x(output, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(output, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(output, "z");

      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        const tf2::Vector3 corrected = apply_matrix(
          correction_, tf2::Vector3(*iter_x, *iter_y, *iter_z));
        *iter_x = static_cast<float>(corrected.x());
        *iter_y = static_cast<float>(corrected.y());
        *iter_z = static_cast<float>(corrected.z());
      }
    } catch (const std::runtime_error& error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skip cloud correction because PointCloud2 xyz fields are unavailable: %s",
        error.what());
      return;
    }

    cloud_pub_->publish(output);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    auto output = *msg;
    output.header.frame_id = output_frame_;
    output.child_frame_id = output_child_frame_;

    const auto& p = msg->pose.pose.position;
    const tf2::Vector3 corrected_position = apply_matrix(correction_, tf2::Vector3(p.x, p.y, p.z));
    output.pose.pose.position.x = corrected_position.x();
    output.pose.pose.position.y = corrected_position.y();
    output.pose.pose.position.z = corrected_position.z();

    output.pose.pose.orientation = to_msg(transform_orientation(
      correction_, correction_inverse_, msg->pose.pose.orientation));

    const auto& linear = msg->twist.twist.linear;
    const tf2::Vector3 corrected_linear = apply_matrix(
      correction_, tf2::Vector3(linear.x, linear.y, linear.z));
    output.twist.twist.linear.x = corrected_linear.x();
    output.twist.twist.linear.y = corrected_linear.y();
    output.twist.twist.linear.z = corrected_linear.z();

    const auto& angular = msg->twist.twist.angular;
    const tf2::Vector3 corrected_angular = apply_matrix(
      correction_, tf2::Vector3(angular.x, angular.y, angular.z));
    output.twist.twist.angular.x = corrected_angular.x();
    output.twist.twist.angular.y = corrected_angular.y();
    output.twist.twist.angular.z = corrected_angular.z();

    odom_pub_->publish(output);

    geometry_msgs::msg::TransformStamped transform;
    transform.header = output.header;
    transform.child_frame_id = output.child_frame_id;
    transform.transform.translation.x = output.pose.pose.position.x;
    transform.transform.translation.y = output.pose.pose.position.y;
    transform.transform.translation.z = output.pose.pose.position.z;
    transform.transform.rotation = output.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
  }

  void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    auto output = *msg;
    output.header.frame_id = output_frame_;

    for (auto& pose : output.poses) {
      pose.header.frame_id = output_frame_;

      const auto& p = pose.pose.position;
      const tf2::Vector3 corrected_position = apply_matrix(correction_, tf2::Vector3(p.x, p.y, p.z));
      pose.pose.position.x = corrected_position.x();
      pose.pose.position.y = corrected_position.y();
      pose.pose.position.z = corrected_position.z();

      pose.pose.orientation = to_msg(transform_orientation(
        correction_, correction_inverse_, pose.pose.orientation));
    }

    path_pub_->publish(output);
  }

  std::string input_cloud_topic_;
  std::string output_cloud_topic_;
  std::string input_odom_topic_;
  std::string output_odom_topic_;
  std::string input_path_topic_;
  std::string output_path_topic_;
  std::string output_frame_;
  std::string output_child_frame_;

  tf2::Matrix3x3 correction_;
  tf2::Matrix3x3 correction_inverse_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FastlioFrameCorrector>());
  rclcpp::shutdown();
  return 0;
}
