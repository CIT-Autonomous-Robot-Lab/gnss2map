// SPDX-FileCopyrightText: 2024 MakotoYoshigoe
// SPDX-License-Identifier: Apache-2.0

#ifndef GNSS2MAP__GNSSPOSER_HPP_
#define GNSS2MAP__GNSSPOSER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <string>

namespace gnss2map
{
class GnssPoser : public rclcpp::Node
{
        public:
    GnssPoser();
    ~GnssPoser();
    void gnss_pose_callback(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);
    void ekf_pose_callback(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);
    void loop(void);
    void declare_params(void);
    void init_pub_sub(void);
    void set_gnss_info(void);
    void set_ekf_info(void);
    double get_pub_rate(void);

        private:
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_gnss_ekf_pose_with_covariance_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_ekf_pose_with_covariance_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_gnss_pose_with_covariance_;
    bool ekf_first_receive_;
    geometry_msgs::msg::PoseWithCovarianceStamped gnss_ekf_pose_with_covariance_;
    geometry_msgs::msg::PoseWithCovarianceStamped ekf_pose_with_covariance_;
    geometry_msgs::msg::PoseWithCovarianceStamped gnss_pose_with_covariance_;
    std::string frame_id_;
    double pub_rate_;
};
}

#endif // GNSS2MAP__GNSSPOSER_HPP_