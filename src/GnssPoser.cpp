// SPDX-FileCopyrightText: 2024 MakotoYoshigoe
// SPDX-License-Identifier: Apache-2.0

#include "gnss2map/GnssPoser.hpp"

namespace gnss2map
{
    GnssPoser::GnssPoser() : Node("gnss_poser")
    {
        declare_params();
        init_pub_sub();
    }

    GnssPoser::~GnssPoser(){}

    void GnssPoser::declare_params(void)
    {
        declare_parameter("frame_id", "map");
        get_parameter("frame_id", frame_id_);
        declare_parameter("pub_rate", 10.0);
        get_parameter("pub_rate", pub_rate_);
        ekf_first_receive_ = false;
    }

    void GnssPoser::init_pub_sub(void)
    {
        pub_gnss_ekf_pose_with_covariance_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("gnss_ekf_pose_with_covariance", 2);
        sub_gnss_pose_with_covariance_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "gnss_pose_with_covariance", 2, std::bind(&GnssPoser::gnss_pose_callback, this, std::placeholders::_1)
        );
        sub_ekf_pose_with_covariance_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "ekf_pose_with_covariance", 2, std::bind(&GnssPoser::ekf_pose_callback, this, std::placeholders::_1)
        );

    }

    void GnssPoser::gnss_pose_callback(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
    {
        gnss_pose_with_covariance_ = *msg;
    }

    void GnssPoser::ekf_pose_callback(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
    {
        if(!ekf_first_receive_) ekf_first_receive_ = true;
        ekf_pose_with_covariance_ = *msg;
    }

    void GnssPoser::set_gnss_info()
    {
        gnss_ekf_pose_with_covariance_.pose.pose.position = gnss_pose_with_covariance_.pose.pose.position;
        gnss_ekf_pose_with_covariance_.pose.covariance[0] = gnss_pose_with_covariance_.pose.covariance[0];
        gnss_ekf_pose_with_covariance_.pose.covariance[7] = gnss_pose_with_covariance_.pose.covariance[7];
        gnss_ekf_pose_with_covariance_.pose.covariance[14] = gnss_pose_with_covariance_.pose.covariance[14];
    }

    void GnssPoser::set_ekf_info()
    {
        gnss_ekf_pose_with_covariance_.pose.pose.orientation = ekf_pose_with_covariance_.pose.pose.orientation;
        gnss_ekf_pose_with_covariance_.pose.covariance[21] = ekf_pose_with_covariance_.pose.covariance[21];
        gnss_ekf_pose_with_covariance_.pose.covariance[28] = ekf_pose_with_covariance_.pose.covariance[28];
        gnss_ekf_pose_with_covariance_.pose.covariance[35] = ekf_pose_with_covariance_.pose.covariance[35];
    }

    double GnssPoser::get_pub_rate(){
        return pub_rate_;
    }

    void GnssPoser::loop(void)
    {
        if(!ekf_first_receive_) return;
        gnss_ekf_pose_with_covariance_.header.frame_id = frame_id_;
        gnss_ekf_pose_with_covariance_.header.stamp = now();
        set_gnss_info();
        set_ekf_info();
        pub_gnss_ekf_pose_with_covariance_->publish(gnss_ekf_pose_with_covariance_);
    }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<gnss2map::GnssPoser>();
    rclcpp::Rate rate(node->get_pub_rate());
    while(rclcpp::ok()){
        node->loop();
        rclcpp::spin_some(node);
        rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}