// SPDX-FileCopyrightText: 2023 MakotoYoshigoe
// SPDX-License-Identifier: Apache-2.0

#ifndef GNSS2MAP__GAUSSKRUGER_HPP_
#define GNSS2MAP__GAUSSKRUGER_HPP_

#define NO_FIX -1

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace gnss2map
{
class GaussKruger : public rclcpp::Node
{
        public:
    GaussKruger();
    ~GaussKruger();
        private:
    std::vector<double> p0_, gnss0_, p1_, gnss1_;
    double a_, F_;
    double m0_;
    double theta_offset_, rad_theta_offset_;
    double alpha_[5], A_[6];
    double A_bar_, S_bar_phi0_;
    double kt_;
    double kx_, ky_;
    double ignore_th_cov_;

    // Eigen::Matrix2d R_;
    Eigen::Matrix2d K_;
    Eigen::Rotation2Df R_;

    
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gnss_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_gnss_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_gnss_pose_;

    void initPubSub();
    void setParam();
    void getParam();
    void cbGnss(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);
    void initVariable();
    void gaussKruger(double rad_phi, double rad_lambda, double &x, double &y);
    void printVariable();
    void pubOdomGnss(double x, double y);
    void pubGnssPose(double x, double y);
};
}

#endif // GNSS2MAP__GAUSSKRUGER_HPP_