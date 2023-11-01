// SPDX-FileCopyrightText: 2023 MakotoYoshigoe
// SPDX-License-Identifier: Apache-2.0

#include "gnss2map/GaussKruger.hpp"
#include <cmath>
#include <vector>

namespace gnss2map
{
    GaussKruger::GaussKruger() : Node("gauss_kruger")
    {
        setParam();
        getParam();
        initPubSub();
        initVariable();
    }

    GaussKruger::~GaussKruger(){}

    void GaussKruger::setParam()
    {
        this->declare_parameter("p0", std::vector<double>(2, 0.0));
        this->declare_parameter("gnss0", std::vector<double>(2, 0.0));
        this->declare_parameter("p1", std::vector<double>(2, 0.0));
        this->declare_parameter("gnss1", std::vector<double>(2, 0.0));
        this->declare_parameter("a", 6378137.0);
        this->declare_parameter("F", 298.257222);
        this->declare_parameter("m0", 0.9999);
        this->declare_parameter("theta_offset", 0.0);
    }

    void GaussKruger::getParam()
    {
        this->get_parameter("p0", p0_);
        this->get_parameter("gnss0", gnss0_);
        this->get_parameter("p1", p1_);
        this->get_parameter("gnss1", gnss1_);
        this->get_parameter("a", a_);
        this->get_parameter("F", F_);
        this->get_parameter("m0", m0_);
        this->get_parameter("theta_offset", theta_offset_);
    }

    void GaussKruger::initPubSub()
    {
        sub_gnss_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("gnss/fix", 2, std::bind(&GaussKruger::cbGnss, this, std::placeholders::_1));
        pub_odom_gnss_ = this->create_publisher<nav_msgs::msg::Odometry>("odom/gnss", 2);
        pub_gnss_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("gnss_pose", 2);
    }

    void GaussKruger::cbGnss(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
    {
        double rad_phi = msg->latitude*M_PI/180;
        double rad_lambda = msg->longitude*M_PI/180;
        double x, y;
        gaussKruger(rad_phi, rad_lambda, x, y);
        pubOdomGnss(x, y);
        pubGnssPose(x, y);
    }

    void GaussKruger::initVariable()
    {
        for(int i=0; i<2; ++i){
            gnss0_[i] *= M_PI/180;
            gnss1_[i] *= M_PI/180;
        }

        rad_theta_offset_ = theta_offset_*M_PI/180;

        double n = 1 / (2 * F_ - 1);
        double n2 = pow(n, 2), n3 = pow(n, 3), n4 = pow(n, 4), n5 = pow(n, 5);
        alpha_[0] = n/2 - 2.0*n2/3 + 5.0*n3/16 + 41.0*n4/180 - 127.0*n5/288;
        alpha_[1] = 13.0*n2/48 - 3.0*n3/5 + 557.0*n4/1440 + 281.0*n5/630;
        alpha_[2] = 61.0*n3/240 - 103.0*n4/140 + 15061.0*n5/26880;
        alpha_[3] = 49561.0*n4/161280 - 179.0*n5/168;
        alpha_[4] = 34729.0*n5/80640;

        A_[0] = 1.0 + n2/4 + n4/64;
        A_[1] = -3.0/2*(n - n3/8 - n5/64);
        A_[2] = 15.0/16*(n2 - n4/4);
        A_[3] = -35./48*(n3 - 5.*n5/16);
        A_[4] = 315.*n4/512;
        A_[5] = -693.*n5/1280;

        A_bar_ = m0_ * a_ * A_[0]/(1 + n);

        S_bar_phi0_ = A_[0]*gnss0_[0];
        for(int i=1; i<=5; ++i) S_bar_phi0_ += A_[i]*sin(2*i*gnss0_[0]);
        S_bar_phi0_ *= m0_*a_ /(1+n);

        kt_ = 2*sqrt(n) / (1+n);
        kx_ = 1, ky_ = 1; 

        double x, y;
        gaussKruger(gnss1_[0], gnss1_[1], x, y);
        kx_ = p1_[0] / x;
        ky_ = p1_[1] / y;
        // RCLCPP_INFO(get_logger(), "kx: %lf, ky: %lf", kx_, ky_);
    }

    void GaussKruger::gaussKruger(double rad_phi, double rad_lambda, double &x, double &y)
    {
        double t = sinh(atanh(sin(rad_phi)) - kt_*atanh(kt_*sin(rad_phi)));
        double t_bar = sqrt(1+pow(t, 2));
        double diff_lambda = rad_lambda - gnss0_[1];
        double lambda_cos = cos(diff_lambda), lambda_sin = sin(diff_lambda);
        double zeta = atan2(t, lambda_cos), eta = atanh(lambda_sin / t_bar);
        x = zeta;
        y = eta;
        for(int i=1; i<=5; ++i){
            x += alpha_[i-1] * sin(2*i*zeta) * cosh(2*i*eta);
            y += alpha_[i-1] * cos(2*i*zeta) * sinh(2*i*eta);
        }
        x = x * A_bar_ - S_bar_phi0_;
        y *= A_bar_;
        x = x*cos(rad_theta_offset_) - y*sin(rad_theta_offset_);
        x *= kx_;
        y = x*sin(rad_theta_offset_) + y*cos(rad_theta_offset_);
        y *= ky_;
        x += p0_[0];
        y = p0_[1]-y;
    }

    void GaussKruger::pubOdomGnss(double x, double y)
    {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = now();
        odom.header.frame_id = "map";
        odom.child_frame_id = "base_footprint";
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        pub_odom_gnss_->publish(odom);
    }

    void GaussKruger::pubGnssPose(double x, double y)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = now();
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pub_gnss_pose_->publish(pose);
    }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<gnss2map::GaussKruger>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}