#ifndef CALC_MOTIONS_HPP
#define CALC_MOTIONS_HPP

#include "amphibious_choreonoid_motions/machine_state.hpp"
#include <amphibious_choreonoid/simple_joint_command.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <map>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class CalcMotions {
  public:
    CalcMotions();

    ~CalcMotions(){};

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher vel_pub_;
    ros::Publisher joint_ref_pub_;
    ros::Publisher state_pub_;
    ros::Timer timer_;
    ros::Time clock_;

    geometry_msgs::Twist vel_msg_;
    amphibious_choreonoid::simple_joint_command joint_ref_msg_;
    std::map<std::string, int> joint_list_; // Joint name and index

    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    cv_bridge::CvImagePtr cv_ptr_;
    cv::Point2f target_;
    cv::Point2f target_prev_;

    tf2_ros::Buffer tf_buf_;
    tf2_ros::TransformListener tf_listener_;

    MachineState::StateTransition state_;
    std::string robot_name;
    int freq_;
    int fps_;
    double dt_;
    double offset_vel_linear_;
    double offset_vel_angular_;
    double P_image_;
    double D_image_;
    double P_;
    double D_;

    void image_callback(const sensor_msgs::ImageConstPtr &);

    void timer_callback(const ros::TimerEvent &);

    void calc_vel(const std::string, const MachineState::State, bool);

    void stop();
};

#endif // CALC_MOTIONS_HPP
