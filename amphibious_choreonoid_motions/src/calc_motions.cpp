#include <amphibious_choreonoid_motions/machine_state.hpp>
#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64MultiArray.h>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class CalcMotions {
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher vel_pub_;
    ros::Publisher joint_ref_pub_;
    ros::Publisher state_pub_;
    ros::Timer timer_;
    ros::Time clock_;

    geometry_msgs::Twist vel_msg_;
    std::map<std::string, int> joint_list_;
    std_msgs::Float64MultiArray joint_ref_msg_;
    std::vector<double> joint_ref_;

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

  public:
    CalcMotions() : pnh_("~"), it_(nh_), tf_listener_(tf_buf_) {
        vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        joint_ref_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("joint_ref", 1);
        state_pub_ = nh_.advertise<std_msgs::Int32>("machine_state", 1);
        image_sub_ = it_.subscribe("image_raw", 1, &CalcMotions::image_callback, this);
        image_pub_ = it_.advertise("image_result", 1);

        pnh_.param("freq", freq_, 100);
        pnh_.param("fps", fps_, 30);
        pnh_.param("offset_vel_linear", offset_vel_linear_, 1.0);
        pnh_.param("offset_vel_angular", offset_vel_angular_, 1.0);
        pnh_.param("P_image", P_image_, 0.001);
        pnh_.param("D_image", D_image_, 0.0);
        pnh_.param("P", P_, 200.0);
        pnh_.param("D", D_, 50.0);
        pnh_.param("robot", robot_name, std::string("robot"));
        dt_ = 1.0 / freq_;

        XmlRpc::XmlRpcValue robot_param;
        pnh_.getParam(robot_name, robot_param);

        if (robot_param["joint_list"].valid()) {
            auto joint_list = robot_param["joint_list"];
            joint_ref_.resize(joint_list.size());
            joint_ref_msg_.data.resize(joint_list.size());
            for (int i = 0; i < joint_list.size(); i++) {
                joint_list_[static_cast<std::string>(joint_list[i]["name"])] = i;
            }
        }

        joint_ref_msg_.data[joint_list_["TURRET_Y"]] = 0.0;
        joint_ref_msg_.data[joint_list_["TURRET_P"]] = 0.0;
        joint_ref_msg_.data[joint_list_["ARM_L"]] = 0.0;
        joint_ref_msg_.data[joint_list_["ARM_R"]] = 0.0;

        vel_msg_.linear.x = 0.0;
        vel_msg_.linear.y = 0.0;
        vel_msg_.linear.z = 0.0;
        vel_msg_.angular.x = 0.0;
        vel_msg_.angular.y = 0.0;
        vel_msg_.angular.z = 0.0;
        vel_pub_.publish(vel_msg_);

        timer_ = nh_.createTimer(ros::Duration(dt_), &CalcMotions::timer_callback, this);
    }

    ~CalcMotions() {}

    void image_callback(const sensor_msgs::ImageConstPtr &msg) {
        try {
            cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat hsv_image, color_mask, gray_image, bin_image, cv_image2;
        cv::cvtColor(cv_ptr_->image, hsv_image, CV_BGR2HSV);

        cv::inRange(hsv_image, cv::Scalar(0, 0, 100, 0), cv::Scalar(180, 45, 255, 0), color_mask); // detect white

        cv::bitwise_and(cv_ptr_->image, cv_ptr_->image, cv_image2, color_mask);
        cv::cvtColor(cv_image2, gray_image, CV_BGR2GRAY);
        cv::threshold(gray_image, bin_image, 80, 255, CV_THRESH_BINARY);

        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(bin_image, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

        double max_area = 0;
        int max_area_contour = -1;
        for (int j = 0; j < contours.size(); j++) {
            double area = cv::contourArea(contours.at(j));
            if (max_area < area) {
                max_area = area;
                max_area_contour = j;
            }
        }

        if (max_area_contour >= 0) {
            float radius;
            cv::minEnclosingCircle(contours.at(max_area_contour), target_, radius);
            cv::circle(cv_ptr_->image, target_, radius, cv::Scalar(0, 0, 255), 3, 4);

            if (state_.current() == MachineState::State::SEARCH_OBJ) {
                vel_msg_.linear.x = offset_vel_linear_;
                vel_msg_.linear.y = 0.0;
                vel_msg_.linear.z = 0.0;
                vel_msg_.angular.x = 0.0;
                vel_msg_.angular.y = 0.0;
                vel_msg_.angular.z = P_image_ * (cv_ptr_->image.cols / 2 - target_.x) - D_image_ * (target_.x - target_prev_.x) * fps_;
                if (max_area > cv_ptr_->image.cols * cv_ptr_->image.rows * 0.6) {
                    state_.set_next(MachineState::State::HOLDING_OBJ);
                }
            }

            target_prev_ = target_;
            // state_.transition();
        } else if (state_.current() == MachineState::State::SEARCH_OBJ) {
            vel_msg_.linear.x = 0.0;
            vel_msg_.linear.y = 0.0;
            vel_msg_.linear.z = 0.0;
            vel_msg_.angular.x = 0.0;
            vel_msg_.angular.y = 0.0;
            vel_msg_.angular.z = offset_vel_angular_;
        }

        image_pub_.publish(cv_ptr_->toImageMsg());
    }

    void timer_callback(const ros::TimerEvent &) {
        using namespace MachineState;

        static geometry_msgs::TransformStamped tf_robot;
        try {
            tf_robot = tf_buf_.lookupTransform("map", "base_link", ros::Time(0));
            if (tf_robot.transform.translation.z >= 0.0) {
                joint_ref_msg_.data[joint_list_["TURRET_P"]] = -15.0;
            } else {
                joint_ref_msg_.data[joint_list_["TURRET_P"]] = 0.0;
            }
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }

        if (state_.current() == State::HOLDING_OBJ && state_.prev() == State::SEARCH_OBJ) {
            clock_ = ros::Time::now();
            stop();
        } else if (state_.current() == State::HOLDING_OBJ && state_.prev() == State::HOLDING_OBJ) {
            if (ros::Time::now() - clock_ > ros::Duration(1.0)) {
                state_.set_next(State::CARRY_OBJ);
            }
            stop();
            joint_ref_msg_.data[joint_list_["ARM_L"]] = 180.0;
            joint_ref_msg_.data[joint_list_["ARM_R"]] = 180.0;
        } else if (state_.current() == State::CARRY_OBJ) {
            calc_vel(std::string("goal_front"), State::INTO_GOAL, true);
        } else if (state_.current() == State::INTO_GOAL) {
            calc_vel(std::string("goal"), State::RELEASE_OBJ, true);
        } else if (state_.current() == State::RELEASE_OBJ && state_.prev() == State::INTO_GOAL) {
            clock_ = ros::Time::now();
            stop();
        } else if (state_.current() == State::RELEASE_OBJ && state_.prev() == State::RELEASE_OBJ) {
            if (ros::Time::now() - clock_ > ros::Duration(1.0)) {
                state_.set_next(State::EXIT_GOAL);
            }
            stop();
            joint_ref_msg_.data[joint_list_["ARM_L"]] = 0.0;
            joint_ref_msg_.data[joint_list_["ARM_R"]] = 0.0;
        } else if (state_.current() == State::EXIT_GOAL) {
            calc_vel(std::string("goal_front"), State::PREPARE, false);
        } else if (state_.current() == State::PREPARE) {
            calc_vel(std::string("initial_position"), State::SEARCH_OBJ, true);
        }
        state_.transition();
        vel_pub_.publish(vel_msg_);
        joint_ref_pub_.publish(joint_ref_msg_);
        state_pub_.publish(state_.current_as_msg());
    }

    void calc_vel(const std::string tf_name, const MachineState::State next, bool forward) {
        static geometry_msgs::TransformStamped tf_target;
        try {
            tf_target = tf_buf_.lookupTransform("base_link", tf_name, ros::Time(0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }

        double dist = sqrt(pow(tf_target.transform.translation.x, 2.0) + pow(tf_target.transform.translation.y, 2.0));
        double vel_x = P_ * dist;
        if (vel_x > offset_vel_linear_) {
            vel_x = offset_vel_linear_;
        }
        double angle = atan2(tf_target.transform.translation.y, tf_target.transform.translation.x);

        if (forward) {
            angle = atan2(tf_target.transform.translation.y, tf_target.transform.translation.x);
        } else {
            angle = -atan2(tf_target.transform.translation.y, -tf_target.transform.translation.x);
            vel_x = -vel_x;
        }
        if (abs(angle) > 0.1) {
            vel_x = 0.0;
        }
        vel_msg_.linear.x = vel_x;
        vel_msg_.linear.y = 0.0;
        vel_msg_.linear.z = 0.0;
        vel_msg_.angular.x = 0.0;
        vel_msg_.angular.y = 0.0;
        vel_msg_.angular.z = P_ * angle;

        if (dist < 0.1) {
            state_.set_next(next);
        }
    }

    void stop() {
        vel_msg_.linear.x = 0.0;
        vel_msg_.linear.y = 0.0;
        vel_msg_.linear.z = 0.0;
        vel_msg_.angular.x = 0.0;
        vel_msg_.angular.y = 0.0;
        vel_msg_.angular.z = 0.0;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "calc_motions");
    CalcMotions cm;
    ros::spin();
    return 0;
}
