#include <amphibious_choreonoid_motions/machine_state.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

class CalcMotions {
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher vel_pub_;
    ros::Publisher state_pub_;
    ros::Timer timer_;
    geometry_msgs::Twist vel_msg_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    cv_bridge::CvImagePtr cv_ptr_;
    cv::Point2f target_;
    cv::Point2f target_prev_;
    MachineState::StateTransition state_;
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
    CalcMotions() : pnh_("~"), it_(nh_) {
        vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        state_pub_ = nh_.advertise<std_msgs::Int32>("machine_state", 1);
        image_sub_ = it_.subscribe("image_raw", 1, &CalcMotions::imageCb, this);
        image_pub_ = it_.advertise("image_result", 1);

        pnh_.param("freq", freq_, 100);
        pnh_.param("fps", fps_, 30);
        pnh_.param("offset_vel_linear", offset_vel_linear_, 1.0);
        pnh_.param("offset_vel_angular", offset_vel_angular_, 1.0);
        pnh_.param("P_image", P_image_, 0.001);
        pnh_.param("D_image", D_image_, 0.0);
        pnh_.param("P", P_, 200.0);
        pnh_.param("D", D_, 50.0);
        dt_ = 1.0 / freq_;

        vel_msg_.linear.x = 0.0;
        vel_msg_.linear.y = 0.0;
        vel_msg_.linear.z = 0.0;
        vel_msg_.angular.x = 0.0;
        vel_msg_.angular.y = 0.0;
        vel_msg_.angular.z = 0.0;
        vel_pub_.publish(vel_msg_);

        timer_ = nh_.createTimer(ros::Duration(dt_), &CalcMotions::timerCb, this);
    }

    ~CalcMotions() {
        cv::destroyAllWindows();
    }

    void imageCb(const sensor_msgs::ImageConstPtr &msg) {
        try {
            cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat hsv_image, color_mask, gray_image, bin_image, cv_image2;
        cv::cvtColor(cv_ptr_->image, hsv_image, CV_BGR2HSV);

        cv::inRange(hsv_image, cv::Scalar(0, 0, 100, 0), cv::Scalar(180, 45, 255, 0), color_mask);

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
            } else if (state_.current() == MachineState::State::HOLDING_OBJ) {
                vel_msg_.linear.x = 0.0;
                vel_msg_.linear.y = 0.0;
                vel_msg_.linear.z = 0.0;
                vel_msg_.angular.x = 0.0;
                vel_msg_.angular.y = 0.0;
                vel_msg_.angular.z = 0.0;
            }

            target_prev_ = target_;
            state_.transition();
        } else {
            vel_msg_.linear.x = 0.0;
            vel_msg_.linear.y = 0.0;
            vel_msg_.linear.z = 0.0;
            vel_msg_.angular.x = 0.0;
            vel_msg_.angular.y = 0.0;
            vel_msg_.angular.z = offset_vel_angular_;
        }

        image_pub_.publish(cv_ptr_->toImageMsg());
    }

    void timerCb(const ros::TimerEvent &) {
        vel_pub_.publish(vel_msg_);
        state_pub_.publish(state_.current_as_msg());
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "calc_motions");
    CalcMotions cm;
    ros::spin();
    return 0;
}
