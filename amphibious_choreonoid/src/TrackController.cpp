#include <cnoid/SimpleController>
#include <geometry_msgs/Twist.h>
#include <mutex>
#include <ros/ros.h>

class TrackController : public cnoid::SimpleController {
  private:
    std::unique_ptr<ros::NodeHandle> nh_;
    ros::Subscriber vel_sub_;
    cnoid::Link *trackL;
    cnoid::Link *trackR;
    std::mutex cmd_mutex_;
    float track_vel_[2];

  public:
    virtual bool configure(cnoid::SimpleControllerConfig *config) override {
        nh_.reset(new ros::NodeHandle(config->body()->name()));
        vel_sub_ = nh_->subscribe("cmd_vel", 1, &TrackController::motionsCb, this);
        return true;
    }

    virtual bool initialize(cnoid::SimpleControllerIO *io) override {
        trackL = io->body()->link("TRACK_L");
        trackR = io->body()->link("TRACK_R");

        trackL->setActuationMode(cnoid::Link::JointVelocity);
        trackR->setActuationMode(cnoid::Link::JointVelocity);

        io->enableOutput(trackL);
        io->enableOutput(trackR);

        return true;
    }

    virtual bool control() override {
        {
            std::lock_guard<std::mutex> lock(cmd_mutex_);
            trackL->dq_target() = track_vel_[0];
            trackR->dq_target() = track_vel_[1];
        }
        return true;
    }

    void motionsCb(const geometry_msgs::TwistConstPtr msg) {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        if (msg->angular.z > 0) {
            track_vel_[0] = -1.0;
            track_vel_[1] = 1.0;
        } else if (msg->angular.z < 0) {
            track_vel_[0] = 1.0;
            track_vel_[1] = -1.0;
        } else if (msg->linear.x > 0) {
            track_vel_[0] = 1.0;
            track_vel_[1] = 1.0;
        } else {
            track_vel_[0] = 0.0;
            track_vel_[1] = 0.0;
        }
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(TrackController)
