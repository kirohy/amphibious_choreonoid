#include <cnoid/SimpleController>
#include <cnoid/ext/hairo-world-plugin/src/FluidDynamicsPlugin/Thruster.h>
#include <geometry_msgs/Twist.h>
#include <mutex>
#include <ros/ros.h>

class ThrustController : public cnoid::SimpleController {
  private:
    std::unique_ptr<ros::NodeHandle> nh_;
    ros::Subscriber vel_sub_;
    cnoid::Thruster *thrusterL;
    cnoid::Thruster *thrusterR;
    std::mutex cmd_mutex_;
    float thuruster_force_[2];

  public:
    virtual bool configure(cnoid::SimpleControllerConfig *config) override {
        nh_.reset(new ros::NodeHandle(config->body()->name()));
        vel_sub_ = nh_->subscribe("cmd_vel", 1, &ThrustController::motionsCb, this);
        return true;
    }

    virtual bool initialize(cnoid::SimpleControllerIO *io) override {
        thrusterL = io->body()->findDevice<cnoid::Thruster>("Thruster_L");
        thrusterR = io->body()->findDevice<cnoid::Thruster>("Thruster_R");

        if (!thrusterL || !thrusterR) {
            std::cout << "The thrusters are not found." << std::endl;
            return false;
        }

        return true;
    }

    virtual bool control() override {
        {
            std::lock_guard<std::mutex> lock(cmd_mutex_);
            thrusterL->force() = thuruster_force_[0];
            thrusterR->force() = thuruster_force_[1];
        }
        thrusterL->notifyStateChange();
        thrusterR->notifyStateChange();
        return true;
    }

    void motionsCb(const geometry_msgs::TwistConstPtr msg) {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        double x = msg->linear.x;
        double z = msg->angular.z;
        thuruster_force_[0] = (x - z) * 10;
        thuruster_force_[1] = (x + z) * 10;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(ThrustController)
