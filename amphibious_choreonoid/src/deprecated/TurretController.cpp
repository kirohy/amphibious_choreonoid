#include <cnoid/MathUtil>
#include <cnoid/SimpleController>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

class TurretController : public cnoid::SimpleController {
  private:
    std::unique_ptr<ros::NodeHandle> nh_;
    tf2_ros::Buffer tf_buf_;
    tf2_ros::TransformListener tf_listener_;
    cnoid::BodyPtr body_;
    cnoid::LinkPtr joints_[2];
    double q_ref_[2];
    double q_prev_[2];
    double dt_;

  public:
    TurretController() : tf_listener_(tf_buf_){};

    virtual bool configure(cnoid::SimpleControllerConfig *config) override {
        nh_.reset(new ros::NodeHandle(config->body()->name()));
        return true;
    }

    virtual bool initialize(cnoid::SimpleControllerIO *io) override {
        body_ = io->body();
        joints_[0] = body_->link("TURRET_Y");
        joints_[1] = body_->link("TURRET_P");

        for (int i = 0; i < 2; i++) {
            joints_[i]->setActuationMode(cnoid::Link::JOINT_TORQUE);
            io->enableIO(joints_[i]);
            q_ref_[i] = q_prev_[i] = joints_[i]->q();
        }

        io->enableInput(io->body()->rootLink(), LinkPosition);
        dt_ = io->timeStep();

        return true;
    }

    virtual bool control() override {
        static const double P = 200.0;
        static const double D = 50.0;

        geometry_msgs::TransformStamped tf_stamped;
        try {
            tf_stamped = tf_buf_.lookupTransform("world", "base_link", ros::Time(0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            return true;
        }

        auto pos_z = tf_stamped.transform.translation.z;

        if (pos_z > 0) {
            q_ref_[1] = cnoid::radian(-15.0);
        } else {
            q_ref_[1] = 0.0;
        }

        for (int i = 0; i < 2; i++) {
            double q = joints_[i]->q();
            double dq = (q - q_prev_[i]) / dt_;
            double dq_ref = 0.0;
            joints_[i]->u() = P * (q_ref_[i] - q) + D * (dq_ref - dq);
            q_prev_[i] = q;
        }

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(TurretController)
