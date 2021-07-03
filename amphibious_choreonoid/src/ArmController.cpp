#include <amphibious_choreonoid_motions/machine_state.hpp>
#include <cnoid/MathUtil>
#include <cnoid/SimpleController>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

class ArmController : public cnoid::SimpleController {
  private:
    std::unique_ptr<ros::NodeHandle> nh_;
    ros::Subscriber state_sub_;
    cnoid::BodyPtr body_;
    cnoid::LinkPtr joints_[2];
    double q_ref_[2];
    double q_prev_[2];
    double dt_;

  public:
    virtual bool configure(cnoid::SimpleControllerConfig *config) override {
        nh_.reset(new ros::NodeHandle(config->body()->name()));
        state_sub_ = nh_->subscribe("machine_state", 1, &ArmController::stateCb, this);
        return true;
    }

    virtual bool initialize(cnoid::SimpleControllerIO *io) override {
        body_ = io->body();
        joints_[0] = body_->link("ARM_L");
        joints_[1] = body_->link("ARM_R");

        for (int i = 0; i < 2; i++) {
            joints_[i]->setActuationMode(cnoid::Link::JOINT_TORQUE);
            io->enableIO(joints_[i]);
            q_ref_[i] = q_prev_[i] = joints_[i]->q();
        }

        dt_ = io->timeStep();

        return true;
    }

    virtual bool control() override {
        static const double P = 200.0;
        static const double D = 50.0;

        for (int i = 0; i < 2; i++) {
            double q = joints_[i]->q();
            double dq = (q - q_prev_[i]) / dt_;
            double dq_ref = 0.0;
            joints_[i]->u() = P * (q_ref_[i] - q) + D * (dq_ref - dq);
            q_prev_[i] = q;
        }

        return true;
    }

    void stateCb(const std_msgs::Int32Ptr &msg) {
        using namespace MachineState;
        auto recv_state = from_msg(msg);
        if (recv_state == State::HOLDING_OBJ || recv_state == State::CARRY_OBJ || recv_state == State::INTO_GOAL) {
            q_ref_[0] = cnoid::radian(180.0);
            q_ref_[1] = cnoid::radian(180.0);
        } else if (recv_state == State::RELEASE_OBJ || recv_state == State::EXIT_GOAL || recv_state == State::SEARCH_OBJ) {
            q_ref_[0] = cnoid::radian(0.0);
            q_ref_[1] = cnoid::radian(0.0);
        }
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(ArmController)
