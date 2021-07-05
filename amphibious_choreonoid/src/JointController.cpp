#include <amphibious_choreonoid/simple_joint_command.h>
#include <cnoid/MathUtil>
#include <cnoid/SimpleController>
#include <map>
#include <ros/ros.h>
#include <vector>

class JointController : public cnoid::SimpleController {
  private:
    std::unique_ptr<ros::NodeHandle> nh_;
    std::unique_ptr<ros::NodeHandle> pnh_;
    ros::Subscriber joint_ref_sub_;
    cnoid::BodyPtr body_;
    double dt_;

    struct joint_params {
        cnoid::LinkPtr joint;
        double P;
        double D;
        double q_ref;
        double q_prev;
    };
    std::vector<joint_params> joints_;
    std::map<std::string, int> joint_list_; // Joint name and index

  public:
    virtual bool configure(cnoid::SimpleControllerConfig *config) override {
        nh_.reset(new ros::NodeHandle(config->body()->name()));
        pnh_.reset(new ros::NodeHandle("~"));

        return true;
    }

    virtual bool initialize(cnoid::SimpleControllerIO *io) override {
        body_ = io->body();
        std::string robot_name = body_->name();

        XmlRpc::XmlRpcValue robot_param;
        pnh_->getParam(robot_name, robot_param);

        if (!robot_param["joint_list"].valid()) {
            ROS_WARN("[%s] NO Joint Configuration.", robot_name.c_str());
        } else {
            auto joint_list_param = robot_param["joint_list"];
            joints_.resize(joint_list_param.size());
            for (int i = 0; i < joint_list_param.size(); i++) {
                joints_[i].joint = body_->link(static_cast<std::string>(joint_list_param[i]["name"]));
                joints_[i].joint->setActuationMode(cnoid::Link::JOINT_TORQUE);
                io->enableIO(joints_[i].joint);
                joints_[i].P = static_cast<double>(joint_list_param[i]["P"]);
                joints_[i].D = static_cast<double>(joint_list_param[i]["D"]);
                joints_[i].q_ref = joints_[i].joint->q();
                joints_[i].q_prev = joints_[i].joint->q();
                joint_list_[joints_[i].joint->name()] = i;
                ROS_INFO("[%s] JointName: %s , P: %lf ,  D: %lf", robot_name.c_str(), joints_[i].joint->name().c_str(), joints_[i].P, joints_[i].D);
            }
        }

        dt_ = io->timeStep();

        joint_ref_sub_ = nh_->subscribe("joint_ref", 1, &JointController::update_joint_ref, this);

        return true;
    }

    virtual bool control() override {
        for (auto itr = joints_.begin(); itr != joints_.end(); ++itr) {
            double q = itr->joint->q();
            double dq = (q - itr->q_prev) / dt_;
            double dq_ref = 0.0;
            itr->joint->u() = itr->P * (itr->q_ref - q) + itr->D * (dq_ref - dq);
            itr->q_prev = q;
        }

        return true;
    }

    void update_joint_ref(const amphibious_choreonoid::simple_joint_commandConstPtr &msg) {
        auto num = msg->name.size();
        for (int i = 0; i < num; i++) {
            auto name = msg->name[i];
            auto itr = joint_list_.find(name);
            if (itr != joint_list_.end()) {
                joints_[itr->second].q_ref = cnoid::radian(msg->command[i]);
            }
        }
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(JointController)
