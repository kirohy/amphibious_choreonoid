#include <cnoid/SimpleController>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

class RobotStateController : public cnoid::SimpleController {
  private:
    std::unique_ptr<ros::NodeHandle> nh_;
    ros::Timer timer_;
    ros::Publisher joint_state_pub_;
    sensor_msgs::JointState joint_state_;
    tf2_ros::StaticTransformBroadcaster goal_tf_pub_;
    tf2_ros::TransformBroadcaster odom_pub_;
    cnoid::BodyPtr body_;

  public:
    virtual bool configure(cnoid::SimpleControllerConfig *config) override {
        nh_.reset(new ros::NodeHandle(config->body()->name()));
        joint_state_pub_ = nh_->advertise<sensor_msgs::JointState>("joint_state", 1);

        return true;
    }

    virtual bool initialize(cnoid::SimpleControllerIO *io) override {
        body_ = io->body();
        io->enableInput(body_->rootLink(), LinkPosition);

        int n = body_->numJoints();
        joint_state_.name.resize(n);
        joint_state_.position.resize(n);
        joint_state_.velocity.resize(n);
        joint_state_.effort.resize(n);

        for (int i = 0; i < n; ++i) {
            auto joint = body_->joint(i);
            io->enableInput(joint, JointDisplacement | JointVelocity | JointEffort);
            joint_state_.name[i] = joint->name();
        }

        geometry_msgs::TransformStamped goal;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "world";
        goal.child_frame_id = "goal";
        goal.transform.translation.x = -6.5;
        goal.transform.translation.y = 6.5;
        goal.transform.translation.z = 0.0;
        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, 0.0);
        goal.transform.rotation.x = quat.x();
        goal.transform.rotation.y = quat.y();
        goal.transform.rotation.z = quat.z();
        goal.transform.rotation.w = quat.w();
        goal_tf_pub_.sendTransform(goal);

        geometry_msgs::TransformStamped goal_front;
        goal_front.header.stamp = ros::Time::now();
        goal_front.header.frame_id = "world";
        goal_front.child_frame_id = "goal_front";
        goal_front.transform.translation.x = -6.5;
        goal_front.transform.translation.y = 4.5;
        goal_front.transform.translation.z = 0.0;
        quat.setRPY(0.0, 0.0, 0.0);
        goal_front.transform.rotation.x = quat.x();
        goal_front.transform.rotation.y = quat.y();
        goal_front.transform.rotation.z = quat.z();
        goal_front.transform.rotation.w = quat.w();
        goal_tf_pub_.sendTransform(goal_front);

        timer_ = nh_->createTimer(ros::Duration(0.01), &RobotStateController::timer_callback, this);

        return true;
    }

    void timer_callback(const ros::TimerEvent &e) {
        auto now = ros::Time::now();

        for (int i = 0; i < body_->numJoints(); ++i) {
            auto joint = body_->joint(i);
            joint_state_.position[i] = joint->q();
            joint_state_.velocity[i] = joint->dq();
            joint_state_.effort[i] = joint->u();
        }
        joint_state_.header.stamp = now;
        joint_state_pub_.publish(joint_state_);

        auto pos = body_->rootLink()->T();
        Eigen::Quaterniond quat(pos.rotation());
        geometry_msgs::TransformStamped odom;
        odom.header.stamp = now;
        odom.header.frame_id = "world";
        odom.child_frame_id = "base_link";
        odom.transform.translation.x = pos.translation().x();
        odom.transform.translation.y = pos.translation().y();
        odom.transform.translation.z = pos.translation().z();
        odom.transform.rotation.x = quat.x();
        odom.transform.rotation.y = quat.y();
        odom.transform.rotation.z = quat.z();
        odom.transform.rotation.w = quat.w();
        odom_pub_.sendTransform(odom);
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(RobotStateController)
