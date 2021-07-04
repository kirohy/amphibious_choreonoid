#include <cnoid/SimpleController>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

class RobotStateController : public cnoid::SimpleController {
  private:
    std::unique_ptr<ros::NodeHandle> nh_;
    std::unique_ptr<ros::NodeHandle> pnh_;
    ros::Timer timer_;
    ros::Publisher joint_state_pub_;
    sensor_msgs::JointState joint_state_;
    tf2_ros::StaticTransformBroadcaster static_tf_pub_;
    tf2_ros::TransformBroadcaster odom_pub_;
    cnoid::BodyPtr body_;

  public:
    virtual bool configure(cnoid::SimpleControllerConfig *config) override {
        nh_.reset(new ros::NodeHandle(config->body()->name()));
        pnh_.reset(new ros::NodeHandle("~"));

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

        XmlRpc::XmlRpcValue goal_pos;
        XmlRpc::XmlRpcValue goal_front_pos;
        pnh_->getParam("goal", goal_pos);
        pnh_->getParam("goal_front", goal_front_pos);

        geometry_msgs::TransformStamped static_tf;
        auto start_time = ros::Time::now();

        static_tf.header.stamp = start_time;
        static_tf.header.frame_id = "map";
        static_tf.child_frame_id = "goal";
        static_tf.transform.translation.x = goal_pos["x"];
        static_tf.transform.translation.y = goal_pos["y"];
        static_tf.transform.translation.z = 0.0;
        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, 0.0);
        static_tf.transform.rotation.x = quat.x();
        static_tf.transform.rotation.y = quat.y();
        static_tf.transform.rotation.z = quat.z();
        static_tf.transform.rotation.w = quat.w();
        static_tf_pub_.sendTransform(static_tf);

        static_tf.header.stamp = start_time;
        static_tf.header.frame_id = "map";
        static_tf.child_frame_id = "goal_front";
        static_tf.transform.translation.x = goal_front_pos["x"];
        static_tf.transform.translation.y = goal_front_pos["y"];
        static_tf.transform.translation.z = 0.0;
        quat.setRPY(0.0, 0.0, 0.0);
        static_tf.transform.rotation.x = quat.x();
        static_tf.transform.rotation.y = quat.y();
        static_tf.transform.rotation.z = quat.z();
        static_tf.transform.rotation.w = quat.w();
        static_tf_pub_.sendTransform(static_tf);

        auto pos = body_->rootLink()->T();
        Eigen::Quaterniond quat_robot(pos.rotation());
        static_tf.header.stamp = start_time;
        static_tf.header.frame_id = "map";
        static_tf.child_frame_id = "initial_position";
        static_tf.transform.translation.x = pos.translation().x();
        static_tf.transform.translation.y = pos.translation().y();
        static_tf.transform.translation.z = pos.translation().z();
        static_tf.transform.rotation.x = quat_robot.x();
        static_tf.transform.rotation.y = quat_robot.y();
        static_tf.transform.rotation.z = quat_robot.z();
        static_tf.transform.rotation.w = quat_robot.w();
        static_tf_pub_.sendTransform(static_tf);

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
        odom.header.frame_id = "map";
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
