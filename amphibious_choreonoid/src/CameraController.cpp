#include <cnoid/Camera>
#include <cnoid/SimpleController>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

class CameraController : public cnoid::SimpleController {
  private:
    cnoid::CameraPtr cam_;
    cnoid::ScopedConnection cam_connection_;
    std::unique_ptr<ros::NodeHandle> nh_;
    image_transport::Publisher image_pub_;
    double time_;
    double time_step_;

  public:
    virtual bool configure(cnoid::SimpleControllerConfig *config) override {
        nh_.reset(new ros::NodeHandle);
        image_transport::ImageTransport imageTransport(*nh_);
        image_pub_ = imageTransport.advertise("/amphibious_tank/camera/image", 1);
        return true;
    }

    virtual bool initialize(cnoid::SimpleControllerIO *io) override {
        cam_ = io->body()->findDevice<cnoid::Camera>("RGBCamera");

        io->enableInput(cam_);

        cam_connection_ = cam_->sigStateChanged().connect([&]() { publishCameraImage(); });

        time_ = 0.0;
        time_step_ = io->timeStep();

        return true;
    }

    virtual bool control() override {
        time_ += time_step_;
        return true;
    }

    void publishCameraImage() {
        sensor_msgs::Image rosImage;
        rosImage.header.stamp.fromSec(time_);
        rosImage.header.frame_id = cam_->name();
        auto &srcImage = cam_->image();
        rosImage.height = srcImage.height();
        rosImage.width = srcImage.width();
        if (srcImage.numComponents() == 3) {
            rosImage.encoding = sensor_msgs::image_encodings::RGB8;
        } else if (srcImage.numComponents() == 1) {
            rosImage.encoding = sensor_msgs::image_encodings::MONO8;
        } else {
            ROS_WARN("unsupported image component number: %i", srcImage.numComponents());
        }
        rosImage.is_bigendian = 0;
        rosImage.step = srcImage.width() * srcImage.numComponents();
        rosImage.data.resize(rosImage.step * rosImage.height);
        std::memcpy(&(rosImage.data[0]), &(srcImage.pixels()[0]), rosImage.step * rosImage.height);
        image_pub_.publish(rosImage);
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(CameraController)
