#include <cnoid/SimpleController>
#include <cnoid/Camera>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

class CameraController : public cnoid::SimpleController
{
private:
    cnoid::CameraPtr camera;
    cnoid::ScopedConnection cameraConnection;
    std::unique_ptr<ros::NodeHandle> nh;
    image_transport::Publisher cameraImagePublisher;
    double time;
    double timeStep;

public:
    virtual bool configure(cnoid::SimpleControllerConfig* config) override
    {
      nh.reset(new ros::NodeHandle);
      image_transport::ImageTransport imageTransport(*nh);
      cameraImagePublisher = imageTransport.advertise("/amphibious_tank/camera/image", 1);
      return true;
    }

    virtual bool initialize(cnoid::SimpleControllerIO* io) override
    {
        camera = io->body()->findDevice<cnoid::Camera>("RGBCamera");

        io->enableInput(camera);

        cameraConnection = camera->sigStateChanged().connect([&] (){ publishCameraImage(); });

        time = 0.0;
        timeStep = io->timeStep();

        return true;
    }

    virtual bool control() override
    {
        time += timeStep;
        return true;
    }

    void publishCameraImage()
    {
        sensor_msgs::Image rosImage;
        rosImage.header.stamp.fromSec(time);
        rosImage.header.frame_id = camera->name();
        auto& srcImage = camera->image();
        rosImage.height = srcImage.height();
        rosImage.width = srcImage.width();
        if(srcImage.numComponents() == 3){
            rosImage.encoding = sensor_msgs::image_encodings::RGB8;
        } else if (srcImage.numComponents() == 1){
            rosImage.encoding = sensor_msgs::image_encodings::MONO8;
        } else {
            ROS_WARN("unsupported image component number: %i", srcImage.numComponents());
        }
        rosImage.is_bigendian = 0;
        rosImage.step = srcImage.width() * srcImage.numComponents();
        rosImage.data.resize(rosImage.step * rosImage.height);
        std::memcpy(&(rosImage.data[0]), &(srcImage.pixels()[0]), rosImage.step * rosImage.height);
        cameraImagePublisher.publish(rosImage);
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(CameraController)

