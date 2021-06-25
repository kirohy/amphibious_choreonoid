#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

class ImageConverter {
  private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    cv_bridge::CvImagePtr cv_ptr_;

  public:
    ImageConverter() : it_(nh_) {
        image_sub_ = it_.subscribe("/image_raw", 1, &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/image_result", 1);
    }

    ~ImageConverter() {
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
            cv::Point2f center;
            float radius;
            cv::minEnclosingCircle(contours.at(max_area_contour), center, radius);
            cv::circle(cv_ptr_->image, center, radius, cv::Scalar(0, 0, 255), 3, 4);

            cv::imshow("Result Image", cv_ptr_->image);
            cv::waitKey(3);

            image_pub_.publish(cv_ptr_->toImageMsg());
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "calc_motions");
    ImageConverter ic;
    ros::spin();
    return 0;
}
