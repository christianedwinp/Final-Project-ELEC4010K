#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//image window name
static const std::string OPENCV_WINDOW = "Processed Image window";

class ImageProcessing
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber procimage_sub_;
  image_transport::Publisher procimage_pub_;

public:
  ImageProcessing()
    : it_(nh_)
  {
    //Subscribe to rosopencv_interface image and publish to RVIZ
    procimage_sub_ = it_.subscribe("/rosopencv_interface/image", 1,
      &ImageProcessing::imageCb, this);
    procimage_pub_ = it_.advertise("/rviz/processed_image", 1);

    //OpenCV HighGUI calls to create a display window on start-up
    cv::namedWindow(OPENCV_WINDOW);
  }

  //OpenCV HighGUI calls to destroy a display window on shutdown
  ~ImageProcessing()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  //subscriber callback
  void imageCb(const cv_bridge::CvImagePtr& cv_ptr)
  {
    //DO IMAGE PROCESSING HERE

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr);
    cv::waitKey(3);

    // Output modified image stream
    procimage_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "opencv_imgproc");
  ImageProcessing ip;
  ros::spin();
  return 0;
}