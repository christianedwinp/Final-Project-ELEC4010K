#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//image window name
static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    //Subscribe to VREP image and publish converted image
    image_sub_ = it_.subscribe("/vrep/image", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/rosopencv_interface/image", 1);

    //OpenCV HighGUI calls to create a display window on start-up
    cv::namedWindow(OPENCV_WINDOW);
  }

  //OpenCV HighGUI calls to destroy a display window on shutdown
  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  //subscriber callback
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    //convert ROS image to CvImage that is suitable to work with OpenCV
    //must use 'try' and 'catch' format
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      //convert to suitable color encoding scheme
      /*color encoding scheme:
       * mono8  : CV_8UC1, grayscale image
       * mono16 : CV_16UC1, 16-bit grayscale image
       * bgr8   : CV_8UC3, color image with blue-green-red color order
       * rgb8   : CV_8UC3, color image with red-green-blue color order
       * bgra8  : CV_8UC4, BGR color image with an alpha channel
       * rgba8  : CV_8UC4, RGB color image with an alpha channel
      */
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat flippedImage;
    /* flip image
    * flipcode = 0 -> flip on X axis
    * flipcode > 0 -> flip on Y axis
    * flipcode < 0 -> flip on both axis
    */
    cv::flip(cv_ptr->image, flippedImage, 1);

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, flippedImage);
    cv::waitKey(3);

    // Output modified image stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rosopencv_interface");
  ImageConverter ic;
  ros::spin();
  return 0;
}