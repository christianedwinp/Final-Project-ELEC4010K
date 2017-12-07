#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/features2d/features2d.hpp"

using namespace cv;
//image window name
static const std::string OPENCV_WINDOW = "Image window";
static const std::string OPENCV_WINDOW2 = "Image window2";

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
    cv::namedWindow(OPENCV_WINDOW2);
  }

  //OpenCV HighGUI calls to destroy a display window on shutdown
  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
    cv::destroyWindow(OPENCV_WINDOW2);
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

    cv::Mat flippedImage, test, imgThresholded;
    // Use HSV param for yellow tracking
    int iLowH = 20;
    int iHighH = 30;

    int iLowS = 100; 
    int iHighS = 255;

    int iLowV = 100;
    int iHighV = 255;
    
    //flip image 
    cv::flip(cv_ptr->image, flippedImage, 1);
    cv::imshow(OPENCV_WINDOW, flippedImage);
    
    // Map BGR to HSV
    cvtColor(flippedImage, test, COLOR_BGR2HSV);
    // Update GUI Window
    inRange(test, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded);
    //morphological opening (removes small objects from the foreground)
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

     //morphological closing (removes small holes from the foreground)
    dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

     //Calculate the moments of the thresholded image
    Moments oMoments = moments(imgThresholded);

    double dM01 = oMoments.m01;
    double dM10 = oMoments.m10;
    double dArea = oMoments.m00;


    cv::imshow(OPENCV_WINDOW2, imgThresholded);
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