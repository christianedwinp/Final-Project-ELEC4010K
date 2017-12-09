/**************************************************/
// WINDOW SIZE 510 x 510 Quadran 4
// Ideally we want to keep the iamge in the middle, say 250*250
/**************************************************/
/////FIXX: FIX THE SUBSCRIBE TO AUTO CONTROL MODE
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>

#include "opencv2/features2d/features2d.hpp"
//For Debugging
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <sstream>


using namespace cv;
//image window name
static const std::string OPENCV_WINDOW = "Raw Flipped Image";
static const std::string OPENCV_WINDOW2 = "Blob tracking";

Mat img_1, img_2, img_3, img_4, img_5;


bool auto_control_condition = false;
int counter = 0;
float x_pos, y_pos, orientation;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  /****************/
  ros::Publisher cmd_vel;
  geometry_msgs::Twist vel;
  /***************/
  // For debugging
  ros::Publisher chatter_debug;
  /***************/

public:
  ImageConverter()
    : it_(nh_)
  {
    //Subscribe to VREP image and publish converted image
    image_sub_ = it_.subscribe("/vrep/image", 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/rosopencv_interface/image", 1);
    
    cmd_vel = nh_.advertise<geometry_msgs::Twist>("/vrep/cmd_vel", 1);
    chatter_debug = nh_.advertise<std_msgs::String>("/debugging",1000);  
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

    // =============================================
    // FLIPPED IMAGE
    // =============================================
    cv::Mat flippedImage;
    /* flip image
     * flipcode = 0 -> flip on X axis
     * flipcode > 0 -> flip on Y axis
     * flipcode < 0 -> flip on both axis
    */
    cv::flip(cv_ptr->image, flippedImage, 1);
    cv::imshow(OPENCV_WINDOW, flippedImage);
    
    // =============================================
    // BLOB TRACKING
    // =============================================

    cv::Mat test, imgThresholded;
    // Use HSV param for yellow tracking
    int iLowH = 20;
    int iHighH = 30;

    int iLowS = 100; 
    int iHighS = 255;

    int iLowV = 100;
    int iHighV = 255;
        
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

    int posX = dM10 / dArea;
    int posY = dM01 / dArea;
    int lastX,lastY; 
    /****************/
    // Debugging
    std_msgs::String debug_msg;
    std::stringstream ss;
    // Check ball detected or not
    if (posY > 280) {
      auto_control_condition = true;
    }
    if (dArea > 100000) {
      ss << "xPos: " << posX << " Ypos: " << posY << " area : "<< dArea <<' '<< auto_control_condition;
      debug_msg.data = ss.str();
      chatter_debug.publish(debug_msg);
      /*****************************/
      // Perform Control
      /* Hypothetically We split img to 4 quadrans + zero axis*/
      /* We want to keep x & y in the middle which is about 250 */
      /* Use 240 as Threshold Value */
      /* If X  < 240 angle orient to left */
      /* If X is in between angle stays */
      /* If X > 260 angle orient to right */
    
      if (posX < 240) {
        vel.angular.z = 0.6;
      } else if (posX > 260) {
        vel.angular.z = -0.6;
      } else {
        vel.angular.z = 0;
      }

      /* Y < 330 go forward */
      /* Y > 360 slowdown */
      if (posY < 330) {
        vel.linear.x = 0.4;
      } else if (posY > 360) {
        vel.linear.x = -0.3;
      } else {
        vel.linear.x = 0.2;
      }
      
      if(auto_control_condition) {
        cmd_vel.publish(vel);
      }
      /*****************************/
    }
    /****************/
    // IF LASTX & LASTY suddenly gone missing 

    /***************/

    cv::imshow(OPENCV_WINDOW2, imgThresholded);
    cv::waitKey(3);

    // =============================================
    // SEND BACK EDITED IMAGE IN CV TO ROS SENSOR MESSAGE
    // =============================================
    //change the cv image as suitable
    sensor_msgs::ImagePtr publishback2ROS = cv_bridge::CvImage(std_msgs::Header(), "bgr8", flippedImage).toImageMsg(); 
    image_pub_.publish(publishback2ROS);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rosopencv_interface");

  //LOAD IMAGE DATASET
  if( argc != 6){
    std::cout << "Missing arguments" << std::endl;
    return -1;
  }
         
  img_1 = imread( argv[1], CV_LOAD_IMAGE_COLOR ); //load pic001.jpg
  img_2 = imread( argv[2], CV_LOAD_IMAGE_COLOR ); //load pic002.jpg
  img_3 = imread( argv[3], CV_LOAD_IMAGE_COLOR ); //load pic003.jpg
  img_4 = imread( argv[4], CV_LOAD_IMAGE_COLOR ); //load pic004.jpg
  img_5 = imread( argv[5], CV_LOAD_IMAGE_COLOR ); //load pic005.jpg

  if( !img_1.data || !img_2.data || !img_3.data || !img_4.data || !img_5.data )
      { std::cout<< " --(!) Error reading images " << std::endl; return -1; }

  ImageConverter ic;
  ros::spin();
  return 0;
}