/**************************************************/
// WINDOW SIZE 510 x 510 Quadran 4
// Ideally we want to keep the iamge in the middle, say 250*250
/**************************************************/
/////FIXX: FIX THE SUBSCRIBE TO AUTO CONTROL MODE
#define PI 3.141519

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/features2d/features2d.hpp"

//IF YOU USE KINETIC, YOU WILL USE OPENCV3, IN OPENCV3 YOU NEED TO INCLUDE BELOW LINE
//IF YOU USE OPENCV < 3, YOU DON'T NEED TO INCLUDE THIS HEADER FILE
// #include <opencv2/face.hpp>
#if CV_MAJOR_VERSION >= 3
  #include <opencv2/face.hpp>
#else
  #include <opencv2/contrib/contrib.hpp>
#endif

//For Debugging
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <sstream>
#include <math.h>
#include <stdint.h>


using namespace cv;
using namespace std;
//image window name
static const std::string OPENCV_WINDOW = "Image window";
static const std::string OPENCV_WINDOW2 = "Image window2";
static const std::string OPENCV_WINDOW3 = "Cropped Faces";
std::string image_address = "../catkin_ws/src/demo_elec4010k/image/";
std::string actual_image_path;
Mat croppedImage;
int image_counter = 0;
bool image_collection = false; //change to true for training

/******************/
// Face Recognizer
std::vector<Mat> images;
std::vector<int> labels;
int label_prediction = -1;
double confidence_level = -1;
// Obama | Avril | Legolas | Levi | Zhang | 
double image_scale[5] = {0.503, 0.583, 0.727, 0.569, 0.639};
const double THRESHOLD = 1500;
float distance_from_car=0;
const double TAN_22_5 = 0.414;
/******************/

bool auto_control_condition = false;
float x_pos, y_pos, orientation;

float x,y,y_x_ratio,angle_det,x_rel, y_rel;
/*************************/
// Global Marker
/************************/
visualization_msgs::Marker marker;
visualization_msgs::MarkerArray markers_data;
geometry_msgs::PoseStamped slam_data;
bool status_detection[5] = {true, true, true, true, true};
const int DATA_POINT = 900;

float angle_max;
float angle_increment;
float laser_scan_data[DATA_POINT];
float laser_radian_angle;
int data_number;

void LaserCallback(const sensor_msgs::LaserScan& msg) {
  angle_max = msg.angle_max;
  angle_increment = msg.angle_increment;
  for (int i = 0; i < DATA_POINT; i++) {
    if(msg.ranges[i] < 0) {
      laser_scan_data[i] = 0;
    } else if (msg.ranges[i] >50) {
      laser_scan_data[i] = 50;
    } else {
    laser_scan_data[i] = (float)msg.ranges[i];
    }
  }
}


// bool obama_detect = true;
void slamCallback(const geometry_msgs::PoseStamped& msg) {
  slam_data.pose.position.x = msg.pose.position.x;
  slam_data.pose.position.y = msg.pose.position.y;
  slam_data.pose.position.z = msg.pose.position.z;

  slam_data.pose.orientation.x = msg.pose.orientation.x;
  slam_data.pose.orientation.y = msg.pose.orientation.y;
  slam_data.pose.orientation.z = msg.pose.orientation.z;
  slam_data.pose.orientation.w = msg.pose.orientation.w;
}

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
  // Haar Cascade
  cv::CascadeClassifier face_cascade_;
  cv::HOGDescriptor hog_;
  /***************/
  // Detection
  // if you use openCV3
  #if CV_MAJOR_VERSION >= 3
    Ptr<face::FaceRecognizer> model;
  #else
    Ptr<cv::FaceRecognizer> model;
  #endif
  
   // if you use openCV2 
  // 
  
  /*******************/
  // Marker & subscribe to slam_out orientation
  ros::Publisher image_marker;
  ros::Subscriber slam_position;
  ros::Subscriber laser_range_data;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Take Hog Haar Params
    std::string face_cascade_name_std;
    if (!nh_.getParam("face_cascade_name", face_cascade_name_std))
      ROS_ERROR("Could not get face_cascade_name");
    cv::String face_cascade_name = face_cascade_name_std;

    // Load the hog descriptor
    hog_.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());

    // Load face detector HAAR cascades
    if(!face_cascade_.load(face_cascade_name))
      ROS_ERROR("--(!)Error loading face detector cascade \n");


    //Subscribe to VREP image and publish converted image
    image_sub_ = it_.subscribe("/vrep/image", 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/rosopencv_interface/image", 1);
    
    cmd_vel = nh_.advertise<geometry_msgs::Twist>("/vrep/cmd_vel", 1);
    chatter_debug = nh_.advertise<std_msgs::String>("/debugging",1000);  
    image_marker = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 100);

    markers_data.markers.resize(5);
    //OpenCV HighGUI calls to create a display window on start-up
    // Setting Markers
    for (int i = 0; i < 5; i ++) {
      markers_data.markers[i].header.frame_id = "map";
      markers_data.markers[i].header.stamp = ros::Time();
      markers_data.markers[i].ns = "my_namespace";
      markers_data.markers[i].id = i;
      markers_data.markers[i].type = visualization_msgs::Marker::SPHERE;
      markers_data.markers[i].action = visualization_msgs::Marker::ADD;
      markers_data.markers[i].pose.position.x = 1;
      markers_data.markers[i].pose.position.y = 1;
      markers_data.markers[i].pose.position.z = 0.6;
      markers_data.markers[i].pose.orientation.x = 0.0;
      markers_data.markers[i].pose.orientation.y = 0.0;
      markers_data.markers[i].pose.orientation.z = 0.0;
      markers_data.markers[i].pose.orientation.w = 1.0;
      markers_data.markers[i].scale.x = 1;
      markers_data.markers[i].scale.y = 0;
      markers_data.markers[i].scale.z = 1;
      markers_data.markers[i].color.a = 0; // Don't forget to set the alpha!
      markers_data.markers[i].color.r = 1.0/(i+1);
      markers_data.markers[i].color.g = 1.0/(i+1);
      markers_data.markers[i].color.b = 1.0;
      markers_data.markers[i].lifetime = ros::Duration();
    }

    /********************************************/
    image_marker.publish( markers_data );
    // Subscribe to ros_slam_out
    slam_position = nh_.subscribe("/slam_out_pose", 1, slamCallback);
    laser_range_data = nh_.subscribe("/vrep/scan",1,LaserCallback);


    cv::namedWindow(OPENCV_WINDOW);
    cv::namedWindow(OPENCV_WINDOW2);
    if(image_collection) cv::namedWindow(OPENCV_WINDOW3);

    /************/
    // Train Datasets
    std::string directories;
    //images for first person
    for (int i = 0; i < 42; i++) {
        std::stringstream training_directories;
        if (i < 10) {
            training_directories << image_address << "person1/0" << i << ".jpg";
        } else {
            training_directories << image_address << "person1/" << i << ".jpg";
        }
        directories = training_directories.str();
        images.push_back(imread(directories, CV_LOAD_IMAGE_GRAYSCALE)); labels.push_back(0);    
    }
    
    // images for second person
    for (int i = 0; i < 49; i++) {
        std::stringstream training_directories;
        if (i < 10) {
            training_directories << image_address << "person2/0" << i << ".jpg";
        } else {
            training_directories << image_address << "person2/" << i << ".jpg";
        }
        directories = training_directories.str();
        images.push_back(imread(directories, CV_LOAD_IMAGE_GRAYSCALE)); labels.push_back(1);    
    }

    // images for third person
    for (int i = 0; i < 110; i++) {
        std::stringstream training_directories;
        if (i < 10) {
            training_directories << image_address << "person3/0" << i << ".jpg";
        } else {
            training_directories << image_address << "person3/" << i << ".jpg";
        }
        directories = training_directories.str();
        images.push_back(imread(directories, CV_LOAD_IMAGE_GRAYSCALE)); labels.push_back(2);    
    }

    // images for fourth person
    for (int i = 0; i < 26; i++) {
        std::stringstream training_directories;
        if (i < 10) {
            training_directories << image_address << "person4/0" << i << ".jpg";
        } else {
            training_directories << image_address << "person4/" << i << ".jpg";
        }
        directories = training_directories.str();
        images.push_back(imread(directories, CV_LOAD_IMAGE_GRAYSCALE)); labels.push_back(3);    
    }

    // images for fifth person
    for (int i = 0; i < 44; i++) {
        std::stringstream training_directories;
        if (i < 10) {
            training_directories << image_address << "person5/0" << i << ".jpg";
        } else {
            training_directories << image_address << "person5/" << i << ".jpg";
        }
        directories = training_directories.str();
        images.push_back(imread(directories, CV_LOAD_IMAGE_GRAYSCALE)); labels.push_back(4);    
    }
    #if CV_MAJOR_VERSION >= 3
      model = face::createEigenFaceRecognizer();
    #else
      model = createEigenFaceRecognizer();
    #endif
    
    model->train(images,labels);

    /************/
  }

  //OpenCV HighGUI calls to destroy a display window on shutdown
  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
    cv::destroyWindow(OPENCV_WINDOW2);
    if(image_collection) cv::destroyWindow(OPENCV_WINDOW3);
  }

  //subscriber callback
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    //convert ROS image to CvImage that is suitable to work with OpenCV
    //must use 'try' and 'catch' format
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
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
    /* flip image code
     * flipcode = 0 -> flip on X axis
     * flipcode > 0 -> flip on Y axis
     * flipcode < 0 -> flip on both axis
    */
    cv::Mat flippedImage;
    cv::flip(cv_ptr->image, flippedImage, 1);
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
    cv::imshow(OPENCV_WINDOW2, imgThresholded);

    // =============================================
    // HOG HAAR
    // =============================================
    std::vector<cv::Rect> detected_faces;
    cv::Mat im_gray;
    cv:cvtColor(flippedImage, im_gray, CV_BGR2GRAY);
    cv::equalizeHist( im_gray, im_gray );
    face_cascade_.detectMultiScale(im_gray, detected_faces, 1.1, 4, 0|CV_HAAR_SCALE_IMAGE, cv::Size(50, 50));
    unsigned index = 0;
    if(detected_faces.size() >= 1) {
      // Do Something
      croppedImage = flippedImage(detected_faces[index]);
      /************************/
      // Image Detection
      cv::Mat im_resized;
      label_prediction = -1;
      confidence_level = -1;
      cv::resize(croppedImage, im_resized, cv::Size(55,55));
      cv::cvtColor(im_resized, im_resized, CV_BGR2GRAY);
      model->predict(im_resized, label_prediction, confidence_level);
      /***********************/
      /*******/
      // Classify person
      // Add confidence level to differentiate w ground
      std::stringstream text_to_put;
      std::string text_to_write;
      /******************************************/
      // image_scale
      distance_from_car = detected_faces[index].width;
      // Pixel Representing 1m image
      distance_from_car /= image_scale[label_prediction];
      // Finding the actual distance
      distance_from_car = 256/(TAN_22_5*distance_from_car);
      /******************************************/
      // LOCATION OF IMAGE
      x = 256 - (detected_faces[index].x + detected_faces[index].width/2);
      y = 512 - (detected_faces[index].y + detected_faces[index].height/2);
      y_x_ratio = x/y;
      angle_det = atan(abs(y_x_ratio));
      switch(label_prediction) {
        case 0:
          text_to_put << "Obama ";
          break;
        case 1:
          text_to_put << "Avril ";
          break;
        case 2:
          text_to_put << "Legolas ";
          break;
        case 3:
          text_to_put << "Levi ";
          break;
        case 4:
          text_to_put << "Zhang ";
          break;
        default:
          break;
      }
      // text_to_put << slam_data->pose.position.x << ' ' << slam_data->pose.position.y << ' ' << slam_data->pose.position.z; 
      text_to_put << label_prediction << " " << confidence_level;
      text_to_write = text_to_put.str();

      if(y_x_ratio > 0) {
        laser_radian_angle = PI/2.0 - angle_det;
      } else {
        laser_radian_angle = PI/2.0 + angle_det;
      }
      data_number = abs(laser_radian_angle/ angle_increment);
  
      /******************************************
      *****************************************
      *********************************************
      DEBUGGING SECTION DELETE LATER 
      *******************************************/
      std::stringstream DEBUG;
      std_msgs::String debug_msgg;
      DEBUG << "DISTANCE IS : " << distance_from_car<< "x: "<< x << " y: "<< y << " ratio: "<<y_x_ratio <<" ";
      DEBUG << "Laser Distance is " << data_number << " ";
      DEBUG << laser_scan_data[data_number];
      debug_msgg.data = DEBUG.str();
      chatter_debug.publish(debug_msgg);

      /******************************************
      *****************************************
      *********************************************
      DEBUGGING SECTION DELETE LATER 
      *******************************************/
      // Update actual dist
      if(label_prediction != 1) {
        if(laser_scan_data[data_number] != 0) {
          distance_from_car = laser_scan_data[data_number];  
        }
      }

      if (status_detection[label_prediction]) { 

        if(y_x_ratio > 0) {
          markers_data.markers[label_prediction].pose.position.x = distance_from_car*sin(angle_det);
        } else {
          markers_data.markers[label_prediction].pose.position.x = -distance_from_car*sin(angle_det);
        }
        markers_data.markers[label_prediction].pose.position.y = -distance_from_car*cos(angle_det);

        // Now perform matrix transformation to absolute coordinate
        // delta x absolute = x read cos theta - yread sin theta
        // delta y absolute = x read sin theta + y read cos theta
        // theta is obtained from 2*acos(w)
        if (slam_data.pose.orientation.z > 0) {
          angle_det = 2.0*acos(slam_data.pose.orientation.w);
        } else {
          angle_det = -2.0*acos(slam_data.pose.orientation.w);
        }
        x_rel = markers_data.markers[label_prediction].pose.position.x * cos(angle_det) - markers_data.markers[label_prediction].pose.position.y * sin(angle_det);
        y_rel = markers_data.markers[label_prediction].pose.position.x * sin(angle_det) + markers_data.markers[label_prediction].pose.position.y * cos(angle_det);
        // markers_data.markers[label_prediction].pose.orientation.w = slam_data.pose.orientation.w;

        markers_data.markers[label_prediction].pose.position.x = slam_data.pose.position.x + x_rel;
        markers_data.markers[label_prediction].pose.position.y = slam_data.pose.position.y + y_rel;

        markers_data.markers[label_prediction].scale.x = abs(1.0*cos(angle_det));
        markers_data.markers[label_prediction].scale.y = abs(1.0*sin(angle_det));
      }
      /*******/
      // Draw on screen.
      if (image_collection) {
        cv::imshow(OPENCV_WINDOW3, croppedImage);
        std::stringstream ss_path;
        ss_path << image_address << "image" << image_counter << ".jpg";
        actual_image_path = ss_path.str();
        image_counter++;
        cv::imwrite(actual_image_path ,croppedImage);
      }
      /**********************************************/
      // Give Marker only if confidence level <= threshold (distance)
      if (confidence_level <= THRESHOLD) {
        cv::putText(flippedImage, text_to_write, Point2f(detected_faces[index].x, detected_faces[index].y), FONT_HERSHEY_PLAIN, 2, cv::Scalar(0,255,255));
        cv::rectangle(flippedImage, detected_faces[index], cv::Scalar(255),5);       
        // markers[label_prediction] = marker;
        markers_data.markers[label_prediction].color.a = 1.0;
        // status_detection[label_prediction] = false;
        // image_marker.publish( markers_data );
      }
      /******************************************************/
    } 
    cv::imshow(OPENCV_WINDOW, flippedImage);

    cv::waitKey(3);
    cv_ptr->image = flippedImage;
    // Output modified image stream
    image_pub_.publish(cv_ptr->toImageMsg());
    image_marker.publish( markers_data );
   }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rosopencv_interface");
  ImageConverter ic;
  ros::spin();
  return 0;
}