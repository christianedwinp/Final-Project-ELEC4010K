/**************************************************/
// WINDOW SIZE 510 x 510 Quadran 4
// Ideally we want to keep the iamge in the middle, say 250*250
/**************************************************/
/////FIXX: FIX THE SUBSCRIBE TO AUTO CONTROL MODE
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/face.hpp"
//For Debugging
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <sstream>


using namespace cv;
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
const double THRESHOLD = 1500;
/******************/


// Mat img_1, img_2, img_3, img_4, img_5;
bool auto_control_condition = false;
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
  // Haar Cascade
  cv::CascadeClassifier face_cascade_;
  cv::HOGDescriptor hog_;
  /***************/
  // Detection
  Ptr<face::FaceRecognizer> model;

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
    //OpenCV HighGUI calls to create a display window on start-up
    cv::namedWindow(OPENCV_WINDOW);
    cv::namedWindow(OPENCV_WINDOW2);
    if(image_collection) cv::namedWindow(OPENCV_WINDOW3);

    /************/
    // Train Datasets
    std::string directories;
    // for (int i = 1; i < 6; i++) {
    //   std::stringstream training_directories;
    //   training_directories << image_address << "pic00" << i << ".jpg";
    //   directories = training_directories.str();
    //   images.push_back(imread(directories, CV_LOAD_IMAGE_GRAYSCALE)); labels.push_back(i-1);
    // }
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
    model = face::createEigenFaceRecognizer();
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

  // cv::Mat ImageDetector(const cv::Mat flippedImage)
  // {
  //  //-- Step 1: Detect the keypoints using SURF Detector
  //   int minHessian = 400;
  //   SurfFeatureDetector detector( minHessian );
  //   std::vector<KeyPoint> keypoints_1, keypoints_2, keypoints_3, keypoints_4, keypoints_5, keypoints_stream;
  //   detector.detect( img_1, keypoints_1 );
  //   // detector.detect( img_2, keypoints_2 );
  //   // detector.detect( img_3, keypoints_3 );
  //   // detector.detect( img_4, keypoints_4 );
  //   // detector.detect( img_5, keypoints_5 );
  //   detector.detect( flippedImage, keypoints_stream );

  //   //-- Step 2: Calculate descriptors (feature vectors)
  //   // cv::Ptr<cv::DescriptorExtractor> extractor = new cv::SurfDescriptorExtractor;
  //   SurfDescriptorExtractor extractor;
  //   Mat descriptors_1, descriptors_2, descriptors_3, descriptors_4, descriptors_5, descriptors_stream;
  //   extractor.compute( img_1, keypoints_1, descriptors_1 );
  //   // extractor.compute( img_2, keypoints_2, descriptors_2 );
  //   // extractor.compute( img_3, keypoints_3, descriptors_3 );
  //   // extractor.compute( img_4, keypoints_4, descriptors_4 );
  //   // extractor.compute( img_5, keypoints_5, descriptors_5 );
  //   extractor.compute( flippedImage, keypoints_stream, descriptors_stream );

  //   // //-- Step 3: Matching descriptor vectors using FLANN matcher 
  //   FlannBasedMatcher matcher;
  //   //matching process pic001.jpg
  //   std::vector< DMatch > matches;
  //   matcher.match( descriptors_1, descriptors_stream, matches );

  //   // //-- Quick calculation of max and min distances between keypoints_1
  //   double max_dist = 0; double min_dist = 100;
  //   for( int i = 0; i < descriptors_1.rows; i++ ){ 
  //     double dist = matches[i].distance;
  //     if( dist < min_dist ) min_dist = dist;
  //     if( dist > max_dist ) max_dist = dist;
  //   }
  //   printf("-- Max dist : %f \n", max_dist );
  //   printf("-- Min dist : %f \n", min_dist );

  //   // //-- Get good matches pic001.jpg
  //   std::vector< DMatch > good_matches;
  //   for( int i = 0; i < descriptors_1.rows; i++ ){ 
  //     if( matches[i].distance < 3*min_dist ){ 
  //       good_matches.push_back( matches[i]);
  //     }
  //   }
    
  //   Mat img_matches;
  //   drawMatches( img_1, keypoints_1, flippedImage, keypoints_stream,
  //              good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
  //              vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );


  //   // //-- Localize pic001.jpg
  //   std::vector<Point2f> obj1;
  //   std::vector<Point2f> stream;
  //   for( int i = 0; i < good_matches.size(); i++ ){
  //       //-- Get the keypoints from the good matches
  //       obj1.push_back( keypoints_1[ good_matches[i].queryIdx ].pt );
  //       stream.push_back( keypoints_stream[ good_matches[i].trainIdx ].pt );
  //   }
  //   Mat H = findHomography( obj1, stream, CV_RANSAC );

  //   // //-- Get the corners from the pic001.jpg
  //   std::vector<Point2f> obj1_corners(4);
  //   obj1_corners[0] = cvPoint(0,0); 
  //   obj1_corners[1] = cvPoint( img_1.cols, 0 );
  //   obj1_corners[2] = cvPoint( img_1.cols, img_1.rows ); 
  //   obj1_corners[3] = cvPoint( 0, img_1.rows );
  //   std::vector<Point2f> stream_corners(4);

  //   perspectiveTransform( obj1_corners, stream_corners, H);

  //   // //-- Draw pic001.jpg border line in camera stream
  //   line( img_matches, stream_corners[0] + Point2f( img_1.cols, 0), stream_corners[1] + Point2f( img_1.cols, 0), Scalar(0, 255, 0), 4 );
  //   line( img_matches, stream_corners[1] + Point2f( img_1.cols, 0), stream_corners[2] + Point2f( img_1.cols, 0), Scalar( 0, 255, 0), 4 );
  //   line( img_matches, stream_corners[2] + Point2f( img_1.cols, 0), stream_corners[3] + Point2f( img_1.cols, 0), Scalar( 0, 255, 0), 4 );
  //   line( img_matches, stream_corners[3] + Point2f( img_1.cols, 0), stream_corners[0] + Point2f( img_1.cols, 0), Scalar( 0, 255, 0), 4 );

  //   // Update GUI Window
  //   cv::imshow(OPENCV_WINDOW, img_matches);
  //   return img_matches;
    
  // }

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
    // cv::imshow(OPENCV_WINDOW, flippedImage);

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
      if(detected_faces.size() > 1) {
        //do comparison
      }
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
          text_to_put << "Chinese dude ";
          break;
        default:
          break;
      }
      text_to_put << label_prediction << " " << confidence_level;
      text_to_write = text_to_put.str();
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
      }
      /******************************************************/
    }
    cv::imshow(OPENCV_WINDOW, flippedImage);

    cv::waitKey(3);
    cv_ptr->image = flippedImage;
    // Output modified image stream
    image_pub_.publish(cv_ptr->toImageMsg());
    // =============================================
    // SEND BACK EDITED IMAGE IN CV TO ROS SENSOR MESSAGE
    // =============================================
    // cv_bridge::CvImage img_bridge;
    // sensor_msgs::Image img_msg;
    // std_msgs::Header header;
    // //change the output image as suitable
    // img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, flippedImage); //convert CVimage to ROS
    // img_bridge.toImageMsg(img_msg); 
    // image_pub_.publish(img_msg); 
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rosopencv_interface");
  
  //LOAD IMAGE DATASET
  // if( argc != 2)
  //   {
  //    std::cout << "Missing arguments" << std::endl;
  //    return -1;
  //   }
    
  // img_1 = imread( argv[1], CV_LOAD_IMAGE_COLOR ); //load pic001.jpg
  // img_2 = imread( argv[2], CV_LOAD_IMAGE_COLOR ); //load pic002.jpg
  // img_3 = imread( argv[3], CV_LOAD_IMAGE_COLOR ); //load pic003.jpg
  // img_4 = imread( argv[4], CV_LOAD_IMAGE_COLOR ); //load pic004.jpg
  // img_5 = imread( argv[5], CV_LOAD_IMAGE_COLOR ); //load pic005.jpg

  // if( !img_1.data || !img_2.data || !img_3.data || !img_4.data || !img_5.data )
  //     { std::cout<< " --(!) Error reading images " << std::endl; return -1; }

  //   img_1 = imread( argv[1], CV_LOAD_IMAGE_COLOR ); //load pic001.jpg

  // if( !img_1.data )
  //     { std::cout<< " --(!) Error reading images " << std::endl; return -1; }

  ImageConverter ic;
  ros::spin();
  return 0;
}