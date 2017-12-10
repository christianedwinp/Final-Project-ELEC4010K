//====================================================
//LAUNCH FILE BUAT JALANIN FLANN MATCHER
//====================================================
 <node pkg="demo_elec4010k" type="rosopencv_interface" name="rosopencv_interface"
    args="$(find demo_elec4010k)/src/picture/pic001.jpg
    $(find demo_elec4010k)/src/picture/pic002.jpg
    $(find demo_elec4010k)/src/picture/pic003.jpg
    $(find demo_elec4010k)/src/picture/pic004.jpg
    $(find demo_elec4010k)/src/picture/pic005.jpg"/>

//====================================================
//TARO DI PALING ATAS SEBAGI GLOBAL VARIABLE
//====================================================
cv::Mat img_1, img_2, img_3, img_3, img_4, img_5;


//====================================================
//TARO DI MAIN FUNCTION
//====================================================
  if( argc != 2)
    {
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

    img_1 = imread( argv[1], CV_LOAD_IMAGE_COLOR ); //load pic001.jpg

  if( !img_1.data )
      { std::cout<< " --(!) Error reading images " << std::endl; return -1; }

//====================================================
//TARO DI IMAGE CONVERTER CLASS SEBAGAI FUNCTION
//====================================================
  cv::Mat FlannMatcher_Homography(const cv::Mat flippedImage)
  {
  	//-- Step 1: Detect the keypoints using SURF Detector
    int minHessian = 400;
    SurfFeatureDetector detector( minHessian );
    std::vector<KeyPoint> keypoints_1, keypoints_2, keypoints_3, keypoints_4, keypoints_5, keypoints_stream;
    detector.detect( img_1, keypoints_1 );
    detector.detect( img_2, keypoints_2 );
    detector.detect( img_3, keypoints_3 );
    detector.detect( img_4, keypoints_4 );
    detector.detect( img_5, keypoints_5 );
    detector.detect( flippedImage, keypoints_stream );

    //-- Step 2: Calculate descriptors (feature vectors)
    SurfDescriptorExtractor extractor;
    Mat descriptors_1, descriptors_2, descriptors_3, descriptors_4, descriptors_5, descriptors_stream;
    extractor.compute( img_1, keypoints_1, descriptors_1 );
    extractor.compute( img_2, keypoints_2, descriptors_2 );
    extractor.compute( img_3, keypoints_3, descriptors_3 );
    extractor.compute( img_4, keypoints_4, descriptors_4 );
    extractor.compute( img_5, keypoints_5, descriptors_5 );
    extractor.compute( flippedImage, keypoints_stream, descriptors_stream );

    // //-- Step 3: Matching descriptor vectors using FLANN matcher 
    FlannBasedMatcher matcher;
    //matching process pic001.jpg
    std::vector< DMatch > matches;
    matcher.match( descriptors_1, descriptors_stream, matches );

    // //-- Quick calculation of max and min distances between keypoints_1
    double max_dist = 0; double min_dist = 100;
    for( int i = 0; i < descriptors_1.rows; i++ ){ 
      double dist = matches[i].distance;
      if( dist < min_dist ) min_dist = dist;
      if( dist > max_dist ) max_dist = dist;
    }
    printf("-- Max dist : %f \n", max_dist );
    printf("-- Min dist : %f \n", min_dist );

    // //-- Get good matches pic001.jpg
    std::vector< DMatch > good_matches;
    for( int i = 0; i < descriptors_1.rows; i++ ){ 
      if( matches[i].distance < 3*min_dist ){ 
        good_matches.push_back( matches[i]);
      }
    }
    
    Mat img_matches;
    drawMatches( img_1, keypoints_1, flippedImage, keypoints_stream,
               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );


    // //-- Localize pic001.jpg
    std::vector<Point2f> obj1;
    std::vector<Point2f> stream;
    for( int i = 0; i < good_matches.size(); i++ ){
        //-- Get the keypoints from the good matches
        obj1.push_back( keypoints_1[ good_matches[i].queryIdx ].pt );
        stream.push_back( keypoints_stream[ good_matches[i].trainIdx ].pt );
    }
    Mat H = findHomography( obj1, stream, CV_RANSAC );

    // //-- Get the corners from the pic001.jpg
    std::vector<Point2f> obj1_corners(4);
    obj1_corners[0] = cvPoint(0,0); 
    obj1_corners[1] = cvPoint( img_1.cols, 0 );
    obj1_corners[2] = cvPoint( img_1.cols, img_1.rows ); 
    obj1_corners[3] = cvPoint( 0, img_1.rows );
    std::vector<Point2f> stream_corners(4);

    perspectiveTransform( obj1_corners, stream_corners, H);

    //Draw pic001.jpg border line in camera stream
    line( img_matches, stream_corners[0] + Point2f( img_1.cols, 0), stream_corners[1] + Point2f( img_1.cols, 0), Scalar(0, 255, 0), 4 );
    line( img_matches, stream_corners[1] + Point2f( img_1.cols, 0), stream_corners[2] + Point2f( img_1.cols, 0), Scalar( 0, 255, 0), 4 );
    line( img_matches, stream_corners[2] + Point2f( img_1.cols, 0), stream_corners[3] + Point2f( img_1.cols, 0), Scalar( 0, 255, 0), 4 );
    line( img_matches, stream_corners[3] + Point2f( img_1.cols, 0), stream_corners[0] + Point2f( img_1.cols, 0), Scalar( 0, 255, 0), 4 );

    //output result
    return img_matches;
  }