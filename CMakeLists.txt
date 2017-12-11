cmake_minimum_required(VERSION 2.8.3)
project(demo_elec4010k)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
  sensor_msgs
)

find_package( OpenCV REQUIRED )
message(STATUS "OpenCV version = ${OpenCV_VERSION}")


#FOR EDWIN LAPTOP ONLY
# set(OpenCV_LIB_COMPONENTS 
  # opencv_calib3d 
  # opencv_coreopencv_dnn
  # opencv_features2d
  # opencv_flann
  # opencv_highgui
  # opencv_imgcodecs
  # opencv_imgproc
  # opencv_ml
  # opencv_objdetect
  # opencv_photo
  # opencv_shape
  # opencv_stitching
  # opencv_superres
  # opencv_video
  # opencv_videoio
  # opencv_videostab
  # opencv_viz
  # opencv_aruco
  # opencv_bgsegm
  # opencv_bioinspired
  # opencv_ccalib
  # opencv_datasets
  # opencv_dpm
  # opencv_face
  # opencv_freetype
  # opencv_fuzzy
  # opencv_img_hash
  # opencv_line_descriptor
  # opencv_optflow
  # opencv_phase_unwrapping
  # opencv_plot
  # opencv_reg
  # opencv_rgbd
  # opencv_saliency
  # opencv_stereo
  # opencv_structured_light
  # opencv_surface_matching
  # opencv_text
  # opencv_tracking
  # opencv_xfeatures2d
  # opencv_ximgproc
  # opencv_xobjdetect
  # opencv_xphoto)
# set(OpenCV_INCLUDE_DIRS 
  # /usr/include/opencv 
  # /usr/include
  # /usr/include/opencv2
  ## "/home/edwin/OpenCV/build" "/home/edwin/OpenCV/opencv/include" "/home/edwin/OpenCV/opencv/include/opencv" "/home/edwin/OpenCV/opencv/modules/core/include" "/home/edwin/OpenCV/opencv/modules/flann/include" "/home/edwin/OpenCV/opencv/modules/imgproc/include" "/home/edwin/OpenCV/opencv/modules/ml/include" "/home/edwin/OpenCV/opencv/modules/objdetect/include" "/home/edwin/OpenCV/opencv_contrib/modules/phase_unwrapping/include" "/home/edwin/OpenCV/opencv/modules/photo/include" "/home/edwin/OpenCV/opencv_contrib/modules/plot/include" "/home/edwin/OpenCV/opencv_contrib/modules/reg/include" "/home/edwin/OpenCV/opencv_contrib/modules/surface_matching/include" "/home/edwin/OpenCV/opencv/modules/video/include" "/home/edwin/OpenCV/opencv/modules/viz/include" "/home/edwin/OpenCV/opencv_contrib/modules/xphoto/include" "/home/edwin/OpenCV/opencv/modules/dnn/include" "/home/edwin/OpenCV/opencv_contrib/modules/freetype/include" "/home/edwin/OpenCV/opencv_contrib/modules/fuzzy/include" "/home/edwin/OpenCV/opencv_contrib/modules/img_hash/include" "/home/edwin/OpenCV/opencv/modules/imgcodecs/include" "/home/edwin/OpenCV/opencv/modules/shape/include" "/home/edwin/OpenCV/opencv/modules/videoio/include" "/home/edwin/OpenCV/opencv_contrib/modules/xobjdetect/include" "/home/edwin/OpenCV/opencv/modules/highgui/include" "/home/edwin/OpenCV/opencv/modules/superres/include" "/home/edwin/OpenCV/opencv/modules/ts/include" "/home/edwin/OpenCV/opencv_contrib/modules/bioinspired/include" "/home/edwin/OpenCV/opencv_contrib/modules/dpm/include" "/home/edwin/OpenCV/opencv/modules/features2d/include" "/home/edwin/OpenCV/opencv_contrib/modules/line_descriptor/include" "/home/edwin/OpenCV/opencv_contrib/modules/saliency/include" "/home/edwin/OpenCV/opencv_contrib/modules/text/include" "/home/edwin/OpenCV/opencv/modules/calib3d/include" "/home/edwin/OpenCV/opencv_contrib/modules/ccalib/include" "/home/edwin/OpenCV/opencv_contrib/modules/datasets/include" "/home/edwin/OpenCV/opencv_contrib/modules/rgbd/include" "/home/edwin/OpenCV/opencv_contrib/modules/stereo/include" "/home/edwin/OpenCV/opencv_contrib/modules/structured_light/include" "/home/edwin/OpenCV/opencv_contrib/modules/tracking/include" "/home/edwin/OpenCV/opencv/modules/videostab/include" "/home/edwin/OpenCV/opencv_contrib/modules/xfeatures2d/include" "/home/edwin/OpenCV/opencv_contrib/modules/ximgproc/include" "/home/edwin/OpenCV/opencv_contrib/modules/aruco/include" "/home/edwin/OpenCV/opencv_contrib/modules/bgsegm/include" "/home/edwin/OpenCV/opencv_contrib/modules/face/include" "/home/edwin/OpenCV/opencv/modules/java/include" "/home/edwin/OpenCV/opencv_contrib/modules/optflow/include" "/home/edwin/OpenCV/opencv/modules/stitching/include")
  # /home/edwin/OpenCV/build
  # /home/edwin/OpenCV/opencv/include
  # /home/edwin/OpenCV/opencv/include/opencv
  # /home/edwin/OpenCV/opencv/modules/core/include
  # /home/edwin/OpenCV/opencv/modules/flann/include
  # /home/edwin/OpenCV/opencv/modules/imgproc/include
  # /home/edwin/OpenCV/opencv/modules/ml/include
  # /home/edwin/OpenCV/opencv/modules/objdetect/include
  # /home/edwin/OpenCV/opencv_contrib/modules/phase_unwrapping/include
  # /home/edwin/OpenCV/opencv/modules/photo/include
  # /home/edwin/OpenCV/opencv_contrib/modules/plot/include
  # /home/edwin/OpenCV/opencv_contrib/modules/reg/include
  # /home/edwin/OpenCV/opencv_contrib/modules/surface_matching/include
  # /home/edwin/OpenCV/opencv/modules/video/include
  # /home/edwin/OpenCV/opencv/modules/viz/include
  # /home/edwin/OpenCV/opencv_contrib/modules/xphoto/include
  # /home/edwin/OpenCV/opencv/modules/dnn/include
  # /home/edwin/OpenCV/opencv_contrib/modules/freetype/include
  # /home/edwin/OpenCV/opencv_contrib/modules/fuzzy/include
  # /home/edwin/OpenCV/opencv_contrib/modules/img_hash/include
  # /home/edwin/OpenCV/opencv/modules/imgcodecs/include
  # /home/edwin/OpenCV/opencv/modules/shape/include
  # /home/edwin/OpenCV/opencv/modules/videoio/include
  # /home/edwin/OpenCV/opencv_contrib/modules/xobjdetect/include
  # /home/edwin/OpenCV/opencv/modules/highgui/include
  # /home/edwin/OpenCV/opencv/modules/superres/include
  # /home/edwin/OpenCV/opencv/modules/ts/include
  # /home/edwin/OpenCV/opencv_contrib/modules/bioinspired/include
  # /home/edwin/OpenCV/opencv_contrib/modules/dpm/include
  # /home/edwin/OpenCV/opencv/modules/features2d/include
  # /home/edwin/OpenCV/opencv_contrib/modules/line_descriptor/include
  # /home/edwin/OpenCV/opencv_contrib/modules/saliency/include
  # /home/edwin/OpenCV/opencv_contrib/modules/text/include
  # /home/edwin/OpenCV/opencv/modules/calib3d/include
  # /home/edwin/OpenCV/opencv_contrib/modules/ccalib/include
  # /home/edwin/OpenCV/opencv_contrib/modules/datasets/include
  # /home/edwin/OpenCV/opencv_contrib/modules/rgbd/include
  # /home/edwin/OpenCV/opencv_contrib/modules/stereo/include
  # /home/edwin/OpenCV/opencv_contrib/modules/structured_light/include
  # /home/edwin/OpenCV/opencv_contrib/modules/tracking/include
  # /home/edwin/OpenCV/opencv/modules/videostab/include
  # /home/edwin/OpenCV/opencv_contrib/modules/xfeatures2d/include
  # /home/edwin/OpenCV/opencv_contrib/modules/ximgproc/include
  # /home/edwin/OpenCV/opencv_contrib/modules/aruco/include
  # /home/edwin/OpenCV/opencv_contrib/modules/bgsegm/include
  # /home/edwin/OpenCV/opencv_contrib/modules/face/include
  # /home/edwin/OpenCV/opencv/modules/java/include
  # /home/edwin/OpenCV/opencv_contrib/modules/optflow/include
  # /home/edwin/OpenCV/opencv/modules/stitching/include)
# set(OpenCV_LIB_DIR /home/edwin/OpenCV/build/lib)


message(STATUS "OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBS = ${OpenCV_LIBS}")
message(STATUS "OpenCV_LIB_COMPONENTS = ${OpenCV_LIB_COMPONENTS}")
message(STATUS "OpenCV_LIB_DIR = $(OpenCV_LIB_DIR)")

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES demo_elec4010k
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/demo_elec4010k.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(rosopencv_interface  src/image_converter.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(
    rosopencv_interface
##    opencv_nonfree 
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_demo_elec4010k.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
