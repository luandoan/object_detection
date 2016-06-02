#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "std_msgs/String.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"

using namespace cv;
using namespace std;


class ImageConverter
{
    // declare all ros node handle:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher pos_pub_;

public:
    ImageConverter()
        : it_(nh_)
    {
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
            &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
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

        /// Global variables
        Mat src_gray;
        int thresh = 200;
        int max_thresh = 255;

        /// Function header
        void cornerHarris_demo();

        /** @function main */

            /// Load source image and convert it to gray
            //src = imread( "corner.jpg", 1 );
            cvtColor( cv_ptr->image, src_gray, CV_BGR2GRAY );

            /// Create a window and a trackbar
            namedWindow( "Source Window", CV_WINDOW_NORMAL);
            //createTrackbar( "Threshold: ", "Source Window", &thresh, max_thresh, cornerHarris_demo );
            imshow( "Source Window", cv_ptr->image );

            //cornerHarris_demo( 0, 0 );

        //    waitKey(0);
        //  /  return(0);


          /** @function cornerHarris_demo */
          void cornerHarris_demo( );
          //  {
              Mat dst, dst_norm, dst_norm_scaled;
              dst = Mat::zeros( src_gray.size(), CV_32FC1 );

              /// Detector parameters
              int blockSize = 2;
              int apertureSize = 3;
              double k = 0.04;

              /// Detecting corners
              cornerHarris( src_gray, dst, blockSize, apertureSize, k, BORDER_DEFAULT );

              /// Normalizing
              normalize( dst, dst_norm, 145, 255, NORM_MINMAX, CV_32FC1, Mat() );
              convertScaleAbs( dst_norm, dst_norm_scaled );

              /// Drawing a circle around corners
              for( int j = 0; j < dst_norm.rows ; j++ )
                {
                  for( int i = 0; i < dst_norm.cols; i++ )
                    {
                      if( (int) dst_norm.at<float>(j,i) > thresh )
                        {
                          circle( dst_norm_scaled, Point( i, j ), 5,  Scalar(0), 2, 8, 0 );
                        }
                    }
                }
              /// Showing the result
              namedWindow( "Corners Window", CV_WINDOW_NORMAL);
              imshow( "Corners Window", dst_norm_scaled );
        //    }

        // Update GUI Window
        waitKey(3);

        // Output modified video stream
        image_pub_.publish(cv_ptr->toImageMsg());
      }

};

  int main(int argc, char** argv)
    {
      ros::init(argc, argv, "image_converter");
      ImageConverter ic;
      ros::spin();
      return 0;
    }
