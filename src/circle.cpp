#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
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
    // declare variables to be used later:
    int postX;
    int postY;

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

        Mat src_gray;

        /// Convert it to gray
        cvtColor( cv_ptr->image, src_gray, CV_BGR2GRAY );

        /// Reduce the noise so we avoid false circle detection
        GaussianBlur( src_gray, src_gray, Size(9, 9), 2, 2 );
        Canny(src_gray,src_gray,20,40);
        vector<Vec3f> circles;

        /// Apply the Hough Transform to find the circles
        HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, 80, 10, 5, 20 );

        /// Draw the circles detected
        for( size_t i = 0; i < circles.size(); i++ )
        {
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            // circle center
            circle( cv_ptr->image, center, 2, Scalar(0,255,0), -1, 8, 0 );
            // circle outline
            circle( cv_ptr->image, center, radius, Scalar(0,0,255), 1, 8, 0 );

            // show message on picture
            //ostringstream str1;
            //str1 << "Circle found at: " << center.x << " x " << center.y;
            //putText(cv_ptr->image, str1.str(), Point(30,30), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(255,0,0),1,CV_AA);

            // show the radius of circle:
            //ostringstream str2;
            //str2 << "Radius of Circle is: " << radius;
            //putText(cv_ptr->image, str2.str(), Point(30,60), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(255,0,0),1,CV_AA);

            // calculate the distance from center
            //int dist = abs(sqrt((center.x - 320)*(center.x - 320) + (center.y - 240)*(center.y - 240)));
            //ostringstream str3;
            //str3 << "Distance from center point is: " << dist << " pixel";
            //putText(cv_ptr->image, str3.str(), Point(30, 90), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(255,0,0), 1, CV_AA);

            // calculate the distance from camera
            //int obj_dist = (547.2*5)/(16*radius);
            //ostringstream str4;
            //str4 << "Distance from camera to circle is: " << obj_dist << " inches";
            //putText(cv_ptr->image, str4.str(), Point(30,120), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(255,0,0), 1, CV_AA);

            // send as rostopic for robot subcribe
            //std_msgs::Int32MultiArray array;
            //array.data.clear();
            //array.data.push_back(center.x);
            //array.data.push_back(center.y);
            //pos_pub_.publish(array);
        }

        // Update GUI Window
        // imshow(OPENCV_WINDOW, cv_ptr->image);
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
