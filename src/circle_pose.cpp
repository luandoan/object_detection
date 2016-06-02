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
#include <detect/pose.h>



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
    //int posX = 1000;
    //int posY = 1000;
    //int posZ = 1000;

    ImageConverter()
        : it_(nh_)
    {
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
            &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);
        pos_pub_ = nh_.advertise<detect::pose>("/circle_pos",2);
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

        int idx, minC = 800;
        int distC[circles.size()];

        //cout << "Number of circles detected is: " << circles.size() << endl;

        /// Draw the circles detected
        for( int i = 0; i < circles.size(); i++ )
        {
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));

            distC[i] = sqrt((center.x - 400)*(center.x - 400) + (center.y - 240)*(center.y - 240));

            if (distC[i] < minC)
              {
                minC = distC[i];
                idx = i;
              }
        }

        cout << minC;

            Point center(cvRound(circles[idx][0]), cvRound(circles[idx][1]));
            int radius = cvRound(circles[idx][2]);

            // circle center
            circle( cv_ptr->image, center, 2, Scalar(0,255,0), -1, 8, 0 );
            // circle outline
            circle( cv_ptr->image, center, radius, Scalar(0,0,255), 1, 8, 0 );

            int posX = center.x - 400;
            int posY = center.y - 240;

            // show message on picture
            ostringstream str1;
            str1 << "Circle found at: " << center.x << " x " << center.y;
            putText(cv_ptr->image, str1.str(), Point(20,30), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(255,0,0),0.6,CV_AA);

            // show the radius of circle:
            ostringstream str2;
            str2 << "Radius of Circle on camera frame is: " << radius << " pixel";
            putText(cv_ptr->image, str2.str(), Point(20,50), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(255,0,0),0.6,CV_AA);

            // calculate the distance from center to interest points (400x240)
            int distX = (posX*330)/677.574;
              ostringstream str3;
              if (posX > 0)
                    str3 << "Circle is on right of IP, dist in X-dir is: " << distX << " mm";
                else if (posX < 0)
                    str3 << "Circle is on left of IP, dist in X-dir is: " << distX << " mm";
                else
                    str3 << "Circle is right on IP of X-direction.";
                putText(cv_ptr->image, str3.str(), Point(20, 70), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(255,0,0), 0.6, CV_AA);

            // calculate the distance from center to interest points (400x240)
            int distY = (posY*330)/677.41;
              ostringstream str4;
              if (posX > 0)
                  str4 << "Circle is higher than IP, dist in Y-dir is: " << distY << " mm";
              else if (posX < 0)
                  str4 << "Circle is lower than IP, dist in Y-dir is: " << distY << " mm";
              else
                  str4 << "Circle is right on IP of Y-direction.";
              putText(cv_ptr->image, str4.str(), Point(20, 90), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(255,0,0), 0., CV_AA);

            // calculate the distance from camera
            int distZ = (677.5*4.4)/radius;
            ostringstream str5;
            str5 << "Distance from camera to circle is: " << distZ << " mm";
            putText(cv_ptr->image, str5.str(), Point(20,110), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(255,0,0), 0.6, CV_AA);

        // print out position of circle of interest
        std::cout << "X:pos: " << distX << std::endl;
       	std::cout << "Y:pos: " << distY << std::endl;
        std::cout << "Z:pos: " << distZ << std::endl;


        detect::pose dist_Err;
            dist_Err.translate_x = distX;
            dist_Err.translate_y = distY;
            dist_Err.translate_z = distZ;

        waitKey(3);

        // Output modified video stream
        image_pub_.publish(cv_ptr->toImageMsg());
        pos_pub_.publish(dist_Err);

    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::spin();
    return 0;
}
