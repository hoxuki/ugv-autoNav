#include <ros/ros.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#include <iostream>
#include <sstream>

// using namespace std;
// using namespace cv;
// using namespace std_msgs;
// using namespace cv_bridge;
// using namespace image_transport;
// using namespace sensor_msgs;
// using namespace ros;

void detectOpenings(cv::Mat frame) {
    cv::Mat frameGray, cdstP;
    cvtColor(frame, frameGray, cv::COLOR_BGR2GRAY);
    
    GaussianBlur(frameGray,frameGray, cv::Size(3,3), 0);    
    cv::Canny(frameGray,frameGray, 100, 100*3, 3);
    threshold(frameGray,frameGray, 200, 255,CV_THRESH_BINARY); 

    cvtColor(frameGray, cdstP, cv::COLOR_GRAY2BGR);
    
    //finding all contours in the image
    std::vector<cv::Vec4i> linesP; // will hold the results of the detection
    HoughLinesP(frameGray, linesP, 1, CV_PI/180, 50, 50, 10 ); // runs the actual detection
    // Draw the lines
    for(size_t i = 0; i < linesP.size(); i++ ) {
        cv::Vec4i l = linesP[i];
        line(cdstP, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, cv::LINE_AA);
    }

    // Vector for storing contour
    std::vector< std::vector <cv::Point> > contours; 
    std::vector< cv::Vec4i > hierarchy;
    
    // findContours(cdstP, contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
    // // iterate through each contour.
    // for (int i = 0; i< contours.size(); i=hierarchy[i][0]) {
    //     cv::Rect rect= boundingRect(contours[i]);
    //     //Check if there is a child contour
    //     if(hierarchy[i][2]<0) { 
    //         rectangle(cdstP,cv::Point(rect.x-10,rect.y-10), cv::Point(rect.x+rect.width+10,rect.y+rect.height+10), cv::Scalar(0,0,255),2,8,0); //Opened contour
    //     } else {
    //         rectangle(cdstP,cv::Point(rect.x-10,rect.y-10), cv::Point(rect.x+rect.width+10,rect.y+rect.height+10), cv::Scalar(0,255,0),2,8,0); //closed contour
    //     }
    // }


    imshow("camCapture (frameGray)", cdstP);
    cv::waitKey(1);
}


int main(int argc, char **argv) {
    // init with a name
    ros::init(argc, argv, "detect_openings");
    // create a node
    ros::NodeHandle nh;

    cv::VideoCapture camcapture(0); //cv::CAP_V4L2
    if (!camcapture.isOpened()) {
        ROS_ERROR_STREAM("Failed to open webcam!");
        ros::shutdown();
    } else {
        ROS_INFO_STREAM("Successfully opened webcam");
    }

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_frame = it.advertise("camCapture", 10);

    cv::Mat frame;
    sensor_msgs::ImagePtr msgs;
    ros::Rate loop_rate(1);

    while (nh.ok()) {
        camcapture.read(frame);
        if (frame.empty()) {
            ROS_ERROR_STREAM("Failed to capture image!");
            ros::shutdown;
        } else {
            ROS_INFO_STREAM("Displaying image feed");
        }

        msgs = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        pub_frame.publish(msgs);
        detectOpenings(frame);

        if (cv::waitKey(10) == 27) {
            ROS_INFO_STREAM("Stopping image feed");
            break;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    camcapture.release();
    return 0;
}