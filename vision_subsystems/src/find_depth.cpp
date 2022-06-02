#include <ros/ros.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

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


void callback(const sensor_msgs::ImageConstPtr& msg) {
    std_msgs::Header msg_header = msg->header;
    std::string frame_id = msg_header.frame_id.c_str();
    ROS_INFO_STREAM("New Image from " << frame_id);

    cv_bridge::CvImagePtr cv_ptr; 
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat frameGray, cdst, cdstP;
    cvtColor(cv_ptr->image, frameGray, cv::COLOR_BGR2GRAY);
      

    cv::imshow("image window", frameGray);
    cv::waitKey(1);
}


int main(int argc, char** argv) {
    // init with a name
    ros::init(argc, argv, "detect_openings");
    // create a node
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub_image = it.subscribe("/camera/color/image_raw", 1, callback);
    image_transport::Publisher pub_image = it.advertise("/detect_openings/output_video", 1);
    
    ros::spin();
    cv::destroyAllWindows();
    return 0;
}



