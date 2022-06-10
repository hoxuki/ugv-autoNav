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

#include <librealsense2/rs.hpp> 

// using namespace std;
// using namespace cv;
// using namespace std_msgs;
// using namespace cv_bridge;
// using namespace image_transport;
// using namespace sensor_msgs;
// using namespace ros;

// rs2::pipeline p;

void callback(const sensor_msgs::ImageConstPtr& msg) {
    std_msgs::Header msg_header = msg->header;
    std::string frame_id = msg_header.frame_id.c_str();
    ROS_INFO_STREAM("New Image from " << frame_id);

    cv_bridge::CvImagePtr cv_ptr; 
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // p.start();

    while (true){
        // // Block program until frames arrive
        // rs2::frameset frames = p.wait_for_frames();

        // // Try to get a frame of a depth image
        // rs2::depth_frame depth = frames.get_depth_frame();

        // // Get the depth frame's dimensions
        // float width = depth.get_width();
        // // float height = depth.get_height();

        // // Query the distance from the camera to the object in the center of the image
        // float dist_to_center = depth.get_distance(width / 2, height / 2);

        // // Print the distance
        // std::cout << "The camera is facing an object " << dist_to_center << " meters away \r";
    }
    cv::imshow("image window", cv_ptr->image);
    cv::waitKey(1);
}


int main(int argc, char** argv) {
    // init with a name
    ros::init(argc, argv, "detect_openings");
    // create a node
    ros::NodeHandle nh;


    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub_image = it.subscribe("/camera/depth/image_rect_raw", 1, callback);
    image_transport::Publisher pub_image = it.advertise("/detect_openings/depth_output", 1);
    
    ros::spin();
    cv::destroyAllWindows();
    return 0;
}