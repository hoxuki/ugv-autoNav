#include "ros/ros.h"
#include "navigation_subsystems/SetWaypoint.h"
// #include "geometry_msgs/Twist.h"
// #include "nav_msgs/Odometry.h"
// #include <tf/transform_broadcaster.h>
#include "move_base_msgs/MoveBaseAction.h"
#include "move_base_msgs/MoveBaseGoal.h"
#include "actionlib/client/simple_action_client.h"

double x, y, z;

/** ---------------------------------------------------------------- WIPs ----------------------------------------------------------------*/

// service fuction
bool getCoordinates(navigation_subsystems::SetWaypoint::Request  &req,
                    navigation_subsystems::SetWaypoint::Response &res) {
    x = req.x;
    y = req.y;
    z = req.z;
    ROS_INFO("Moving to coordinates: x=%ld, y=%ld, z=%ld", (long int)req.x, (long int)req.y, (long int)req.z);
    
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    
    // Set the action cient to spin the thread by default. Therefore no need ros.spin
    MoveBaseClient actionClient("move_base", true);
    while (!actionClient.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    double setpointGoal[3] = {x, y, z};
    goal.target_pose.pose.position.x = setpointGoal[0];
    goal.target_pose.pose.position.y = setpointGoal[1];
    goal.target_pose.pose.orientation.w = setpointGoal[2];

    ROS_INFO("Sending goal");
    actionClient.sendGoal(goal);
    actionClient.waitForResult();
    
    if (actionClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, reached goal!");
    } else {
        ROS_WARN("The base failed to move forward"); 
    }

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "set_waypoint_server");

    // create a node
    ros::NodeHandle n;

    // node will have a service
    ros::ServiceServer service = n.advertiseService("set_waypoint", getCoordinates);
    ROS_INFO("Ready to set waypoint");

    // main loop
    ros::spin();

    return 0;
}



/** ---------------------------------------------------------------- working stuffs ----------------------------------------------------------------*/
// typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
// int main(int argc, char **argv) {
//     ros::init(argc, argv, "set_waypoint_server");
    
//     // Set the action cient to spin the thread by default. Therefore no need ros.spin
//     MoveBaseClient actionClient("move_base", true);
//     while (!actionClient.waitForServer(ros::Duration(5.0))) {
//         ROS_INFO("Waiting for the move_base action server to come up");
//     }

//     move_base_msgs::MoveBaseGoal goal;
//     goal.target_pose.header.frame_id = "map";
//     goal.target_pose.header.stamp = ros::Time::now();

//     double setpointGoal[3] = {-7, -8, 0.1};
//     goal.target_pose.pose.position.x = setpointGoal[0];
//     goal.target_pose.pose.position.y = setpointGoal[1];
//     goal.target_pose.pose.orientation.w = setpointGoal[2];

//     ROS_INFO("Sending goal");
//     actionClient.sendGoal(goal);

//     actionClient.waitForResult();
        
//     /**
//      * For settingg multiple goals
//      */
//     // float goals[2][3] = {{-8, 8, 1.0}, {7, 7, 1.57}};
//     //     for(int i=0; i<2; i++) {
//     //         goal.target_pose.pose.position.x = goals[i][0];
//     //         goal.target_pose.pose.position.y = goals[i][1];
//     //         goal.target_pose.pose.orientation.w = goals[i][2];
        
//     //         ROS_INFO("Sending goal");
//     //         actionClient.sendGoal(goal);

//     //         actionClient.waitForResult();

//     //         ros::Duration(5.0).sleep();
//     //     }

//     if (actionClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
//         ROS_INFO("Hooray, reached goal!");
//     } else {
//         ROS_WARN("The base failed to move forward"); 
//     }
//     return 0;
// }






/**
 * ! reference / depreciated
 */

// void moveToCoordinate(double posX, double posY, double posZ) {
//     goal_set = true;
    
//     current_time = ros::Time::now();
//     // Create goal
// }

// bool successfullyMovedToCoordinate() {
//     bool result = false, success = false;
//     success = move_base.wait_for_result(rospy.Duration(60)) 
//     if success
//     return result;
// }



// ---------------------------------------------------------------------------------------------------------/
// while (ros::ok()) {
//         moveToCoordinate(x, y, z);
        
//         double dt = (current_time - last_time).toSec();
//         double delta_x = (vx * cos(z) - vy * sin(z)) * dt;
//         double delta_y = (vx * sin(z) + vy * cos(z)) * dt;
//         double delta_th = vz * dt;
   
//         x += delta_x;
//         y += delta_y;
//         z += delta_th;

//         nav_msgs::Odometry odom;
//         geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(z);
//         odom.header.stamp = current_time;

//         odom.header.frame_id = "odom";

//         odom.pose.pose.position.x = x;
//         odom.pose.pose.position.x = x;
//         odom.pose.pose.position.x = x;
//         odom.pose.pose.orientation = odom_quat;

//         odom.child_frame_id = "base_link";
//         odom.twist.twist.linear.x = vx;
//         odom.twist.twist.linear.y = vy;
//         odom.twist.twist.angular.z = vz;

//         pub_odom.publish(odom);
//         ros::spinOnce();
//         loop_rate.sleep();
//     }
