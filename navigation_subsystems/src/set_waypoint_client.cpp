#include "ros/ros.h"
#include "navigation_subsystems/SetWaypoint.h"
#include <cstdlib>


int main(int argc, char **argv) {
    // check args
    ros::init(argc, argv, "set_waypoint_client");
    if (argc != 4) {
        ROS_INFO("usage: input X, Y, Z coordinate");
        return 1;
    }

    // create a node
    ros::NodeHandle n;

    // node will be a client
    ros::ServiceClient client = n.serviceClient<navigation_subsystems::SetWaypoint>("set_waypoint");
    
    // create a service target
    navigation_subsystems::SetWaypoint srv;

    // add params
    srv.request.x = atoll(argv[1]);
    srv.request.y = atoll(argv[2]);
    srv.request.z = atoll(argv[3]);

    // call to service
    if (client.call(srv)) { 
        ROS_INFO("Sending coordinates: x=%ld, y=%ld, z=%ld", (long int)srv.request.x, (long int)srv.request.y, (long int)srv.request.z);
        ROS_ERROR("Failed to call service 'set_waypoint'");
    } else {
        return 1;
    }

    return 0;
}