from sympy import Ge
import rospy
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.msg  
import numpy as np
from PIL import Image
import cv2
from main.srv import GetFrameCoordinates 
import sys

 
cx = cy = 0
def set_client(cx, cy):
    rospy.wait_for_service('GetFrameCoordinates')
    try:
        get_intersection_pt = rospy.ServiceProxy('GetFrameCoordinates', GetFrameCoordinates)     
        resp1 = get_intersection_pt(cx, cy)
        return resp1.depthValue  
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def usage():
    return "%s [x y]"%sys.argv[0]


bridge = CvBridge()
def callback_rgb(ros_image_color):
    rgb_image = bridge.imgmsg_to_cv2(ros_image_color, "passthrough")
    bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
   
    cv2.drawMarker(bgr_image, (int(cx),int(cy)), color=(0,0,255), markerType=cv2.MARKER_CROSS, thickness=2)
    cv2.namedWindow("bgr frame", cv2.WINDOW_AUTOSIZE)
    cv2.imshow("bgr frame", bgr_image)
    cv2.waitKey(1)
    # TODO: get intersection and center 
    # call service with center and get depth


def main():
    rospy.init_node('pixel2depth',anonymous=True)
    rospy.Subscriber("/camera/color/image_raw", sensor_msgs.msg.Image, callback_rgb)
    rospy.spin()


if __name__ == '__main__':
    if len(sys.argv) == 3:
        cx = int(sys.argv[1])
        cy = int(sys.argv[2])
        set_client(cx, cy)
    else:
        print(usage())
        sys.exit(1)
    main()
    print("depth value at crosshair = %s" %set_client(cx, cy))
    