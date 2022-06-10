#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.msg  
import numpy as np
from PIL import Image
import cv2  
from main.srv import GetFrameCoordinates 


i = cx = cy = depth_image= 0
def get_client(req):
    # print("Received rgb frame coordinates [%s + %s]"%(req.fx, req.fy))
    global i, cx, cy
    cx = req.fx + (848-640)/2
    cy = req.fy 
    depth_array = np.array(depth_image, dtype=np.float32)
    im = Image.fromarray(depth_array)
    im = im.convert("L")
    idx = str(i).zfill(4)
    # im.save(root+"/depth/frame{index}.png".format(index = idx))
    # im.save("test.png")   
    # print(depth_array[200:250, 400:450])
    i += 1
    print("depth_idx: ", i)
    print(depth_array[0][0])
    # return int(depth_array[cx,cy])

    # offset
    # get depth image
    # get the depth value and return
 


bridge = CvBridge()
def callback_depth(ros_image): # For debugging only
    global i, depth_image
    depth_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")
    # import pdb;pdb.set_trace()                #breakpoint
    cv2.drawMarker(depth_image, (int(cx),int(cy)), color=(0,255,255), markerType=cv2.MARKER_CROSS, thickness=4)
    cv2.namedWindow("depth", cv2.WINDOW_AUTOSIZE)
    cv2.imshow("depth", depth_image)
    cv2.waitKey(1)


def main():
    rospy.init_node('pixel2depth',anonymous=True)
    s = rospy.Service('GetFrameCoordinates', GetFrameCoordinates, get_client)
    print("Ready to receive frame coordinates.")
    rospy.Subscriber("/camera/depth/image_rect_raw", sensor_msgs.msg.Image ,callback_depth)
    rospy.spin()


if __name__ == '__main__':
    main()






# #!/usr/bin/env python
# i =0
# class Server:
#     def __init__(self):
#         self.orientation = None
#         self.velocity = None

#     def callback_depth(self, msg):
#         global i
#         bridge = CvBridge()
#         depth_image = bridge.imgmsg_to_cv2(msg, "passthrough")
#         # np.set_printoptions(threshold=np.inf)
#         depth_array = np.array(depth_image, dtype=np.float32)
#         im = Image.fromarray(depth_array)
#         im = im.convert("L")
#         idx = str(i).zfill(4)
#         # im.save(root+"/depth/frame{index}.png".format(index = idx))
#         # im.save("test.png")
#         print(depth_array[200:250, 400:450])
#         # import pdb;pdb.set_trace()                #breakpoint
#         cv2.drawMarker(depth_image, (int(420),int(240)), color=(255,255,255), markerType=cv2.MARKER_CROSS, thickness=4)
#         cv2.namedWindow("depth", cv2.WINDOW_AUTOSIZE)
#         cv2.imshow("depth", depth_image)
#         cv2.waitKey(1)
#         i += 1
#         print("depth_idx: ", i)

#     def callback_rgb(self, msg):
#         bridge = CvBridge()
#         rgb_image = bridge.imgmsg_to_cv2(msg, "passthrough")
#         cv2.drawMarker(rgb_image, (int(320),int(240)), color=(0,0,255), markerType=cv2.MARKER_CROSS, thickness=2)
#         cv2.namedWindow("rgb", cv2.WINDOW_AUTOSIZE)
#         cv2.imshow("rgb", rgb_image)
#         cv2.waitKey(1)
    

# if __name__ == '__main__':
#     rospy.init_node('listener')

#     server = Server()

#     rospy.Subscriber('/camera/depth/image_rect_raw', sensor_msgs.msg.Image , server.callback_depth)
#     rospy.Subscriber('/camera/color/image_raw', sensor_msgs.msg.Image, server.callback_rgb)

#     rospy.spin()