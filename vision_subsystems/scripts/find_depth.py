import rospy
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.msg  
import numpy as np
from PIL import Image
import cv2
i = 0
root='~/yu'
def convert_depth_image(ros_image):
    bridge = CvBridge()
    global i
    depth_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")
    # np.set_printoptions(threshold=np.inf)
    depth_array = np.array(depth_image, dtype=np.float32)
    im = Image.fromarray(depth_array)
    im = im.convert("L")
    idx = str(i).zfill(4)
    # im.save(root+"/depth/frame{index}.png".format(index = idx))
    # im.save("test.png")
    print(depth_array[200:250, 400:450])
    # import pdb;pdb.set_trace()
    cv2.imshow("live", depth_image)
    cv2.imshow("depth", rgb_image)
    cv2.waitKey(1)
    i += 1
    print("depth_idx: ", i)

def pixel2depth():
    rospy.init_node('pixel2depth',anonymous=True)
    rospy.Subscriber("/camera/depth/image_rect_raw", sensor_msgs.msg.Image,callback=convert_depth_image, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    pixel2depth()
