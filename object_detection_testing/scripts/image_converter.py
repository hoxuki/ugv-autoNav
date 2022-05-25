import rospy
import sys
import cv2
from sensor_msgs.msg import Image
from scaledyolov4.scaled_yolov4 import ScaledYOLOV4
from cv_bridge import CvBridge, CvBridgeError
import argparse
import pkg_resources


parser = argparse.ArgumentParser()
parser.add_argument('--vid_list', help='Path of text file containing all video paths, 1 in each row', type=str, required=False)
parser.add_argument('--infer_fps', help='FPS for inference (use higher fps for deepsort to work better)', type=int, default=4)
parser.add_argument('--gpu_dev', help='Gpu device number to use. Default: 0', type=int, default=0)
parser.add_argument('--output_dir', help='Path of output directory', default='output')
parser.add_argument('--save_chips', help='Whether to save cropped chips', action='store_true')
parser.add_argument('--seconds', help='Number of seconds between each chip save', type=int, default=1)
parser.add_argument('--record_tracks', help='Whether to save inference video', action='store_true')
args = parser.parse_args()

od = ScaledYOLOV4(
        bgr=True,
        gpu_device=args.gpu_dev, # gpu_device='cpu',
        model_image_size=608,
        # model_image_size=896,   # to detect mini hoomans
        # model_image_size=1280,
        # model_image_size=1536,
        max_batch_size=1,
        half=True,
        same_size=True,
        weights=pkg_resources.resource_filename('scaledyolov4', 'weights/yolov4-p6_-state.pt'),
        cfg=pkg_resources.resource_filename('scaledyolov4', 'configs/yolov4-p6.yaml'),
)

bridge = CvBridge()


def image_callback(img_msg):
    rospy.loginfo(img_msg.header)
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
        display_image = cv2.putText(cv_image, str(od.get_detections_dict([cv_image])), (5,20), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1, cv2.LINE_4)
        display_image = cv2.putText(display_image, str(od.detect_get_box_in([cv_image], box_format='ltwh')), (5,60), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1, cv2.LINE_4)
        cv2.imshow("Image Window", display_image)
        cv2.waitKey(1)
    except CvBridgeError:
        rospy.logerr("CvBridge Error: {0}".format())
  
def main(args):
    rospy.init_node('image_converter_node', anonymous=True)
    rospy.Subscriber("/realsense/color/image_raw", Image, image_callback)
    rospy.Rate(10)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.logerr("Shutting down")
    cv2.destroyAllWindows


if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass




# --------------------------    Reference  -------------------------------------
# class image_converter:
#     def __init__(self):
#         self.publish = rospy.Publisher("cvImg", Image)
#         self.bridge = CvBridge
#         self.subscibe = rospy.Subscriber("rosImgMsg", Image, self.callback)

#     def callback(self, data):
#         try:
#             cv_image = self.bridge.imgmsg_to_cv2(data, "bdr8")
#         except:
#             print(e)
#         (rows, cols, channels) = cv_image.shape
#         if cols>60 and rows>60:
#             cv2.circle(cv_image, (50,50), 10, 255)
#         cv2.imshow("Image Window", cv_image)
#         cv2.waitKey(3)
        
#         try:
#             self.image_publish.publish(self.bridge.cv2_to_imgmsg(cv_image, "bdr8"))
#         except CvBridgeError as e:
#             print(e)

# def main(args):
#     ic = image_converter()
#     rospy.init_node('image_converter_node', anonymous=True)
#     rospy.Subscriber("/realsense/color/image_raw", Image, image_callback)
#     publish = rospy.Publisher("cvImg", str, queue_size=100)
#     rate = rospy.Rate(0.1)
#     while not rospy.is_shutdown():
#         rospy.loginfo("Success %s" %rospy.get_time())
#         publish.publish()
#         rate.sleep()
#     try:
#         rospy.spin()
#     except KeyboardInterrupt:
#         rospy.loginfo("Shutting down")
#     cv2.destroyAllWindows


# if __name__ == '__main__':
#     try:
#         main(sys.argv)
#     except rospy.ROSInterruptException:
#         pass