#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
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


def main(args):
    rospy.init_node("testing_object_detection_node", anonymous=True)
    publish = rospy.Publisher('camCapture', String, queue_size=10)
    rate = rospy.Rate(10)
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        rospy.logerr("Failed to open webcam")
    else:
        rospy.loginfo("Successfully opened webcam")

    while not rospy.is_shutdown() and cap.isOpened():       
        ret, frame = cap.read()
        if ret == True:
            # Display the resulting frame
            display_image = cv2.putText(frame, str(od.get_detections_dict([frame])), (5,20), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1, cv2.LINE_4)
            display_image = cv2.putText(display_image, str(od.detect_get_box_in([frame], box_format='ltwh')), (5,60), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1, cv2.LINE_4)
            cv2.imshow('Frame', display_image)
        
            # Press Q on keyboard to  exit
            if cv2.waitKey(25) & 0xFF == ord('q'):
                break

        rate.sleep()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass



# ----------------------------------------------------------------
# def show_image(img):
#     cv2.imshow("Image Window", img)
#     cv2.waitKey(3)

# def image_callback(img_msg):
#     rospy.loginfo(img_msg.header)

# try:
#     cv_image = cv2.VideoCapture(0)
#     if not cv_image.isOpened():
#         print("Cannot open camera")
#         exit()
# except CvBridgeError as e:
#     rospy.logerr("CvBridge Error: {0}".format(e))
# show_image(cv_image)
