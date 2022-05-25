#!/usr/bin/env python
from matplotlib.pyplot import contour, gray
import rospy
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from scaledyolov4.scaled_yolov4 import ScaledYOLOV4
from cv_bridge import CvBridge, CvBridgeError
import argparse
import numpy
import math
import pkg_resources


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
            grayscale = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            canny = cv2.Canny(grayscale, 50, 200, None, 3)
            threshold = cv2.threshold(canny, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
            contour = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7,7))
            closed = cv2.morphologyEx(canny, cv2.MORPH_CLOSE, kernel)

            # for cnt in contour:
            #     approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
            #     cv2.drawContours(grayscale, [approx], 0, (0), 5)
            #     x = approx.ravel()[0]
            #     y = approx.ravel()[1]
            #     if len(approx) == 4:
            #         cv2.putText(grayscale, "Rectangle", (x, y), cv2.FONT_HERSHEY_COMPLEX, 1, (0))

            cdst = cv2.cvtColor(canny, cv2.COLOR_GRAY2BGR)
            # lines = cv2.HoughLines(canny, 1, numpy.pi / 180, 150, None, 0, 0)
            lines = cv2.HoughLinesP(canny, 1, numpy.pi / 180, 50, None, 50, 10)
            
            if lines is not None:
                for i in range(0, len(lines)):
                #     rho = lines[i][0][0]
                #     theta = lines[i][0][1]
                #     a = math.cos(theta)
                #     b = math.sin(theta)
                #     x0 = a * rho
                #     y0 = b * rho
                #     pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
                #     pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
                #     cv2.line(cdst, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)
            
                    l = lines[i][0]
                    cv2.line(cdst, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)
                        
            cv2.imshow("Source", cdst)

            # for cnt in contours:
            #     if cv2.contourArea(cnt)>10:
            #         print(1)


            # threshold = cv2.threshold(canny, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
            # contour = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # # contour = cv2.findContours(4)
            # cv2.drawContours(frame, [contour], 0, (0,255,0), 3)
            # # for cnt in contour:
            # #     approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
            # #     cv2.drawContours(frame, [approx], 0, (0), 5)
            # #     x = approx.ravel()[0]
            #     y = approx.ravel()[1]
            #     if len(approx) == 4:
            #         cv2.putText(frame, "Rectangle", (x, y), cv2.FONT_HERSHEY_COMPLEX, 1, (0))




            ################################################################

            # font = cv2.FONT_HERSHEY_SIMPLEX
            # img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # # _, threshold = cv2.threshold(img, 240, 255, cv2.THRESH_BINARY)
            # # _, contours, _ = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # _, threshold = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
            # _, contours = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # for cnt in contours:
            #     approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
            #     cv2.drawContours(img, [approx], 0, (0), 5)
            #     x = approx.ravel()[0]
            #     y = approx.ravel()[1]

            #     if len(approx) == 4:
            #         cv2.putText(img, "Rectangle", (x, y), font, 1, (0))




            # Display the resulting frame
            # cv2.imshow('Frame', grayscale)
          

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