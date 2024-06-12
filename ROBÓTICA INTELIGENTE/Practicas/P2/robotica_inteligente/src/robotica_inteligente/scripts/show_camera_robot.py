#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

def image_callback(msg):
    try:
        cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("Camera View", cv_image)
        cv2.waitKey(1)
    except CvBridgeError as e:
        print(e)

def main():
    rospy.init_node('image_viewer', anonymous=True)
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
