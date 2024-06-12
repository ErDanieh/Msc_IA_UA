#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import mediapipe as mp

mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5,
    model_complexity=1
)

mp_drawing = mp.solutions.drawing_utils

def classify_gesture(hand_landmarks):
    thumb_tip = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP]
    index_finger_tip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
    pinky_tip = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP]
    wrist = hand_landmarks.landmark[mp_hands.HandLandmark.WRIST]

    if pinky_tip.y < wrist.y and all(pinky_tip.y < lm.y for lm in [index_finger_tip, thumb_tip]):
        return "MOVE_LEFT"
    elif thumb_tip.y < wrist.y and all(thumb_tip.y < lm.y for lm in [index_finger_tip, pinky_tip]):
        return "MOVE_RIGHT"
    elif all(lm.y < wrist.y for lm in [thumb_tip, index_finger_tip, pinky_tip]):
        return "MOVE_FORWARD"
    elif all(lm.y > wrist.y for lm in [thumb_tip, index_finger_tip, pinky_tip]):
        return "STOP"
    else:
        return "UNKNOWN"

def video_publisher():
    rospy.init_node('video_publisher', anonymous=True)

    pub = rospy.Publisher('/operator/image', Image, queue_size=10)

    cap = cv2.VideoCapture(0)

    bridge = CvBridge()

    rate = rospy.Rate(30)

    while not rospy.is_shutdown() and cap.isOpened():
        success, image = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
            continue

        image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
        
        results = hands.process(image)
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                gesture = classify_gesture(hand_landmarks)
                print("Detected gesture:", gesture)
        
        ros_image = bridge.cv2_to_imgmsg(image, "bgr8")
        
        pub.publish(ros_image)
        cv2.imshow('MediaPipe Hands', image)
        if cv2.waitKey(5) & 0xFF == 27:
            break
        
        rate.sleep()

    hands.close()
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        video_publisher()
    except rospy.ROSInterruptException:
        pass
