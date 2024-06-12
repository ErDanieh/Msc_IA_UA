import cv2
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

    # Utilizar la posición de la muñeca como referencia para mejorar la estabilidad
    if pinky_tip.y < wrist.y and all(pinky_tip.y < lm.y for lm in [index_finger_tip, thumb_tip]):  # El dedo meñique es el único levantado
        return "MOVE_LEFT"
    elif thumb_tip.y < wrist.y and all(thumb_tip.y < lm.y for lm in [index_finger_tip, pinky_tip]):  # El dedo gordo es el único levantado
        return "MOVE_RIGHT"
    elif all(lm.y < wrist.y for lm in [thumb_tip, index_finger_tip, pinky_tip]):  # Todos los dedos relevantes están levantados
        return "MOVE_FORWARD"
    elif all(lm.y > wrist.y for lm in [thumb_tip, index_finger_tip, pinky_tip]):  # Todos los dedos relevantes están bajados
        return "STOP"
    else:
        return "UNKNOWN"

cap = cv2.VideoCapture(0)

while cap.isOpened():
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

    cv2.imshow('MediaPipe Hands', image)
    if cv2.waitKey(5) & 0xFF == 27:
        break

hands.close()
cap.release()
