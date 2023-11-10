import cv2
import mediapipe as mp
import socket
import json
import math

# UDP receiver ip & port
UDP_IP = "169.254.167.128"   # an test Arduino Shield
UDP_PORT = 5020  # test Arduino Receiver

UDP_IP_2 = "10.137.19.141"  # Computer 1 (Section 1)
UDP_PORT_2 = 20005  # Max receiver port

data = {
    "h": {
        "t1": 0,
        "t2": 0,
        "t3": 0,
        "t4": 0,
        "t5": 0,
        "t6": 0
    }
}

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

text = ""

def wristpos(rightWristX, rightWristY, rightShoulderX, rightShoulderY, leftShoulderX):
    div = abs(rightShoulderX - leftShoulderX)
    wristX = (rightShoulderX - rightWristX) / div
    if wristX > 1.5:
        wristX = 1.5
    if wristX < -1.5:
        wristX = -1.5
    wristX = "{:.2f}".format(wristX)
    wristX = float(wristX)

    wristY = (rightShoulderY - rightWristY) / div
    if wristY> 2:
        wristY = 2
    if wristY < -2:
        wristY = -2
    wristY = "{:.2f}".format(wristY)
    wristY = float(wristY)

    #Wrist X position varies from -1.5->1.5
    #Wrist Y position varies from -2 -> 2

    #Joint 4 should move opposite of Joint 2
    #Joint 3 should move opposite of Joint 1

    #Joint 1 should be -90 when X = -1.5, 90 when X = 1.5
    #y = 60X

    #Joint 2 should be 0 when Y = -2, -90 when Y = 2
    #y=−22.5x−45

    mag = math.sqrt(wristX**2+wristY**2)
    if mag > 2:
        mag = 2

    #Pincers should be in as possible at mag = 0 and out as possible at mag = 5

    theta1 = 60*wristX
    theta2 = -22.5*wristY-45
    theta3 = -theta1
    theta4 = -theta2
    theta5 = 12.5*mag
    theta6 = theta5
    #theta5 = linear fit of mag based on pincer angle range
    #theta6 = -theta5

    return theta1, theta2, theta3, theta4, theta5, theta6


sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
theta1 = 0
theta2 = -90
theta3 = 0
theta4 = 0

cap = cv2.VideoCapture(0)
with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
    while cap.isOpened():
        ret, frame = cap.read()
        # Recolor image to RGB
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image.flags.writeable = False

        # Make detection
        results = pose.process(image)

        # Recolor back to BGR
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        # Extract landmarks
        try:
            landmarks = results.pose_landmarks.landmark

            # Get coordinates
            rightWristX = landmarks[mp_pose.PoseLandmark.RIGHT_WRIST].x
            rightWristY = landmarks[mp_pose.PoseLandmark.RIGHT_WRIST].y

            rightShoulderX = landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER].x
            rightShoulderY = landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER].y

            leftShoulderX = landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER].x

            theta1, theta2, theta3, theta4, theta5, theta6 = wristpos(rightWristX, rightWristY, rightShoulderX, rightShoulderY, leftShoulderX)
            text = '%.2f %.2f %.2f %.2f %.2f %.2f' % (theta1, theta2, theta3, theta4, theta5, theta6)
            print(text)
            # Send UDP message to target
            # data["highest_wrist"]["yRel"] = yRel
            # MESSAGE = bytes(json.dumps(data), "ascii")
            # print(MESSAGE)

            data["h"]["t1"] = theta1
            data["h"]["t2"] = theta2
            data["h"]["t3"] = theta3
            data["h"]["t4"] = theta4
            data["h"]["t5"] = theta5
            data["h"]["t6"] = theta6

            # data["h"]["t1"] = 45
            # data["h"]["t2"] = -45
            # data["h"]["t3"] = -45
            # data["h"]["t4"] = -45

            sock.sendto(bytes(json.dumps(data), "utf-8"), (UDP_IP, UDP_PORT))
            sock.sendto(bytes(json.dumps(data), "utf-8"), (UDP_IP_2, UDP_PORT_2))

            #sock.sendto(str(yRel).encode('UTF-8'), (UDP_IP, UDP_PORT))

            #sock.sendto(str(yRel).encode('UTF-8'), (UDP_IP_2, UDP_PORT_2))
        except:
            pass

        # display info
        cv2.rectangle(image, (0, 0), (255, 73), (245, 117, 16), -1)
        cv2.putText(image, text,
                    (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 2, cv2.LINE_AA)

        # Render detections
        mp_drawing.draw_landmarks(image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                                  mp_drawing.DrawingSpec(color=(245, 117, 66), thickness=2, circle_radius=2),
                                  mp_drawing.DrawingSpec(color=(245, 66, 230), thickness=2, circle_radius=2))

        # print(results)

        cv2.imshow("Mediapipe Feed", image)

        if cv2.waitKey(10) & 0xFF == ord('q'):
            break
cap.release()
cv2.destroyAllWindows()