# ---- PACKAGE DELIVERY SIMULATION
# --- importing libraries
from djitellopy import tello
import cv2
import cvzone
import numpy as np
import time

# ----------------- INITIALIZATION ---------------------- #
# --- changeable parameters
w, h = 360, 240  # width and height of the processed image
nmsthresh = 0.2 # thresholds used for object id
thresh = 0.7
base_aruco_id = 4  # id of the aruco of the home base
pidz = [0.4, 0.4, 0]  # pd controller in the z direction (can be changed slightly)
pidy = [0.4, 0.4, 0]  # pd controller in the y direction (can be changed slightly)
fbRange = [6200, 9200]  # limits in which to keep the area of the marker in pixels

# --- dummy values and switches
k = 0
k_rotate = 0
k_rotate_base = 0
pErrory = 0
pErrorz = 0
trajectory = 0
reached_target = 0
reached_home = 0

# --- deciding on ARUCO family
# relate to the aruco libraries
aruco_type = "DICT_5X5_100"
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
arucoParams = cv2.aruco.DetectorParameters_create()

# --- function to display the ARUCO markers
def displayAruco(corners, ids, rejected, image):

    if len(corners) > 0:
        ids = ids.flatten()
        for (markerCorner, markerID) in zip(corners, ids):

            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
            cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
            cv2.putText(image, str(markerID), (topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (255, 0, 0), 2)
    return image

# --- function to find the area and centre of any ARUCO marker within the video footage
def getArucoInfo(corners, ids, target):

    centre = [0, 0]
    width_marker = 0
    if len(corners) > 0:
        ids = ids.flatten()
        for markerCorner, markerID in zip(corners, ids):

            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            if markerID == target:
                width_marker = (abs(topRight[0]-topLeft[0])+abs(bottomRight[0]-bottomLeft[0])+abs(topRight[1]-bottomRight[1])
                                    +abs(topLeft[1]-bottomLeft[1]))/2
                centre = [(topRight[0]+bottomRight[0]+topLeft[0]+bottomLeft[0])/4, (topRight[1]+bottomRight[1]+topLeft[1]+bottomLeft[1])/4]

    return [centre, width_marker**2]

# --- function to track the marker
def trackAruco(info, pErrory, pErrorz):

    x, y = info[0]
    errory = x - w // 2
    speedy = errory * pidy[0] + pidy[1] * (errory - pErrory)  # speed is found based on error and pid - PD controller
    speedy = int(np.clip(speedy, -30, 30))  # forces speed between -30 and 30 cm/s
    errorz = -y + h // 2
    speedz = errorz * pidz[0] + pidz[1] * (errorz - pErrorz)
    speedz = int(np.clip(speedz, -30, 30))

    # if no aruco marker is identifed, the drone is stationary
    if x == 0:
        speedy = 0
        speedz = 0
        errorz = 0
        errory = 0

    # keep the tracked marker's area within a limit defined by fbRange
    area = info[1]
    fb = 0
    if area > fbRange[0] and area < fbRange[1]:
        fb = 0
    if area > fbRange[1]:
        fb = - 20
    elif area < fbRange[0] and area != 0:
        fb = 20

    # send speed commands to the tello and return the error
    me.send_rc_control(speedy, fb, speedz, 0)

    return [errory, errorz]

# --- loading the neural network that does object identification (no need to change anything)
classNames = []
classFile = 'coco.names'
with open(classFile, 'rt') as f:
    classNames = f.read().split("\n")
configPath = "ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
weightsPath = "frozen_inference_graph.pb"
net = cv2.dnn_DetectionModel(weightsPath, configPath)
net.setInputSize(320, 320)
net.setInputScale(1.0/127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)


# -------------------- PLACING THE ORDER -------------- #
# this starts the webcam and waits for an object to be presented
cap = cv2.VideoCapture(0)  # if you have multiple webcams, change the value of 0 depending on the device you want to use
while trajectory == 0:

    ret, img = cap.read()
    img = cv2.resize(img, (480, 360))

    # --- identifies objects
    classIds, confs, bbox = net.detect(img, confThreshold=thresh, nmsThreshold=nmsthresh)
    try:
        for classId, conf, box in zip(classIds.flatten(), confs.flatten(), bbox):
            cvzone.cornerRect(img, box)
            cv2.putText(img, f"{classNames[classId - 1].upper()} {round(conf * 100, 2)}", (box[0] + 10, box[1] + 30),
                        cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)
    except:
        pass

    # --- decides which trajectory to follow based on the given object
    try:
        for classId in classIds.flatten():
            if classId == 57:  # broccoli
                trajectory = 1
            if classId == 59:  # pizza
                trajectory = 2
            if classId == 60:  # donut
                trajectory = 3
    except:
        pass

    cv2.imshow("Image", img)
    cv2.waitKey(1)

cap.release()
cv2.destroyAllWindows()


# ------------------- ENFORCING TRAJECTORY ON DRONE -------------- #
# --- starting drone
me = tello.Tello()
me.connect()
time.sleep(1)
print("Battery:", me.get_battery())
me.streamon()
me.takeoff()
me.send_rc_control(0,0,0,0)
time.sleep(1)

# --- loop throughout footage
while True:

    # --- gets frame from the drone footage
    frame = me.get_frame_read().frame
    frame = cv2.resize(frame, (w,h))

    # --- identifies ARUCO markers and starts tracking the one decided upon
    corners, ids, rejected = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)

    # --- going towards the target marker until reaching it
    if reached_target == 0:
        if trajectory != 0:
            if ids is not None:
                for id in ids:
                    if id[0] == trajectory:
                        info = getArucoInfo(corners, ids, id[0])
                        area = info[1]
                        centre = info[0]
                        pErrory, pErrorz = trackAruco(info, pErrory, pErrorz)
                        if area > fbRange[0] and area < fbRange[1] and abs(pErrory) <= 6:
                            reached_target = 1
                            print("Successfully Reached Target")

    # --- turning around, landing at the target and taking off again to go towards the home base
    elif reached_target == 1 and reached_home == 0:

        if k_rotate == 0:

            print("Rotate")
            me.rotate_clockwise(180)
            k_rotate = 1
            me.land()
            time.sleep(2)
            me.takeoff()
            me.send_rc_control(0, 0, 0, 0)


        else:

            if ids is not None:
                for id in ids:
                    if id[0] == base_aruco_id:
                        info = getArucoInfo(corners, ids, id[0])
                        area = info[1]
                        centre = info[0]
                        pErrory, pErrorz = trackAruco(info, pErrory, pErrorz)
                        if area > fbRange[0] and area < fbRange[1] and abs(pErrory) <= 5:
                            reached_home = 1
                            print("Successfully Reached Home")

    # --- once reaching the base, turn around to prepare for new missions and then land
    elif reached_home == 1 and reached_target == 1:

        if k_rotate_base == 0:
            me.rotate_clockwise(180)
            k_rotate_base = 1
            print("Rotate")

        else:

            me.end()
            break

    # --- display
    detected_markers = displayAruco(corners, ids, rejected, frame)
    cv2.imshow("Image", frame)
    cv2.waitKey(1)

    # --- key to forcefully stop the drone
    if cv2.waitKey(1) & 0xFF == ord("q"):
        me.end()
        break