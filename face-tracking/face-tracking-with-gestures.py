# --- importing libraries
import cv2
import numpy as np
import mediapipe as mp
from djitellopy import tello
import time

# constants
landed = 0
fbRange = [3000, 4600]  # range where to keep the face
pid = [0.4, 0.4, 0]  # pid gains for yaw control
pError = 0  # original error
pid_height = [0.4, 0.4, 0]  # pid gains for control in the z direction
pError_height = 0  # original error
w, h = 360, 240  # width and height of image - we resize

# --- initializing the face
mp_pose = mp.solutions.pose
mpDraw = mp.solutions.drawing_utils
pose = mp_pose.Pose()

# drone is controlled such that the area is kept at a constant value and the centre is kept at 0, 0
# function to find face - uses pretrained classifier
def findFace(img):

    # initialization
    faceCascade = cv2.CascadeClassifier("Resources/haarcascade_frontalface_default.xml")
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # converts to grey scale
    faces = faceCascade.detectMultiScale(imgGray, 1.2, 8)  # detects faces
    myFaceListC = []  # centre points of faces
    myFaceListArea = []  # values of face areas

    # for loop to find the biggest face
    for (x, y, w, h) in faces:
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)  # creates rectangles around faces
        cx = x+w//2
        cy = y+h//2
        area = w*h
        cv2.circle(img, (cx,cy), 5, (0,255,0), cv2.FILLED)
        myFaceListC.append([cx, cy])
        myFaceListArea.append(area)

    # ensure that, if no faces are identified, the drone is stationary
    if len(myFaceListArea) != 0:
        i = myFaceListArea.index(max(myFaceListArea))  # gives max face area index
        return img, [myFaceListC[i], myFaceListArea[i]]
    else:
        return img, [[0,0], 0]

# drone initialization
me = tello.Tello()
me.connect()
time.sleep(1)
print("Battery is:", me.get_battery())
me.streamon()
me.takeoff()
me.send_rc_control(0, 0, 25, 0)
time.sleep(2)


# tracking function that computes the error and commands given to the drones
def trackFace(info, w, pid, pError, pid_height, pError_height):

    # computes the
    x,y = info[0]
    error = x-w//2
    speed = error*pid[0]+pid[1]*(error-pError)  # yaw speed is found based on error and pid - PD controller
    speed = int(np.clip(speed, -100, 100))  # forces yaw speed between -100 and 100 deg/s
    error_height = -y+h//2
    speed_z = error_height*pid_height[0]+pid_height[1]*(error_height-pError_height)
    speed_z = int(np.clip(speed_z, -30, 30))  # forces vertical speed between -30 and 30 cm/s

    # if no face is identified, the drone should be kept stationary
    if x == 0:
        speed = 0
        error = 0
        speed_z = 0

    # moves backwards if area is big, move forward if area is small
    area = info[1]
    fb = 0
    ud = 0

    if area > fbRange[0] and area < fbRange[1]:
        fb = 0
    if area > fbRange[1]:
        fb = - 20
    elif area < fbRange[0] and area != 0:
        fb = 20

    me.send_rc_control(0, fb, speed_z , speed)
    return error  # comes back as previous error


# --- processing the image given by the Tello
while True:

    # reading the video footage and simplifying it for faster computations
    img = me.get_frame_read().frame
    img = cv2.resize(img, (w,h))
    img, info = findFace(img)
    rgb_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    result = pose.process(rgb_img)

    # incoporation of gesture commands
    if info[1] != 0:
        if result.pose_landmarks:

            lwrist_y = result.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_WRIST].y * h
            rwrist_y = result.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_WRIST].y * h
            lwrist_x = result.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_WRIST].x * h
            rwrist_x = result.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_WRIST].x * h
            nose_y = result.pose_landmarks.landmark[mp_pose.PoseLandmark.NOSE].y * h
            nose_x = result.pose_landmarks.landmark[mp_pose.PoseLandmark.NOSE].x * h

            # raise both wrists above the base of your nose for the drone to flip
            if lwrist_y < nose_y and rwrist_y < nose_y and nose_x > rwrist_x and nose_x < lwrist_x:

                me.send_rc_control(0,0,0,0)
                me.flip_back()
                time.sleep(.5)
                pError = 0

            # put your left wrist to the right of your nose and right wrist to the left of your nose
            # and ensure both wrists are below your nose
            elif nose_x < rwrist_x and nose_x > lwrist_x and landed == 0:

                me.send_rc_control(0,0,0,0)
                me.end()
                landed = 1
                pError = 0

            # raise left wrist above your nose to make the Tello go to the right (your left)
            elif nose_x < lwrist_x and lwrist_y < nose_y and rwrist_y > nose_y:

                pError = 0
                me.send_rc_control(25, 0, 0, 0)

            # raise right wrist above your nose to make the Tello go the the left (your right)
            elif nose_x > rwrist_x and rwrist_y < nose_y and lwrist_y > nose_y:

                pError = 0
                me.send_rc_control(-25, 0, 0, 0)

            # if you use gesture commands, make the drone stop tracking your face
            else:

                pError = trackFace(info, w, pid, pError, pid_height, pError_height)

        else:

            pError = trackFace(info, w, pid, pError, pid_height, pError_height)

    # function to draw your body as identified by the pose classifier
    mpDraw.draw_landmarks(img, result.pose_landmarks, mp_pose.POSE_CONNECTIONS)
    cv2.imshow("Output", img)  # gives image

    if cv2.waitKey(1) & 0xFF == ord("q"):

        me.end()
        break


