#  importing libraries
import cv2
import numpy as np
from djitellopy import tello
import time


# constants
fbRange = [6200, 6800]  # range where to keep the face
pid = [0.4, 0.4, 0]  # pid gains
pError = 0  # orignal error
w, h = 360, 240  # width and height of image - we resize
pid_height = [0.4, 0.4, 0]
pError_height = 0


# drone initialization
me = tello.Tello()
me.connect()
print("Battery is:", me.get_battery())

me.streamon()
me.takeoff()
me.send_rc_control(0, 0, 25, 0) # to go op
time.sleep(2)

# drone is controlled such that the area is kept at a constant value and the centre is kept at 0, 0
# function to find face - uses pretrained classifier
def findFace(img):

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

    if len(myFaceListArea) != 0:
        i = myFaceListArea.index(max(myFaceListArea))  # gives max face area index
        return img, [myFaceListC[i], myFaceListArea[i]]
    else:
        return img, [[0,0], 0]


# tracking function
def trackFace(info, w, pid, pError, pid_height, pError_height):

    # pid design basically
    x,y = info[0]
    error = x-w//2
    error_height = -y+h//2
    speed_z = error_height*pid_height[0]+pid_height[1]*(error_height-pError_height)
    speed_z = int(np.clip(speed_z, -30, 30))
    speed = error*pid[0]+pid[1]*(error-pError)  # speed is found based on error and pid - PD controller
    speed = int(np.clip(speed, -100, 100))  # forces speed between -100 and 100
    if x == 0:
        speed = 0
        error = 0
        speed_z = 0

    # moves backwards if area is big, move forward if area is small
    # nothing is found, stay stationary
    area = info[1]
    fb = 0
    if area > fbRange[0] and area < fbRange[1]:
        fb = 0
    if area > fbRange[1]:
        fb = - 20
    elif area < fbRange[0] and area != 0:
        fb = 20

    # print(speed, fb)
    me.send_rc_control(0, fb, speed_z , speed)
    return error  # comes back as previous error

while True:
    img = me.get_frame_read().frame  # gives individual image coming from the drone
    img = cv2.resize(img, (w,h))
    img, info = findFace(img)
    pError = trackFace(info, w, pid, pError, pid_height, pError_height)
    print("Centre", info[0],"Area", info[1])
    cv2.imshow("Output", img)  # gives image
    if cv2.waitKey(1) & 0xFF == ord("q"):
        me.land()
        me.end()
        break


