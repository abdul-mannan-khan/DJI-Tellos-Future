# --- importing libraries
import cv2
from djitellopy import tello
import cvzone

# --- setting the object identifier
thresh = 0.6
nmsthresh = 0.2
classNames = []
classFile = 'coco.names'  # gives names of recognized objects
with open(classFile, 'rt') as f:
    classNames = f.read().split("\n")
configPath = "ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
weightsPath = "frozen_inference_graph.pb"
net = cv2.dnn_DetectionModel(weightsPath, configPath)
net.setInputSize(320, 320)
net.setInputScale(1.0/127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)

# --- starting the stream on the Tello
me = tello.Tello()
me.connect()
print("Battery is:", me.get_battery())
me.streamoff()
me.streamon()

# --- processing the image given by the Tello while streaming
while True:

    # getting the image from the drone's video footage
    img = me.get_frame_read().frame
    classIds, confs, bbox = net.detect(img, confThreshold=thresh, nmsThreshold=nmsthresh)
    try:
        for classId, conf, box in zip(classIds.flatten(), confs.flatten(), bbox):
            cvzone.cornerRect(img, box)
            cv2.putText(img, f"{classNames[classId-1].upper()} {round(conf*100, 2)}", (box[0]+10, box[1]+30),
                        cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)
    except:
        pass
    me.send_rc_control(0,0,0,0)
    cv2.imshow("Image", img)

    # --- forced stop
    if cv2.waitKey(1) & 0xFF == ord("q"):
        me.end()
        break