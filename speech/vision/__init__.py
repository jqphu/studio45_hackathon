#!/usr/bin/env python3

import cv2
import depthai as dai

import os
import sys
import time

sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI

arm = XArmAPI("192.168.1.185")
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)

# Left side
leftPosition = [69.295586, 388.210449, 193.558914, 179.046962, 8.577293, -98.641216]
originPosition = [257.599991, -0.0, 430.800018, 180, -0.0, 180]

def go_position(pos):
    arm.set_position(x=pos[0], y=pos[1], z=pos[2],roll=pos[3], pitch=pos[4], yaw=pos[5], wait=True);


qcd = cv2.QRCodeDetector()
def detectCoordinate(img, target):
    returnValue, decoded_info, points, _ = qcd.detectAndDecodeMulti(img)

    if not returnValue:
        return None, None, None

    if target == "CASTROL":
        target = "SALT"
    else:
        target = "PEPPER"

    # Assume max 2 itmes ,pepper and salt.
    if decoded_info[0] == target:
        x,y,w,h = cv2.boundingRect(points[0])
    elif len(decoded_info) > 1 and decoded_info[1] == target:
        x,y,w,h = cv2.boundingRect(points[1])
    else:
        return None, None, None



    center_x = x + w / 2
    center_y = y + h / 2

    img = cv2.polylines(img, points.astype(int), True, (0, 255, 0), 3)

    for s, p in zip(decoded_info, points):
        img = cv2.putText(img, s, p[0].astype(int),
                          cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

    return img, center_x, center_y

#    cv2.imshow('QR Code', img)  # Display the image in a window named 'QR Code'
#    cv2.waitKey(1)  # Wait for a key press
#
#    cv2.imwrite('data/dst/qrcode_opencv.jpg', img)

# Create pipeline
pipeline = dai.Pipeline()

# Define source and outputs
camRgb = pipeline.create(dai.node.ColorCamera)
xoutVideo = pipeline.create(dai.node.XLinkOut)

xoutVideo.setStreamName("video")

# Properties
camRgb.setPreviewSize(300, 300)
camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setInterleaved(True)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

# Linking
camRgb.video.link(xoutVideo.input)

print("Setup complete")

redCupZ = 178

def pickup(target):
# Connect to device and start pipeline
    with dai.Device(pipeline) as device:
        video = device.getOutputQueue('video', 1,  blocking=False)
        print("Trying to get video")

        found = False

        go_position(originPosition)
        searchCounter = 0

        while True:
            print("Getting next frame")
            videoFrame = video.get()
            print("Frame recieved")

            cvFrame = videoFrame.getCvFrame()

            img, center_x, center_y = detectCoordinate(cvFrame, target)
            print("Center of bounding rectangle: (", center_x, ",", center_y, ")")

            if img is not None:
                cv2.imshow("video", cvFrame)
                center_x -= 960
                # Offset due to the camera
                center_y -= 275
                print("Center of bounding rectangle: (", center_x, ",", center_y, ")")

                # let's move in x

                # to far away form positive-x -. we wnat to move negative
                if center_x > 200:
                    print("Trying to reduce x")
                    arm.set_position(y=-10, relative=True, wait=True)
                elif center_x < -200:
                    print("Trying to increase x")
                    arm.set_position(y=+10, relative=True, wait=True)
                elif center_y > 200:
                    print("Trying to reduce y")
                    arm.set_position(x=-10, relative=True, wait=True)
                elif center_y < -200:
                    print("Trying to increase y")
                    arm.set_position(x=+10, relative=True, wait=True)

                # to far away form positive-x -. we wnat to move negative
                if center_x > 100:
                    print("Trying to reduce x")
                    arm.set_position(y=-5, relative=True, wait=True)
                elif center_x < -100:
                    print("Trying to increase x")
                    arm.set_position(y=+5, relative=True, wait=True)
                elif center_y > 100:
                    print("Trying to reduce y")
                    arm.set_position(x=-5, relative=True, wait=True)
                elif center_y < -100:
                    print("Trying to increase y")
                    arm.set_position(x=+5, relative=True, wait=True)

                # to far away form positive-x -. we wnat to move negative
                if center_x > 50:
                    print("Trying to reduce x")
                    arm.set_position(y=-3, relative=True, wait=True)
                elif center_x < -50:
                    print("Trying to increase x")
                    arm.set_position(y=+3, relative=True, wait=True)
                elif center_y > 50:
                    print("Trying to reduce y")
                    arm.set_position(x=-3, relative=True, wait=True)
                elif center_y < -50:
                    print("Trying to increase y")
                    arm.set_position(x=+3, relative=True, wait=True)

                if center_x < 50 and center_x > -50 and center_y < 50 and center_y > -50:
                    found=True
                    arm.set_vacuum_gripper(True)
                    # Sleep to suck
                    arm.set_position(z=redCupZ, wait=True)
                    time.sleep(1)

                searchCounter = -1000000

            else:
                # Get BGR frame from NV12 encoded video frame to show with opencv
                cv2.imshow("video", cvFrame)
                searchCounter+=1
                print("Search", searchCounter)
                if searchCounter > 50:
                    return False

            # Show 'preview' frame as is (already in correct format, no copy is made)
            # cv2.imshow("preview", previewFrame.getFrame())

            if found:
                arm.set_position(z=400, wait=True)
                go_position(leftPosition)
                arm.set_vacuum_gripper(False)
                go_position(originPosition)
                return True

            if cv2.waitKey(1) == ord('q'):
                break

if __name__ == "__main__":
    pickup("CASTROL")  # This will execute the main function when the script is run.

