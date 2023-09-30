#!/usr/bin/env python3

import cv2
import numpy as np
import depthai as dai

qcd = cv2.QRCodeDetector()
def detectCoordinate(img):
    returnValue, decoded_info, points, _ = qcd.detectAndDecodeMulti(img)

    if not returnValue:
        return None, None, None

    print(decoded_info)

    x,y,w,h = cv2.boundingRect(points[0])

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
xoutPreview = pipeline.create(dai.node.XLinkOut)

xoutVideo.setStreamName("video")
xoutPreview.setStreamName("preview")

# Properties
camRgb.setPreviewSize(300, 300)
camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setInterleaved(True)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

# Linking
camRgb.video.link(xoutVideo.input)
camRgb.preview.link(xoutPreview.input)

print("Setup complete")

# Connect to device and start pipeline
with dai.Device(pipeline) as device:

    video = device.getOutputQueue('video', 1,  blocking=False)
    preview = device.getOutputQueue('preview')
    print("Trying to get video")

    i = 0;

    while True:
        print("Getting next frame")
        videoFrame = video.get()
        previewFrame = preview.get()
        print("Frame recieved")

        cvFrame = videoFrame.getCvFrame()

        img, center_x, center_y = detectCoordinate(cvFrame)

        if img is not None:
            cv2.imshow("video", cvFrame)
            print("Center of bounding rectangle: (", center_x, ",", center_y, ")")
        else:
            # Get BGR frame from NV12 encoded video frame to show with opencv
            cv2.imshow("video", cvFrame)

        # Show 'preview' frame as is (already in correct format, no copy is made)
        # cv2.imshow("preview", previewFrame.getFrame())

        if cv2.waitKey(1) == ord('q'):
            break
