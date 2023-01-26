import Network_Tables_Sender as nts

import cv2
import depthai as dai
#Special MOE one
import moe_apriltags as apriltag
from time import monotonic
import numpy as np

#OAK-D
using_OAKD = True

if(using_OAKD):
    #Camera Parameters (fx, fy, cx, cy)
    oak_d_camera_params = (794.311279296875, 794.311279296875, 620.0385131835938, 371.195068359375)


    #Creating pipeline
    pipeline = dai.Pipeline()

    monocam = pipeline.create(dai.node.MonoCamera)

    monoout = pipeline.create(dai.node.XLinkOut)

    monoout.setStreamName("mono")

    monocam.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
    monocam.setFps(120)
    monocam.setBoardSocket(dai.CameraBoardSocket.LEFT)

    monocam.out.link(monoout.input)

detector = apriltag.Detector(families="tag16h5", nthreads=2)

with dai.Device(pipeline) as device:
    monoq = device.getOutputQueue(name = "mono")

    while True:
        img = monoq.get().getCvFrame()

        results = detector.detect(img,
                                l=1,
                                r=8,
                                maxhamming=0,
                                estimate_tag_pose=True,
                                tag_size=.1524,
                                camera_params=oak_d_camera_params)
        for result in results:
            #result.tag_id
            cposet = result.pose_t*39.3701
            cposet = [i[0] for i in cposet]
            cposetnames = ['x','y','z']
            for i in range(3):
                nts.sfloat(cposetnames[i], cposet[i])
            
            #find rotation vector
            rnames = ['pitch', 'roll', 'yaw']
            rvals = [0]*3

            rot = result.pose_R
            rot = np.linalg.inv(rot)
            vec = np.array([1,0,0])
            rvec = np.dot(rot,vec)

            rvals[0] = np.arctan2(rvec[2], rvec[1])
            rvals[1] = np.arctan2(rvec[1], rvec[0])
            rvals[2] = np.arctan2(rvec[0], rvec[2])

            for i in range(3):
                nts.sfloat(rnames[i], rvals[i])