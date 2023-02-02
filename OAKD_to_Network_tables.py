import Network_Tables_Sender as nts
import tag

import cv2
import depthai as dai

#Special MOE one
import moe_apriltags as apriltag
import numpy as np

import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

debug = 1

if debug:
    plt.ion()
    fig = plt.figure()
    ax1 = fig.add_subplot(1,1,1)


buffer = []

#Creating pipeline
def create_pipeline():
    pipeline = dai.Pipeline()

    monoout = pipeline.createXLinkOut()
    monoout.setStreamName("mono")

    monocam = pipeline.createMonoCamera()
    monocam.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monocam.setFps(60)
    monocam.setBoardSocket(dai.CameraBoardSocket.LEFT)

    monocam.out.link(monoout.input)
    return pipeline


detector = apriltag.Detector(families="tag16h5", nthreads=2)

def calculate_pose(det: apriltag.Detection):
    rotation = R.from_matrix(result.pose_R)
    
    if debug:
        print(result.pose_R)
    
    translation = result.pose_t[:,0]

    rotation_inv = rotation.inv()
    rinv = np.linalg.inv(result.pose_R)
    translation_inv = rotation.apply(-translation, inverse=True)

    tinv = -rinv@translation

    # print(translation_inv, tinv)

    return rotation_inv, translation_inv, rotation, translation


with dai.Device(create_pipeline()) as device:
    monoq = device.getOutputQueue(name="mono", maxSize=1, blocking=False)

    calibdata = device.readCalibration()
    intrinsics = calibdata.getDefaultIntrinsics(dai.CameraBoardSocket.LEFT)[0]
    oak_d_camera_params = (
        intrinsics[0][0],
        intrinsics[1][1],
        intrinsics[0][2],
        intrinsics[1][2],
    )


    while True:
        img = monoq.get().getCvFrame()

        results = detector.detect(img,
                                l=1,
                                r=8,
                                maxhamming=0,
                                estimate_tag_pose=True,
                                tag_size=.1524,
                                camera_params=oak_d_camera_params)
        
        if debug:
            cv2.imshow('foo', img)
            if cv2.waitKey(1) == ord('q'):
                break
        
        if len(results) == 0:
            continue
        
        results.sort(reverse=True, key = lambda x: x.pose_err)
        result: apriltag.Detection = results[0]

        rotation_ts, translation_ts, rotation_cs, translation_cs = calculate_pose(result)
            
        if(debug):
            if len(buffer) > 20:
                buffer = buffer[-20:]
            
            buffer.append([*translation_ts, *translation_cs])
            b = np.array(buffer)
            ax1.cla()
            ax1.set(xlim=(-5,5), ylim=(-5,5))
            ax1.scatter([0], [0])
            ax1.plot(b[:,0], b[:,2])
            ax1.plot(b[:,3], b[:,5])
            plt.draw()
            plt.pause(.001)
        
        #Original frame of reference: x - side to side, y - up and down, z - towards target
        #New frame of reference: x - towards target, y - side to sidde, z - up and down
        translation_ts = translation_ts[[2,0,1]]

        translation_ts[0] = (-1)**(tag.rot0[result.tag_id])* \
                                translation_ts[0] + \
                                tag.posvec[result.tag_id][0]
        
        translation_ts[1] = (-1)**(tag.rot0[result.tag_id]^1)* \
                                translation_ts[1] + \
                                tag.posvec[result.tag_id][1]


        translation_ts[2] += tag.posvec[result.tag_id][2]
