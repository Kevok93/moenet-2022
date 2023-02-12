import Network_Tables_Sender as nts
import tag

import cv2
import depthai as dai

#Special MOE one
import moe_apriltags as apriltag
import numpy as np

flip = 1

debug = 1

if debug:
    import matplotlib.pyplot as plt
    from scipy.spatial.transform import Rotation as R
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
    monocam.setFps(30)
    monocam.setBoardSocket(dai.CameraBoardSocket.LEFT)

    monocam.out.link(monoout.input)
    return pipeline


detector = apriltag.Detector(families="tag16h5", nthreads=2)

def calculate_pose(det: apriltag.Detection):
    translation = result.pose_t[:,0]

    rotation = result.pose_R

    rinv = np.linalg.inv(rotation)

    #negate -> translation+robotpos = tag 0,0,0 -> robotpos = -translation
    tinv = rinv@-translation

    return rinv, tinv, rotation, translation


with dai.Device(create_pipeline()) as device:
    monoq = device.getOutputQueue(name="mono", maxSize=1, blocking=False)

    calibdata = device.readCalibration()
    intrinsics = calibdata.getCameraIntrinsics(dai.CameraBoardSocket.LEFT, destShape=(600,400))
    print(intrinsics)
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
        #New frame of reference: x - towards target, y - side to side, z - up and down

        #based on tags whose z axis is pointing the same way as the field x axis
        tag2field = np.array([[0,0,1],[-1,0,0],[0,-1,0]])

        #facing wrong way, rotate 180 around current y axis
        if(tag.rot0[result.tag_id]):
            tag2field = tag2field@np.array([[-1,0,0],[0,1,0],[0,0,-1]])
        
        translation_fs = tag2field@translation_ts
        translation_fs += tag.tagpos[result.tag_id]

        #Rotation details
        uvecp = [0,0,1] #plane vector
        uvecn = [0,-1,0] #normal vector
        
        #If camera is flipped, the normal vector has to be rotated 180
        if flip:
            uvecn = [0,1,0]
        
        rotvec = tag2field@(rotation_ts@uvecp)
        rollvec = tag2field@(rotation_ts@uvecn)

        #All angles given in deg, +- 180

        #yaw - counterclockwise - 0 in line with [1,0,0]
        yaw = np.arctan2(rotvec[1], rotvec[0])

        #pitch - counterclockwise - 0 in line with [1,0,0]
        pitch = np.arctan2(rotvec[2], rotvec[0])

        #roll - counterclockwise - 0 in line with [0,0,1]
        roll = np.arctan2(rollvec[1], rollvec[2])

        #compile angles and turn them into degrees
        angles = [yaw, pitch, roll]
        angles = [np.rad2deg(a) for a in angles]

        nts.send_pose([*translation_fs, *angles])