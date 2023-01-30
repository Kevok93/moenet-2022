import Network_Tables_Sender as nts

import cv2
import depthai as dai
#Special MOE one
import quaternion
import moe_apriltags as apriltag
from time import monotonic
import numpy as np

import matplotlib.pyplot as plt
plt.ion()
from scipy.spatial.transform import Rotation as R

fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)


buffer = []

#Creating pipeline
def create_pipeline():
    pipeline = dai.Pipeline()

    monoout = pipeline.createXLinkOut()
    monoout.setStreamName("mono")

    monocam = pipeline.createMonoCamera()
    monocam.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
    monocam.setFps(120)
    monocam.setBoardSocket(dai.CameraBoardSocket.LEFT)

    monocam.out.link(monoout.input)
    return pipeline


detector = apriltag.Detector(families="tag16h5", nthreads=2)

def calculate_pose(det: apriltag.Detection):
    rotation = R.from_matrix(result.pose_R)
    print(result.pose_R)
    translation = result.pose_t[:,0]

    rotation_inv = rotation.inv()
    translation_inv = rotation.apply(-translation, inverse=True)

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
        
        cv2.imshow('foo', img)
        if cv2.waitKey(1) == ord('q'):
            break

        results = [result for result in results if result.tag_id == 1]
        if len(results) == 0:
            continue
        results.sort(reverse=True, key = lambda x: x.pose_err)
        result: apriltag.Detection = results[0]

        rotation_ts, translation_ts, rotation_cs, translation_cs = calculate_pose(result)

        # rotation = quaternion.from_rotation_matrix(result.pose_R)
        # buffer.append(quaternion.as_euler_angles(rotation))
        if len(buffer) > 20:
            buffer = buffer[-20:]
        
        if False:
            buffer.append(rotation.as_euler('zyx'))
        else:
            buffer.append([*translation_ts, *translation_cs])
        ax1.cla()
        b = np.array(buffer)

        if False:
            ax1.set(ylim=(-np.pi, np.pi))
            ax1.plot(b[:,0], label='roll')
            ax1.plot(b[:,1], label='yaw')
            ax1.plot(b[:,2], label='pitch')
            ax1.legend()
        else:
            ax1.set(xlim=(-3,3), ylim=(-3,3))
            ax1.scatter([0], [0])
            ax1.plot(b[:,0], b[:,2])
            ax1.plot(b[:,3], b[:,5])
        plt.draw()
        plt.pause(.01)
        continue

        print(tuple(f'{x:01.03f}' for x in quaternion.as_euler_angles(rotation)))

        rotation_inv = -rotation
        # translation_inv = -translation
        translation_inv = quaternion.rotate_vectors(rotation_inv, (-translation))

        rotation_inv = quaternion.as_euler_angles(rotation_inv)

        # translation_inv[:] = translation_inv[[2,0,1]]
        svals = list(translation_inv)+list(rotation_inv)
        # for val in svals:
        #     print(f'{val:01.03f} ', end='')
        # print()
        nts.send_pose(svals)
        continue


        #result.tag_id
        cposet = result.pose_t #*39.3701
        #x,y,z
        cposet = cposet[:, 0]
        # cposet = [
        #     cposet[2],cposet[0]+4,cposet[1] 
        # ]
        
        #find rotation vector
        #rnames = ['pitch', 'roll', 'yaw']
        rvals = [0]*3

        rot = result.pose_R
        # rot = np.linalg.inv(rot)
        rotvec = np.array([1,0,0])
        robvec = np.array(cposet)
        robvec[0] += 4.0
        rotvec = np.dot(rot,rotvec)
        robvec = np.dot(rot,robvec)

        rvals[0] = np.arctan2(rotvec[2], rotvec[1])
        rvals[1] = np.arctan2(rotvec[1], rotvec[0])
        rvals[2] = np.arctan2(rotvec[0], rotvec[2])

        robvec[:] = robvec[[2,0,1]]

        svals = list(robvec)+list(rvals)
        print(svals)
        nts.send_pose(svals)