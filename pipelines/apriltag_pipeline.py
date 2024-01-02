import depthai as dai
import numpy as np

from Network_Tables_Sender import nts
from pipelines.pipeline import Pipeline
import moe_apriltags as apriltag
import tag


detector = apriltag.Detector(families="tag16h5", nthreads=2)
class ApriltagPipeline(Pipeline):
    pipeline: dai.Pipeline
    def __init__(self):
        pipeline = ApriltagPipeline._create_pipeline()


    def _run(
        self,
        mxid,
        flip=0
    ):
        if (mxid is None):
            #cry
            pass
        with dai.Device(
            pipeline = self.pipeline,
            devInfo = dai.DeviceInfo(mxid)
        ) as device:
            device: dai.Device

            monoq = device.getOutputQueue(name="mono", maxSize=1, blocking=False)

            #April Tag Calibration Data
            intrinsics = device.readCalibration().getCameraIntrinsics(
                dai.CameraBoardSocket.LEFT,
                destShape=(600, 400)
            )

            oak_d_camera_params = (
                intrinsics[0][0],
                intrinsics[1][1],
                intrinsics[0][2],
                intrinsics[1][2],
            )
            while not self.graceful_shutdown:
                label = device.getQueueEvent(['mono'])
                if label == "mono":
                    img = monoq.get().getCvFrame()

                    results = detector.detect(
                        img,
                        l=1,
                        r=8,
                        maxhamming=0,
                        estimate_tag_pose=True,
                        tag_size=.1524,
                        camera_params=oak_d_camera_params
                    )


                    if len(results) == 0:
                        continue

                    results.sort(reverse=True, key = lambda x: x.pose_err)
                    result: apriltag.Detection = results[0]
                    pose = self._parse_detection(result,flip)
                    nts.smart_dashboard.putNumberArray("pose", pose)
                    nts.flush()

    def _parse_detection(
        self,
        detection: apriltag.Detection,
        flip
    ):
        translation_cs = detection.pose_t[:,0]

        rotation_cs = detection.pose_R

        rotation_ts = np.linalg.inv(rotation_cs)

        #negate -> translation+robotpos = tag 0,0,0 -> robotpos = -translation
        translation_ts = rotation_ts@-translation_cs

        #Original frame of reference: x - side to side, y - up and down, z - towards target
        #New frame of reference: x - towards target, y - side to side, z - up and down

        #based on tags whose z axis is pointing the same way as the field x axis
        tag2field = np.array([[0,0,1],[-1,0,0],[0,-1,0]])

        #facing wrong way, rotate 180 around current y axis
        if(tag.rot0[detection.tag_id]):
            tag2field = tag2field@np.array([[-1,0,0],[0,1,0],[0,0,-1]])

        translation_fs = tag2field@translation_ts
        translation_fs += tag.tagpos[detection.tag_id]

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
        return [*translation_fs, *angles]


    def _create_pipeline():
        # Create pipeline
        pipeline = dai.Pipeline()

        # Define sources and outputs
        monoLeft = pipeline.create(dai.node.MonoCamera)
        xOutMono = pipeline.create(dai.node.XLinkOut)

        xOutMono.setStreamName("mono")


        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)

        # Linking

        monoLeft.out.link(xOutMono.input)

        return pipeline
