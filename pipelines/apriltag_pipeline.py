from datetime import timedelta

import depthai as dai
import numpy as np

import Network_Tables_Sender
from pipelines.pipeline import Pipeline
from moenet_exceptions import CameraNotFoundException
from time import time
import pupil_apriltags as apriltag
import tag
import json
import cv2

class ApriltagPipeline(Pipeline):
        
    def _run(
        self,
        config : dict
    ):
        if (config["device_id"] is None):
            #cry
            return
        pipeline = _create_pipeline(config)
        detector = apriltag.Detector(families=config["tag_family"], nthreads=2)

        device = dai.Device(
            pipeline = pipeline,
            devInfo = dai.DeviceInfo(config["device_id"])
        )
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
            #TODO: Timeout
            label = device.getQueueEvent(queueNames=['mono'], timeout=timedelta(milliseconds=200))
            if label == "":
                #Empty string means no event, camera probably unhappy. Shutdown

                device.close()
                break
            if label == "mono":
                img = monoq.get().getCvFrame()

                results = detector.detect(
                    img,
                    estimate_tag_pose=True,
                    tag_size=0.15875,
                    camera_params=oak_d_camera_params
                )
                self.last_image_ts = time()
                Network_Tables_Sender.nts.camera_debug.putString("tags_last_image_ts", str(self.last_image_ts))

                if not results:
                    Network_Tables_Sender.nts.flush()
                    continue

                results.sort(
                    reverse=True,
                    key = lambda x: x.pose_err
                )
                result: apriltag.Detection = results[0]
                pose = _parse_detection(
                    result,
                    config["flip_image"]
                )

                pose = [round(value, 3) for value in pose]

                Network_Tables_Sender.nts.camera_data.putString("pose", json.dumps({
                    "x": pose[0],
                    "y": pose[1],
                    "z": pose[2],
                }))

                Network_Tables_Sender.nts.smart_dashboard.putNumberArray("pose", pose)
                self.last_detection_ts = time()
                Network_Tables_Sender.nts.camera_debug.putString("tags_last_detection_ts", str(self.last_detection_ts))

                Network_Tables_Sender.nts.flush()

def _create_pipeline(config):
    # Create pipeline
    pipeline = dai.Pipeline()

    # Define sources and outputs
    monoLeft = pipeline.create(dai.node.MonoCamera)
    xOutMono = pipeline.create(dai.node.XLinkOut)

    xOutMono.setStreamName("mono")

    monoLeft.setResolution(config["resolution"])
    monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)

    # Linking

    monoLeft.out.link(xOutMono.input)

    return pipeline



def _parse_detection(
    detection: apriltag.Detection,
    flip
):
    translation_cs = detection.pose_t[:,0]

    rotation_cs = detection.pose_R

    rotation_ts = np.linalg.inv(rotation_cs)

    #negate -> translation+robotpos = tag 0,0,0 -> robotpos = -translation
    translation_ts = rotation_ts @ -translation_cs

    #Original frame of reference: x - side to side, y - up and down, z - towards target
    #New frame of reference: x - towards target, y - side to side, z - up and down

    #based on tags whose z axis is pointing the same way as the field x axis
    tag2field = np.array([[0,0,1],[-1,0,0],[0,-1,0]])

    #facing wrong way, rotate 180 around current y axis
    if(tag.rot0[detection.tag_id]):
        tag2field = tag2field @ np.array([[-1,0,0],[0,1,0],[0,0,-1]])

    translation_fs  = tag2field @ translation_ts
    #translation_fs += tag.tagpos[detection.tag_id]

    #Rotation details
    uvecp = [ 0, 0, 1] #plane vector
    uvecn = [ 0,-1, 0] #normal vector

    #If camera is flipped, the normal vector has to be rotated 180
    if flip:
        uvecn = [0,1,0]

    rotvec  = tag2field @ (rotation_ts @ uvecp)
    rollvec = tag2field @ (rotation_ts @ uvecn)

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
    translation_fs *= 39.3701
    return [*translation_fs, *angles]

def default_config():
    return {
        "device_id": "UNSET",
        "flip_image": False,
        "resolution": dai.MonoCameraProperties.SensorResolution.THE_480_P,
        "tag_family": "tag36h11",
        
    }