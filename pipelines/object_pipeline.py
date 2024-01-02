from pathlib import Path
from typing import final

from Network_Tables_Sender import nts

import depthai as dai

from pipelines.pipeline import Pipeline

@final
class ObjectPipeline (Pipeline):
    pipeline: dai.Pipeline

    def __init__(self):
        self.pipeline = ObjectPipeline._create_pipeline()

    def _run(
        self,
        mxid:str
    ):
        if (mxid is None):
            #cry
            pass
        with dai.Device(
            pipeline = self.pipeline,
            devInfo = dai.DeviceInfo(mxid)
        ) as device:
            device: dai.Device

            xoutDetect = device.getOutputQueue(name="detections", maxSize=1, blocking=False)
            still_queue = device.getOutputQueue(name="still_out", maxSize=1, blocking=False)
            ctrl_queue = device.getInputQueue(name='still_in')

            while not self.graceful_shutdown:
                event_name = device.getQueueEvent(['detections', 'still_out'])
                if event_name == "detections":
                    detections: dai.SpatialImgDetections = xoutDetect.get()

                    ntData = []
                    for detection in detections.detections:
                        event_name = detection.label
                        x = detection.spatialCoordinates.x/1000
                        y = detection.spatialCoordinates.y/1000
                        z = detection.spatialCoordinates.z/1000

                        ntData.extend([x,y,z,event_name])
                        print("\t", {'x':x, 'y': y, 'z': z, 'event_name': event_name, 'confidnce': detection.confidence})
                    nts.smart_dashboard.putNumberArray("detections", ntData)
                    nts.flush()
                if event_name == "still_out":
                    img = still_queue.get().getCvFrame()
                    #Todo: save
                    # io_q.put_nowait(img)



    @staticmethod
    def _create_pipeline():
        # Create pipeline
        pipeline = dai.Pipeline()

        # Define sources and outputs
        camRgb = pipeline.create(dai.node.ColorCamera)
        spatialDetectionNetwork = pipeline.createYoloSpatialDetectionNetwork()
        monoLeft = pipeline.create(dai.node.MonoCamera)
        monoRight = pipeline.create(dai.node.MonoCamera)
        stereo = pipeline.create(dai.node.StereoDepth)
        xoutNN = pipeline.create(dai.node.XLinkOut)
        ctrl_in = pipeline.create(dai.node.XLinkIn)
        ctrl_out = pipeline.create(dai.node.XLinkOut)

        xoutNN.setStreamName("detections")
        ctrl_in.setStreamName("still_in")
        ctrl_out.setStreamName("still_out")


        # Properties
        camRgb.setPreviewSize(416, 416)
        camRgb.setStillSize(640, 640)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

        # setting node configs
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        # Align depth map to the perspective of RGB camera, on which inference is done
        stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
        stereo.setOutputSize(monoLeft.getResolutionWidth(), monoLeft.getResolutionHeight())

        nnBlobPath = str((Path(__file__).parent / Path('./data/moeNetV1.blob')).resolve().absolute())
        spatialDetectionNetwork.setBlobPath(nnBlobPath)
        spatialDetectionNetwork.setConfidenceThreshold(0.75)
        spatialDetectionNetwork.input.setBlocking(False)
        spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
        spatialDetectionNetwork.setDepthLowerThreshold(100)
        spatialDetectionNetwork.setDepthUpperThreshold(5000)

        # Yolo specific parameters
        spatialDetectionNetwork.setNumClasses(3)
        spatialDetectionNetwork.setCoordinateSize(4)
        spatialDetectionNetwork.setAnchors([
            10.0,
            13.0,
            16.0,
            30.0,
            33.0,
            23.0,
            30.0,
            61.0,
            62.0,
            45.0,
            59.0,
            119.0,
            116.0,
            90.0,
            156.0,
            198.0,
            373.0,
            326.0])
        spatialDetectionNetwork.setAnchorMasks({
            "side52": [
                0,
                1,
                2
            ],
            "side26": [
                3,
                4,
                5
            ],
            "side13": [
                6,
                7,
                8
            ]
        })
        spatialDetectionNetwork.setIouThreshold(0.5)

        # Linking
        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)

        camRgb.preview.link(spatialDetectionNetwork.input)
        camRgb.still.link(ctrl_out.input)
        ctrl_in.out.link(camRgb.inputControl)

        spatialDetectionNetwork.out.link(xoutNN.input)

        stereo.depth.link(spatialDetectionNetwork.inputDepth)

        return pipeline



