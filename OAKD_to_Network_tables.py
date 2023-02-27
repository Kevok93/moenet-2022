from pathlib import Path
import Network_Tables_Sender as nts
import tag

import depthai as dai

#Special MOE one
import pupil_apriltags
import moe_apriltags as apriltag
import numpy as np

flip = 1

debug = False

if debug:
    import matplotlib.pyplot as plt
    from scipy.spatial.transform import Rotation as R
    plt.ion()
    fig = plt.figure()
    ax1 = fig.add_subplot(1,1,1)


buffer = []

objmxid = ""
tagmxid = ""

#Creating pipeline
def create_pipeline():
    # Create pipeline
    pipeline = dai.Pipeline()

    # Define sources and outputs
    camRgb = pipeline.create(dai.node.ColorCamera)
    spatialDetectionNetwork = pipeline.createYoloSpatialDetectionNetwork()
    monoLeft = pipeline.create(dai.node.MonoCamera)
    monoRight = pipeline.create(dai.node.MonoCamera)
    stereo = pipeline.create(dai.node.StereoDepth)
    xoutNN = pipeline.create(dai.node.XLinkOut)
    xOutMono = pipeline.create(dai.node.XLinkOut)

    xOutMono.setStreamName("mono")

    xoutNN.setStreamName("detections")

    # Properties
    camRgb.setPreviewSize(416, 416)
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

    monoLeft.out.link(xOutMono.input)

    camRgb.preview.link(spatialDetectionNetwork.input)

    spatialDetectionNetwork.out.link(xoutNN.input)

    stereo.depth.link(spatialDetectionNetwork.inputDepth)

    return pipeline

#Creating tag pipeline
def tag_create_pipeline():
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

#Creating object pipeline
def obj_create_pipeline():
    # Create pipeline
    pipeline = dai.Pipeline()

    # Define sources and outputs
    camRgb = pipeline.create(dai.node.ColorCamera)
    spatialDetectionNetwork = pipeline.createYoloSpatialDetectionNetwork()
    monoLeft = pipeline.create(dai.node.MonoCamera)
    monoRight = pipeline.create(dai.node.MonoCamera)
    stereo = pipeline.create(dai.node.StereoDepth)
    xoutNN = pipeline.create(dai.node.XLinkOut)

    xoutNN.setStreamName("detections")

    # Properties
    camRgb.setPreviewSize(416, 416)
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

    spatialDetectionNetwork.out.link(xoutNN.input)

    stereo.depth.link(spatialDetectionNetwork.inputDepth)

    return pipeline

detector = apriltag.Detector(families="tag16h5", nthreads=2)

def calculate_pose(det: apriltag.Detection):
    translation = result.pose_t[:,0]

    rotation = result.pose_R

    rinv = np.linalg.inv(rotation)

    #negate -> translation+robotpos = tag 0,0,0 -> robotpos = -translation
    tinv = rinv@-translation

    return rinv, tinv, rotation, translation
def main():
    try:
        # 0 - Camera on back for april tag detection, 1 - Camera on front for objet detection
        with dai.Device(tag_create_pipeline(), dai.DeviceInfo(tagmxid)) as tagdevice, dai.Device(obj_create_pipeline(), dai.DeviceInfo(objmxid)) as objdevice:
            tagdevice: dai.Device
            objdevice: dai.Device
            monoq = tagdevice.getOutputQueue(name="mono", maxSize=1, blocking=False)
            xoutDetect = objdevice.getOutputQueue(name="detections", maxSize=1, blocking=False)

            #April Tag Calibration Data
            calibdata = tagdevice.readCalibration()
            intrinsics = calibdata.getCameraIntrinsics(dai.CameraBoardSocket.LEFT, destShape=(600,400))

            oak_d_camera_params = (
                intrinsics[0][0],
                intrinsics[1][1],
                intrinsics[0][2],
                intrinsics[1][2],
            )
            currentcam = 0
            while True:
                # Change camera viewed every other iteration
                currentcam ^= 1
                if currentcam == 0:
                    if not monoq.has():
                        continue
                    label = "mono"
                elif currentcam == 1:
                    if not xoutDetect.has():
                        continue
                    label = "detections"

        #        print(label)

                if label == "mono":
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

                elif label == "detections":
                    detections: dai.SpatialImgDetections = xoutDetect.get()

                    ntData = []
                    for detection in detections.detections:
                        label = detection.label
                        x = detection.spatialCoordinates.x/1000
                        y = detection.spatialCoordinates.y/1000
                        z = detection.spatialCoordinates.z/1000

                        ntData.extend([x,y,z,label])
                        print("\t", {'x':x, 'y': y, 'z': z, 'label': label, 'confidnce': detection.confidence})
                    nts.send_detections(ntData)
    except Exception as e:
        print(e)
        main()    

main()