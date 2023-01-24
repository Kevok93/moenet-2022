import cv2
import depthai as dai
#Special MOE one
import pupil_apriltags as apriltag
from time import monotonic

#For webcam
#camera = cv2.VideoCapture(0)

#OAK-D

#Camera Parameters (fx, fy, cx, cy)
oak_d_camera_params = (794.311279296875, 794.311279296875, 620.0385131835938, 371.195068359375)


#Creating pipeline
pipeline = dai.Pipeline()

monocam = pipeline.create(dai.node.MonoCamera)

monoout = pipeline.create(dai.node.XLinkOut)

monoout.setStreamName("mono")

monocam.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
monocam.setFps(30)
monocam.setBoardSocket(dai.CameraBoardSocket.LEFT)

monocam.out.link(monoout.input)

detector = apriltag.Detector(families="tag16h5", nthreads=2)

cnt = 0

ct = monotonic()


times = [0]*100

with dai.Device(pipeline) as device:
#    calibdata = device.readCalibration()
#    print(calibdata.getDefaultIntrinsics(dai.CameraBoardSocket.LEFT))
    monoq = device.getOutputQueue(name = "mono")

    while True:
        #timing data
        ct = monotonic()
        img = monoq.get().getCvFrame()


        #Count Frames
        cnt += 1
        
        #Use this to convert to grayscale if needed - should be reading from the gray camera from oak-D
        #img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        results = detector.detect(img, estimate_tag_pose=True, tag_size=.1524, camera_params=oak_d_camera_params)
        for result in results:
            if result.hamming > 0 or result.tag_id == 0 or result.tag_id > 8:
                continue
                       
            
            
            print(result.pose_t*39.3701)
           
        key = cv2.waitKey(1)
        if key == ord('q') or key == ord('Q'):
            break