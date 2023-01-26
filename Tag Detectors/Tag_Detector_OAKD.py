import cv2
import depthai as dai
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
                       
            
            
            #Annotates images
            """
            (ptA, ptB, ptC, ptD) = result.corners
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            ptA = (int(ptA[0]), int(ptA[1]))
            # draw the bounding box of the AprilTag detection
            cv2.line(img, ptA, ptB, (0, 255, 0),60)
            cv2.line(img, ptB, ptC, (0, 255, 0), 60)
            cv2.line(img, ptC, ptD, (0, 255, 0), 60)
            cv2.line(img, ptD, ptA, (0, 255, 0), 60)
            # draw the center (x, y)-coordinates of the AprilTag
            (cX, cY) = (int(result.center[0]), int(result.center[1]))
            cv2.circle(img, (cX, cY), 5, (0, 0, 255), -1)
            # draw the tag family on the image
            tagFamily = result.tag_family.decode("utf-8")
            cv2.putText(img, str(result.tag_id), (ptA[0], ptA[1] - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            print("[INFO] tag family: {}".format(str(result.hamming)))
            
            
            #Prints pose detection information
            print(result.pose_t*39.3701)
            """
        
        if(cnt%20 == 0):
            mt = monotonic()
            t = mt-ct

            print('Avg:', 100/(mt-times[cnt%100]))
            times[cnt%100] = mt

            if(t != 0):
                print('Instant:',1/(t))
            else:
                print('Subzero?')
        
        """
        # show the output image after AprilTag detection
        cv2.imshow("image", img)
        """
        
        key = cv2.waitKey(1)
        if key == ord('q') or key == ord('Q'):
            break