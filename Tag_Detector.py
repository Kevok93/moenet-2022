import cv2
import pupil_apriltags as apriltag

camera = cv2.VideoCapture(0)

detector = apriltag.Detector(families="tag16h5")

while True:
    ret, img = camera.read()
    #should be reading from the gray camera from oak-D
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    results = detector.detect(gray)
    for result in results:
        (ptA, ptB, ptC, ptD) = result.corners
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))
        # draw the bounding box of the AprilTag detection
        cv2.line(img, ptA, ptB, (0, 255, 0), 2)
        cv2.line(img, ptB, ptC, (0, 255, 0), 2)
        cv2.line(img, ptC, ptD, (0, 255, 0), 2)
        cv2.line(img, ptD, ptA, (0, 255, 0), 2)
        # draw the center (x, y)-coordinates of the AprilTag
        (cX, cY) = (int(result.center[0]), int(result.center[1]))
        cv2.circle(img, (cX, cY), 5, (0, 0, 255), -1)
        # draw the tag family on the image
        tagFamily = result.tag_family.decode("utf-8")
        cv2.putText(img, tagFamily, (ptA[0], ptA[1] - 15),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        print("[INFO] tag family: {}".format(tagFamily))

        #Pose Detection Part

#        pose, e0, e1 = detector.detection_pose(result, camera_params)

    # show the output image after AprilTag detection
    cv2.imshow("image", img)
    cv2.waitKey(1)