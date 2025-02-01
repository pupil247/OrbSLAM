from operator import truediv

import numpy as np
import cv2 as cv
import glob


def calibrate_camera():
    print("starting calibration")
    #set the termination criteria (stop algorithm when certain precision is acheived
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    objp = np.zeros((4*7,3), np.float32)
    objp[:,:2] = np.mgrid[0:7,0:4].T.reshape(-1,2)

    objpoints = []
    imgpoints = []

    images = glob.glob('*.jpg')
    cap = cv.VideoCapture("http://192.168.2.168:81/stream")
    found = 0

    while found < 30:
        print("reading frame")
        ret, frame = cap.read()
        cv.imshow('img',frame)
        gray = cv.cvtColor(frame,cv.COLOR_BGR2GRAY)

        #find chessboard corners of image 7x6
        ret, corners = cv.findChessboardCorners(gray, (4,7), None)
        print(corners)
        print(ret)
        if ret == True:
            print("found match")
            found +=1
            objpoints.append(objp)
            #inc
            corners2 = cv.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            imgpoints.append(corners2)

            #draw and display corners on the img
            cv.drawChessboardCorners(frame, (7,6), corners2, ret)
            cv.imshow('img',frame)

        cv.waitKey(1000)
    cap.release()
    cv.destroyAllWindows()

    #perform calibration based on the points collected
    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    #camera matrice
    print(mtx)
    return mtx