
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
from calibrateCamera import calibrate_camera


#functions to put in class
def getTransf(R,t):
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = t
    return T

def orbSlam():
    #cam_mtx = np.array([[9.39926769e+03, 0.00000000e+00, 3.24430228e+02],
    # [0.00000000e+00, 2.76343667e+02, 2.36049276e+02],
     #[0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

    cam_mtx = calibrate_camera() #if need to rerun calibration
    print(cam_mtx)
    # Initialize the camera (0 for default camera, 1 for external camera)
    cap = cv.VideoCapture(0)
    # Initialize the ORB detector
    orb = cv.ORB_create()
    # Check if the camera opened successfully
    if not cap.isOpened():
        print("Error: Could not open video stream.")
        exit()
    lastFrame = None
    frameNo = 0
    cur_pose = np.eye(4)
    while frameNo < 15:
        frameNo += 1
        print()
        # Capture frame-by-frame
        ret, frame = cap.read()

        # If frame is read correctly, ret is True
        if not ret:
            print("Error: Failed to capture frame.")
            break

        # Display the resulting frame
        #cv.imshow("Camera Frame", frame)
        # Draw matches
        if lastFrame is not None:
          kp1, des1 = orb.detectAndCompute(lastFrame, None)
          kp2, des2 = orb.detectAndCompute(frame, None)
          FLANN_INDEX_LSH = 6
          index_params = dict(algorithm=FLANN_INDEX_LSH, table_number=6, key_size=12, multi_probe_level=1)
          search_params = dict(checks=50)
          matches = cv.FlannBasedMatcher(indexParams=index_params, searchParams=search_params).knnMatch(des1,des2, k=2)

          #bf = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)
          #aqmatches2 = bf.match(des1, des2)

          #print(matches)
          good_matches = []
          try:
              for m,n in matches:
                  #print("any match?")
                  if m.distance < 0.8 * n.distance:

                      good_matches.append(m)
                      #print(m)
                      #if m.trainIdx < len(kp2) and m.queryIdx < len(kp1):
                          #print("add points")
                          #pts2.append(kp2[m.trainIdx].pt)
                          #pts1.append(kp1[m.queryIdx].pt)
          except ValueError:
              pass
          if len(good_matches) < 5:
              continue
          img_matches = cv.drawMatches(lastFrame, kp1, frame, kp2, good_matches, None,
                                       flags=cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
          cv.imshow("Matches", img_matches)

          pts1 = np.float32([kp1[m.queryIdx].pt for m in good_matches])
          pts2 = np.float32([kp2[m.trainIdx].pt for m in good_matches])


          #F, mask = cv.findFundamentalMat(pts1, pts2, cv.FM_LMEDS) //optional since we use essential matrix

          E, mask = cv.findEssentialMat(pts1, pts2, cam_mtx, cv.RANSAC, 0.999, 1.0)

          R,t = decomp_essential_mat(E, pts1, pts2, cam_mtx)

          print("r")
          print(R)
          print("t")
          print(t)

          poses = [];
          T = getTransf(R,np.squeeze(t))
          cur_pose = np.matmul(cur_pose, np.linalg.inv(T))
          print("current pose")
          print(cur_pose)
          poses.append(cur_pose)
          plt.scatter(cur_pose[0,3],cur_pose[1,3])

        lastFrame = frame
        # Break the loop when 'q' key is pressed
        if cv.waitKey(1000) & 0xFF == ord('q'):
            break

    # Release the capture and close any OpenCV windows
    cap.release()
    cv.destroyAllWindows()

    plt.xlabel('X axis')
    plt.ylabel('Y axis')
    plt.legend()
    plt.show()


def decomp_essential_mat(E, q1, q2, k):
    """
    Decompose the Essential matrix

    Parameters
    ----------
    E (ndarray): Essential matrix
    q1 (ndarray): The good keypoints matches position in i-1'th image
    q2 (ndarray): The good keypoints matches position in i'th image

    Returns
    -------
    right_pair (list): Contains the rotation matrix and translation vector
    """

    def sum_z_cal_relative_scale(R, t, k):
        # Get the transformation matrix
        T = getTransf(R, t)
        # Make the projection matrix
        P = np.matmul(np.concatenate((k, np.zeros((3, 1))), axis=1), T)

        # Triangulate the 3D points
        hom_Q1 = cv.triangulatePoints(P, P, q1.T, q2.T)
        # Also seen from cam 2
        hom_Q2 = np.matmul(T, hom_Q1)

        # Un-homogenize
        uhom_Q1 = hom_Q1[:3, :] / hom_Q1[3, :]
        uhom_Q2 = hom_Q2[:3, :] / hom_Q2[3, :]

        # Find the number of points there has positive z coordinate in both cameras
        sum_of_pos_z_Q1 = sum(uhom_Q1[2, :] > 0)
        sum_of_pos_z_Q2 = sum(uhom_Q2[2, :] > 0)
        print("find problem")
        epsilon = 1e-10
        # Form point pairs and calculate the relative scale
        relative_scale = np.mean(np.linalg.norm(uhom_Q1.T[:-1] - uhom_Q1.T[1:], axis=-1) /
                                 np.linalg.norm(uhom_Q2.T[:-1] - uhom_Q2.T[1:], axis=-1)+epsilon)
        print(np.linalg.norm(uhom_Q1.T[:-1] - uhom_Q1.T[1:], axis=-1))
        return sum_of_pos_z_Q1 + sum_of_pos_z_Q2, relative_scale

    # Decompose the essential matrix
    print("essential matrix rank:")
    print(np.linalg.matrix_rank(E))
    R1, R2, t = cv.decomposeEssentialMat(E)
    print("t before squeeze:")
    print(t)
    t = np.squeeze(t)

    print("t after squeeze:!!!")
    print(t)
    # Make a list of the different possible pairs
    pairs = [[R1, t], [R1, -t], [R2, t], [R2, -t]]
    print("pairs")
    print(pairs)
    # Check which solution there is the right one
    z_sums = []
    relative_scales = []
    for R, t in pairs:
        z_sum, scale = sum_z_cal_relative_scale(R, t, k)
        z_sums.append(z_sum)
        print("add new scale")
        print(scale)
        relative_scales.append(scale)
    print(len(relative_scales))
    # Select the pair there has the most points with positive z coordinate
    right_pair_idx = np.argmax(z_sums)
    right_pair = pairs[right_pair_idx]
    relative_scale = relative_scales[right_pair_idx]
    R1, t = right_pair
    t = t * relative_scale
    print("t before return")
    print(t)
    print("relative scale")
    print(relative_scale)
    return [R1, t]

