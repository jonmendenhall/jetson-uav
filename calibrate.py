import cv2
import time
import argparse
import glob
import os
import numpy as np
import pickle
from colorama import Fore, Style

def gstreamer_pipeline(capture_size=(1280, 720), display_size=(1280, 720), framerate=60, flip_method=0):
    return (
        'nvarguscamerasrc exposuretimerange="10000000 10000000" tnr-strength=1 tnr-mode=2 ee-strength=0.5 aeantibanding=0 maxperf=true ! '
        'video/x-raw(memory:NVMM), '
        'width=%d, height=%d, '
        'format=NV12, framerate=%d/1 ! '
        'nvvidconv flip-method=%d ! '
        'video/x-raw, width=%d, height=%d, format=BGRx ! '
        'videoconvert ! '
        'video/x-raw, format=BGR ! appsink'
        % (*capture_size, framerate, flip_method, *display_size)
    )

def main():    
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', action='store_true', dest='calculate', help='calculate camera calibration matrix from captured images')
    parser.add_argument('path', help='path to save captured images to')
    args = parser.parse_args()

    capture_path = args.path
    capture_num = 1

    caldata_path = 'calibration.pkl'

    
    if os.path.exists(capture_path):
        print(f'Calibration path: {capture_path}')
    else:
        print(f'Creating calibration folder: {capture_path}')
        os.mkdir(capture_path)

    if args.calculate:

        print('Finding chessboard corners...')

        chessboard = (9, 6)
        board_pts = np.zeros((chessboard[0] * chessboard[1], 3), np.float32)
        board_pts[:,:2] = np.mgrid[0:chessboard[0],0:chessboard[1]].T.reshape(-1,2)

        obj_pts = []
        img_pts = []

        img_names = glob.glob(f'{capture_path}/*.png')
        for img_name in img_names:
            img = cv2.imread(img_name)
            img_grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            ret, corners = cv2.findChessboardCorners(img_grey, chessboard, None)
            # if ret:
            #     corners2 = cv2.cornerSubPix(img_grey, corners, (3, 3), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01))

            # cv2.drawChessboardCorners(img, chessboard, corners, ret)

            print(img_name, ret)
            if ret:
                obj_pts.append(board_pts)
                img_pts.append(corners)
            else:
                print(Fore.RED + f'No chessboard found... deleting image' + Style.RESET_ALL)
                os.remove(img_name)

            # cv2.imshow('Capture', img)
            # key = cv2.waitKey(1)
            # if key == 27:
            #     break
        
        if len(obj_pts) > 0:
            print('Calculating calibration...')
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_pts, img_pts, img_grey.shape[::-1], None, None)
            print(ret, mtx, dist)

            cal_data = {}
            cal_data['matrix'] = mtx
            cal_data['dist'] = dist
            pickle.dump(cal_data, open(caldata_path, 'wb'))
            print('Calibration data saved!')


    else:

        if os.path.exists(caldata_path):
            w, h = 1280, 720
            cal_data = pickle.load(open(caldata_path, 'rb'))
            cam_mtx, roi = cv2.getOptimalNewCameraMatrix(cal_data['matrix'], cal_data['dist'], (w, h), 0, (w, h))
        else:
            cal_data = None

        print('Capturing...')
        print(Fore.YELLOW + 'Press [Spacebar] to save the current image for calculation later.')
        print('Press [Esc] to exit.' + Style.RESET_ALL)
        # cap = cv2.VideoCapture(0)
        cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)

        if cap.isOpened():
            while True:
                _, img = cap.read()

                if cal_data is not None:
                    img = cv2.undistort(img, cal_data['matrix'], cal_data['dist'], None, cam_mtx)

                cv2.imshow('Capture', img)
                key = cv2.waitKey(1)
                if key == 27:
                    break
                elif key == 32:

                    while True:
                        img_name = f'{capture_path}/{capture_num}.png'
                        if not os.path.exists(img_name):
                            break
                        capture_num += 1

                    print(img_name)
                    cv2.imwrite(img_name, img)
                    capture_num += 1

            cap.release()
            cv2.destroyAllWindows()
        else:
            print('Cannot open stream!')    



if __name__ == '__main__':
    main()