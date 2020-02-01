import cv2
import numpy as np
from util import *
import argparse
import glob
import os
import pickle
from colorama import Fore, Style

def main():
    # parse execution arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', action='store_true', dest='calculate', help='calculate camera calibration matrix from captured images')
    parser.add_argument('path', help='path to save captured images to')
    args = parser.parse_args()

    # path for where to save clibration images
    capture_path = args.path
    
    # the number for the next calibration image to save
    capture_num = 1

    # path to save final calibration data to
    caldata_path = 'calibration.pkl'

    # ensure the capture path directory exists
    if os.path.exists(capture_path):
        print(f'Calibration path: {capture_path}')
    else:
        print(f'Creating calibration folder: {capture_path}')
        os.mkdir(capture_path)

    # check if script is being run with calculation flag
    if args.calculate:
        print('Finding chessboard corners on saved images...')

        # generate known object object points of the chessboard (0, 0), (1, 0), (2, 0)... (8, 5)
        chessboard = (9, 6)
        board_pts = np.zeros((chessboard[0] * chessboard[1], 3), np.float32)
        board_pts[:,:2] = np.mgrid[0:chessboard[0],0:chessboard[1]].T.reshape(-1,2)

        obj_pts = []
        img_pts = []

        # get list of file names for captured images
        img_names = glob.glob(f'{capture_path}/*.png')
        for img_name in img_names:
            # load the image and convert to grayscale
            img = cv2.imread(img_name)
            img_grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # try to find the chessboard corners
            ret, corners = cv2.findChessboardCorners(img_grey, chessboard, None)       

            print(img_name, ret)
            if ret:
                # if the corners were found, add the object points and corners to respective lists
                obj_pts.append(board_pts)
                img_pts.append(corners)
            else:
                # delete the image if no chessboard was found to save time when running repeatedly
                print(Fore.RED + f'No chessboard found... deleting image' + Style.RESET_ALL)
                os.remove(img_name)
        
        # only run calculation if there are enough images
        if len(obj_pts) > 5:
            print('Calculating calibration...')

            # calculate the camera projection matrix and distortion coefficients
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_pts, img_pts, img_grey.shape[::-1], None, None)
            print(ret, mtx, dist)

            # store calibration data in a dictionary
            cal_data = {}
            cal_data['matrix'] = mtx
            cal_data['dist'] = dist

            # save calibration dictionary to a file using Python pickling
            pickle.dump(cal_data, open(caldata_path, 'wb'))
            print('Calibration data saved!')

        else:
            print(Fore.RED + 'Not enough calibration data... please capture more images with good lighting' + Style.RESET_ALL)

    else:
        # the script is being run to capture images

        print('Capturing...')
        print(Fore.YELLOW + 'Press [Spacebar] to save the current image for calculation later.')
        print('Press [Esc] to exit.' + Style.RESET_ALL)
        
        # setup capture stream using gstreamer pipeline
        cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
        if cap.isOpened():
            while True:
                # read a frame from the video stream
                _, img = cap.read()

                # show the frame so user can ensure pictures have variation in location and rotation of chessboard
                cv2.imshow('Capture', img)
                key = cv2.waitKey(1)
                if key == 27:
                    # when [ESC] pressed, exit script
                    break
                elif key == 32:
                    # when [Spacebar] pressed, save the current frame to the disk

                    # find next available image filename based on number: 1.png, 2.png, 3.png, ...
                    while True:
                        img_name = f'{capture_path}/{capture_num}.png'
                        if not os.path.exists(img_name):
                            break
                        capture_num += 1

                    # write the image to the file, then increase the capture number for the next frame to be saved
                    print(img_name)
                    cv2.imwrite(img_name, img)
                    capture_num += 1

            # close the video stream and the preview window
            cap.release()
            cv2.destroyAllWindows()
        else:
            print(Fore.RED + 'Cannot open stream!' + Style.RESET_ALL)    


if __name__ == '__main__':
    main()