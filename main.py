from dronekit import connect
from darknet import *
import cv2
import time
from datetime import datetime
import subprocess
import signal
import os
import glob
import pickle
from colorama import Fore, Style
from math import *
from util import *

class App:

    def __init__(self):

        # attempt connection until successful
        while True:
            try:
                print('Opening connection...')
                self.vehicle = connect('/dev/ttyTHS1', wait_ready=False, baud=57600, source_component=100, rate=10)
                break
            except:
                # if there is no heartbeat from the Pixhawk within the timeout period (30s), retry after 1s
                print('FAILED: TIMEOUT')
                time.sleep(1)

        print('Connected')
        print(self.vehicle.version)
        print(self.vehicle.message_factory)
        
        # setup basic runtime variables for the script
        # self.vehicle.add_message_listener('RC_CHANNELS_RAW', self.on_rc_channels)
        self.recording = False
        self.record_process = None
        self.running = True

        # add callback for any termination signals to the script for graceful exiting
        signal.signal(signal.SIGTERM, self.on_sigterm)

    def start_recording(self, file_name):
        if self.record_process is None:
            self.record_process = subprocess.Popen(['gst-launch-1.0', '-e', 'nvarguscamerasrc', 'exposuretimerange="10000000 10000000"', 'tnr-strength=1', 'tnr-mode=2', 'ee-strength=0.5', '!' ,'video/x-raw(memory:NVMM),width=1280,height=720,format=NV12,framerate=30/1', '!', 'nvvidconv', 'flip-method=0', '!', 'x264enc', 'speed-preset=3', '!', 'mp4mux', '!', 'filesink', f'location={file_name}'], stdout=subprocess.PIPE, stdin=subprocess.PIPE)

    def stop_recording(self):
        if self.record_process is not None:
            self.record_process.send_signal(signal.SIGINT)
            self.record_process.wait()
            self.record_process = None

    def on_sigterm(self, signum, frame):
        self.running = False
        self.stop_recording()

    def on_rc_channels(self, vehicle, name, msg):
        should_record = msg.chan7_raw > 1200
        if should_record != self.recording:
            if should_record:
                file_name = f'Vid_{datetime.now().strftime("%Y-%m-%d_%H-%M-%S")}.mp4'
                print(f'Recording to file: {file_name}')
                self.start_recording(file_name)
            else:
                self.stop_recording()
                print('Interupting record process...')
                print('Stopped recording')
           
            new_msg = self.vehicle.message_factory.statustext_encode(6, f'Recording: {should_record}'.encode('utf8'))
            self.vehicle.send_mavlink(new_msg)
        
        self.recording = should_record

    def run(self):
        
        caldata_path = 'calibration.pkl'

        # load calibration data if it exists
        if os.path.exists(caldata_path):
            cal_data = pickle.load(open(caldata_path, 'rb'))
        else:
            # exit if could not locate the calibration data file
            print(Fore.RED + 'No calibration data! Please run calibrate.py first.' + Style.RESET_ALL)
            return

        # load YOLOv3 network and weights needed for object detection
        net = load_net('model/eagleeye.cfg', 'model/eagleeye.weights', 0)
        meta = load_meta('model/eagleeye.data')

        # open the video capture pipeline using a gstreamer pipeline string
        pipeline = gstreamer_pipeline(capture_size=(1280, 720), display_size=(1280, 720), framerate=60, flip_method=0        
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

        # exit if the video stream could not be opened
        if not cap.isOpened():
            print(Fore.RED + 'Could not open video stream!' + Style.RESET_ALL)
            return
        
        while self.running:
            try:
                # get location, heading, and attitude of the UAV just before the camera frame is captured
                location = self.vehicle.location.global_relative_frame
                heading = self.vehicle.heading
                attitude = self.vehicle.attitude

                # get the camera frame from the capture stream
                _, img = cap.read()
                img_w, img_h = img.shape[1], img.shape[0]

                # run YOLOv3 detection on the frame using the model loaded at startup
                objs = detect(net, meta, img, thresh=0.3)

                # list of center points of all objects detected
                pts = []

                # add the center point of each object to the list
                for obj in objs:
                    classifier, probability, bbox = obj
                    x, y, w, h = bbox
                    center_x = (x + w / 2) * img_w
                    center_y = (y + h / 2) * img_h
                    pts.append([center_x, center_y])

                # only do the undistortion and transformation calculations if there were any objects detected
                # to speed up the detection loop
                if len(pts) > 0:

                    # remove camera distortion from the locations of each object detected
                    pts_undist = cv2.undistortPoints(np.array(pts).reshape(1,-1,2), cal_data['matrix'], cal_data['dist']).reshape(-1, 2)
                    
                    # generate a vector from the focal point to the camera plane for each detected object (right, back, down) relative to the UAV
                    obj_vectors = np.hstack((pts_undist, np.ones((pts_undist.shape[0], 1))))
                    
                    # calculate transformation matrix to orient points in the camera in a North-East-Down reference frame
                    # corrects for roll, pitch, and heading of the UAV
                    mat_transform = matrix_rot_y(attitude.roll) @ matrix_rot_x(-attitude.pitch) @ matrix_rot_z(-(90 + heading) * pi / 180)

                    for i, obj in enumerate(objs):
                        classifier, probability, bbox = obj
                        obj_vec = obj_vectors[i]

                        # transform the vector toward the object to be in a North-East-Down reference frame relative to the UAV
                        ned_vec = obj_vec @ mat_transform

                        # approximate lattitude and longitude by extending the object's vector from the location and altitude of the UAV down to the ground
                        obj_lat = location.lat + location.alt * (ned_vec[0] / ned_vec[2]) * DEGREES_PER_METER
                        obj_lon = location.lon + location.alt * (ned_vec[1] / ned_vec[2]) * DEGREES_PER_METER

                        msg = self.vehicle.message_factory.command_int_encode(255, 25, 0, 31000, 0, 0, 0.1, 0, 0, 0, int(obj_lat * 1e7), int(obj_lon * 1e7), 0)
                        self.vehicle.send_mavlink(msg)
                        # print(probability, obj_lat, obj_lon)

            except KeyboardInterrupt:
                break
            
    def stop(self):
        # close the connection to the Pixhawk before stopping the script
        print('Closing connection...')
        self.vehicle.close()



def main():
    # setup and run the application to completion
    app = App()
    app.run()
    app.stop()

if __name__ == '__main__':
    main()