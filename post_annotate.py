import cv2
from darknet import *
from util import *
import numpy as np
import argparse
import time
import pickle
import os
from main import CAM_MOUNT_ANGLE

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('telemetry_file', help='path for file with live telemetry saved from record process')
    parser.add_argument('video_file', help='path for file with live video saved from record process')
    parser.add_argument('output_file', help='path for annoted video output')
    parser.add_argument('-preview', action='store_true', help='show preview while annotating video')
    parser.add_argument('-location_file', help='path for file to write detected object locations')
    args = parser.parse_args()

    caldata_path = 'calibration.pkl'

    # load calibration data if it exists
    if os.path.exists(caldata_path):
        cal_data = pickle.load(open(caldata_path, 'rb'))
    else:
        # exit if could not locate the calibration data file
        print(Fore.RED + 'No calibration data! Please run calibrate.py first.' + Style.RESET_ALL)
        return

    # load each line of telemetry data from the file
    print(f'loading telemetry from {args.telemetry_file}')
    telemetry = []
    telemetry_t0 = None
    with open(args.telemetry_file, 'r') as f:
        for line in f.readlines():

            # parse the ';' separated values
            parts = line.strip().split(';')

            # create dictionary of fields for easy access
            record = {
                'timestamp': float(parts[0]),
                'lat': float(parts[1]),#int(parts[1]) * 1e-7,
                'lon': float(parts[2]),#int(parts[2]) * 1e-7,
                'alt': float(parts[3]),
                'heading': float(parts[4]),
                'roll': float(parts[5]),
                'pitch': float(parts[6]),
            }

            # ensures all timestamps are relative to the first timestamp
            if telemetry_t0 is None:
                telemetry_t0 = record['timestamp']
            record['timestamp'] -= telemetry_t0

            # add record to the list of telemetry records
            telemetry.append(record)
    
    # load video input
    print(f'loading capture from {args.video_file}')
    capture = cv2.VideoCapture(args.video_file)

    # create video output
    print(f'creating output to {args.output_file}')
    # put black borders around video so detection results can show still show at edges
    padding = 60
    writer = cv2.VideoWriter(args.output_file, cv2.VideoWriter_fourcc(*'mp4v'), 25, (1280 + padding * 2, 720 + padding * 2))

    # create location file if specified in args
    if args.location_file is not None:
        print(f'creating output to {args.location_file}')
        output_loc = open(args.location_file, 'w')

    # load YOLOv3 network and weights needed for object detection
    print('loading darknet model')
    net = load_net(b'model/eagleeye.cfg', b'model/eagleeye.weights', 0)
    meta = load_meta(b'model/eagleeye.data')

    # setup loop variables
    telem_i = 0
    frame_i = 0
    frame_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))

    while True:
        try:
            # read frame from recorded video
            ret, frame = capture.read()

            # break if this was the last frame
            if not ret:
                break

            # get frame size
            img_w, img_h = frame.shape[1], frame.shape[0]

            # create padded image
            frame_padded = np.zeros((img_h + padding * 2, img_w + padding * 2, 3), dtype=np.uint8)
            frame_padded[padding:-padding, padding:-padding] = frame
            
            # get the timestamp of this frame relative to the start of the video
            frame_time = capture.get(cv2.CAP_PROP_POS_MSEC) / 1000

            # find the latest possible telemtry data based on timestamp <= frame_time
            if telem_i + 1 < len(telemetry):
                while telemetry[telem_i + 1]['timestamp'] <= frame_time:
                    telem_i += 1
            frame_telem = telemetry[telem_i]

            print(f'[{frame_i+1}/{frame_count}] Frame Time: {frame_time:.3f} | Latest Telem: {frame_telem["timestamp"]:.3f}')
           
            # increment frame counter
            frame_i += 1
            
            if frame_i > 100:
                # run YOLOv3 detection on the frame using the model loaded at startup
                objs = detect(net, meta, frame, thresh=0.6)
            else:
                objs = []

            # list of center points of all objects detected
            pts = []

            # add the center point of each object to the list
            for obj in objs:
                classifier, probability, bbox = obj
                x, y, w, h = bbox
                pts.append([x, y])

            # only do the undistortion and transformation calculations if there were any objects detected
            # to speed up the detection loop
            if len(pts) > 0:

                # remove camera distortion from the locations of each object detected
                pts_undist = cv2.undistortPoints(np.array(pts).reshape(1,-1,2), cal_data['matrix'], cal_data['dist']).reshape(-1, 2)
                
                # generate a vector from the focal point to the camera plane for each detected object (x,y,z = right,back,down) relative to the UAV
                obj_vectors = np.hstack((pts_undist, np.ones((pts_undist.shape[0], 1))))

                # calculate transformation matrix to orient points in the camera in a North-East-Down reference frame
                # corrects for roll, pitch, and heading of the UAV
                mat_transform = matrix_rot_y(frame_telem['roll'] * pi / 180) @ matrix_rot_x((-frame_telem['pitch'] - CAM_MOUNT_ANGLE) * pi / 180) @ matrix_rot_z(-(90 + frame_telem['heading']) * pi / 180)

                for i, obj in enumerate(objs):
                    classifier, probability, bbox = obj
                    w = int(bbox[2])
                    h = int(bbox[3])
                    x = int(bbox[0] - bbox[2] / 2 + padding)
                    y = int(bbox[1] - bbox[3] / 2 + padding)
                    obj_vec = obj_vectors[i]

                    # transform the vector toward the object to be in a North-East-Down reference frame relative to the UAV
                    ned_vec = obj_vec @ mat_transform

                    # approximate lattitude and longitude by extending the object's vector from the location and altitude of the UAV down to the ground
                    obj_lat = frame_telem['lat'] + frame_telem['alt']  * (ned_vec[0] / ned_vec[2]) * DEGREES_PER_METER
                    obj_lon = frame_telem['lon'] + frame_telem['alt']  * (ned_vec[1] / ned_vec[2]) * DEGREES_PER_METER

                    # draw rectangle around detected object
                    cv2.rectangle(frame_padded, (x, y), (x + w, y + h), (255, 0, 255), 2)

                    # set text style for annotations above objects
                    text_scale = 0.6
                    text_thickness = 1
                    
                    # calculate line sizes based on text and font settings
                    lines = [f'LON: {obj_lon:.7f}', f'LAT: {obj_lat:.7f}', f'Person: {probability:.1%}']
                    line_sizes = [cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, text_scale, text_thickness)[0] for line in lines]
                    
                    # variables for drawing the correct size box
                    max_line_width = max(line_size[0] for line_size in line_sizes)
                    line_height = line_sizes[0][1]
                    line_spacing = 6
                    
                    # draw box to surround all lines of text for this object's annotations
                    cv2.rectangle(frame_padded, (x, y - (len(lines) * line_height + (len(lines) + 1) * line_spacing)), (x + max_line_width, y), (255, 0, 255), -1)
                    
                    # draw each line of text (bottom to top)
                    for l_i, line in enumerate(lines):
                        cv2.putText(frame_padded, line, (x, y - l_i * (line_height + line_spacing) - line_spacing), cv2.FONT_HERSHEY_SIMPLEX, text_scale, (0, 0, 0), text_thickness, cv2.LINE_AA)

                    if args.location_file is not None:
                        output_loc.write(f'{frame_i};{frame_time:.3f};{probability:.4f};{int(obj_lat*1e10)};{int(obj_lon*1e10)}\n')
            
            if args.location_file is not None:
                output_loc.flush()

            # write annotated frame to output
            writer.write(frame_padded)
            
            # show preview if run with flag
            if args.preview: 
                cv2.imshow(args.video_file, frame_padded)
                key = cv2.waitKey(1)
                if key == 27:
                    break
        except KeyboardInterrupt:
            break

    
    # close input and output video streams
    writer.release()    
    capture.release()
    if args.location_file is not None:
        output_loc.close()
    cv2.destroyAllWindows()
    


if __name__ == '__main__':
    main()