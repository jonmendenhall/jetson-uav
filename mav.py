from dronekit import connect
import time
from datetime import datetime
import subprocess
import signal
import os
import glob

class App:

    def __init__(self):
        while True:
            try:
                print('Opening connection...')
                self.vehicle = connect('/dev/ttyTHS1', wait_ready=False, baud=57600, source_component=100)
                break
            except:
                print('FAILED: TIMEOUT')
                time.sleep(1)
                
        print('Connected')
        self.vehicle.add_message_listener('RC_CHANNELS_RAW', self.on_rc_channels)
        self.recording = False
        self.record_process = None
        self.running = True

        signal.signal(signal.SIGTERM, self.on_sigterm)

        print(self.vehicle.version)
        print(self.vehicle.message_factory)

    def on_sigterm(self, signum, frame):
        self.running = False

    def on_rc_channels(self, vehicle, name, msg):
        should_record = msg.chan7_raw > 1200
        if should_record != self.recording:
            if should_record:
                file_name = f'Vid_{datetime.now().strftime("%Y-%m-%d_%H-%M-%S")}.mp4'
                print(f'Recording to file: {file_name}')
                self.record_process = subprocess.Popen(['gst-launch-1.0', '-e', 'nvarguscamerasrc', '!' ,'video/x-raw(memory:NVMM),width=1280,height=720,format=NV12,framerate=30/1', '!', 'nvvidconv', 'flip-method=0', '!', 'x264enc', 'speed-preset=3', '!', 'mp4mux', '!', 'filesink', f'location={file_name}'], stdout=subprocess.PIPE, stdin=subprocess.PIPE)
            else:
                if self.record_process is not None:
                    print('Interupting record process...')
                    self.record_process.send_signal(signal.SIGINT)
                    self.record_process.wait()
                    print('Stopped recording')
           
            new_msg = self.vehicle.message_factory.statustext_encode(6, f'Recording: {should_record}'.encode('utf8'))
            self.vehicle.send_mavlink(new_msg)
        
        self.recording = should_record

    def run(self):
        while self.running:
            try:
                #self.vehicle.attitude.
                #self.vehicle.location.global_frame
                time.sleep(1/10)
            except KeyboardInterrupt:
                break
        
        print('Closing connection...')
        self.vehicle.close()



def main():
    app = App()
    app.run()

if __name__ == '__main__':
    main()