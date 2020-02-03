from PyQt5 import QtCore, QtGui, QtWidgets, QtWebEngineWidgets
from pymavlink.dialects.v20 import ardupilotmega as mavlink

import serial
import serial.tools.list_ports
import socket
import time
import os


# status messages sent from the Jetson Nano
STATUS_RECORDING_START = b'REC_START'
STATUS_RECORDING_STOP = b'REC_STOP'
STATUS_DETECTION_START = b'DET_START'
STATUS_DETECTION_STOP = b'DET_STOP'

# subclass QWebEnginePage so I can route console.log statements to STDOUT
class WebPage(QtWebEngineWidgets.QWebEnginePage):
    def javaScriptConsoleMessage(self, level, msg, line_num, source_id):
        print(f'[{line_num}] {msg}')


# custom combobox subclass which gives event when user clicks to change the selection
class ComboBox(QtWidgets.QComboBox):
    popup_will_show = QtCore.pyqtSignal()

    # override the normal showPopup function to emit the custom signal
    def showPopup(self):
        self.popup_will_show.emit()
        super(ComboBox, self).showPopup()


# handles the TCP connection to QGroundControl
class SocketThread(QtCore.QThread):
    thread_started = QtCore.pyqtSignal(QtCore.QThread, bool, str)   # signals if the thread started, and an error if any
    client_connected = QtCore.pyqtSignal()                          # signals when QGC makes a TCP connection
    client_disconnected = QtCore.pyqtSignal()                       # signals when QGC disconnects
    received_data = QtCore.pyqtSignal(bytes)                        # signals when QGC sends data

    def __init__(self, port=5760):
        super(QtCore.QThread, self).__init__()
        self.port = port
        self.conn = None
    
    # write data back to QGroundControl if it is connected
    def write(self, data):
        if self.conn is not None:
            self.conn.send(data)

    # main thread function
    def run(self):
        try:
            # try to start the socket server listening on the specified port
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(0.5)
            self.sock.bind(('0.0.0.0', self.port))
            self.sock.listen(1)
            self.thread_started.emit(self, True, None)
        except Exception as e:
            self.thread_started.emit(self, False, str(e))
            return

        # wait for a connection from QGroundControl
        while not self.isInterruptionRequested():
            try:
                conn, addr = self.sock.accept()
            except:
                continue

            # store the connection, and notify that QGC is connected
            self.conn = conn
            self.client_connected.emit()

            # continuously read data from the TCP connection
            while not self.isInterruptionRequested():
                data = self.conn.recv(50)
                
                # if data == None, the connection has been closed
                if not data:
                    self.conn.close()
                    self.conn = None
                    self.client_disconnected.emit()
                    break

                # notify of the received data if there is any
                self.received_data.emit(data)
                time.sleep(0.001)
        
        # close the TCP connection and socket server when the thread should be stopped
        if self.conn is not None:
            self.conn.close()
            self.conn = None
            self.client_disconnected.emit()
        self.sock.close()


# handles the serial connection to the telemetry radio
class SerialThread(QtCore.QThread):
    thread_started = QtCore.pyqtSignal(QtCore.QThread, bool, str)       # signals if the thread started, and an error if any
    received_data = QtCore.pyqtSignal(bytes)                            # signals when any data is received from the telemetry radio

    def __init__(self, port='', baudrate=57600):
        super(QtCore.QThread, self).__init__()
        self.port = port
        self.baudrate = baudrate
        self.ser = serial.Serial()

    # write data to the serial port if it is open
    def write(self, data):
        if self.ser.is_open:
            self.ser.write(data)

    # main thread function
    def run(self):
        # setup the serial port with desired port and baudrate
        self.ser.port = self.port
        self.ser.baudrate = self.baudrate

        # try to open the serial port, and return any errors if they occur
        try:
            self.ser.open()
            self.thread_started.emit(self, True, None)
        except Exception as e:
            self.thread_started.emit(self, False, str(e))
            return

        # continuously read from the serial port
        while not self.isInterruptionRequested():
            if self.ser.in_waiting > 0:
                try:
                    # get a buffer of data from the port, then emit the received data via the signal
                    data = self.ser.read(50)
                    self.received_data.emit(data)
                    time.sleep(0.001)
                except:
                    pass
        
        # close the port when the thread stops
        self.ser.close()

# handles parsing MAVLink messages from the telemetry radio
class MAVThread(QtCore.QThread):
    parsed_message = QtCore.pyqtSignal(mavlink.MAVLink_message) # signal when a message is parsed from the data stream

    def __init__(self):
        super(QtCore.QThread, self).__init__()

        # initialize a MAVLink parser on the same system as QGroundControl, but a different component id
        self.mav = mavlink.MAVLink(None, srcSystem=255, srcComponent=25, use_native=False)

    # adds binary data to the parser's buffer
    def extend_buffer(self, data):
        self.mav.buf.extend(data)

    # main thread function
    def run(self):
        # continuously try to get messages from the MAVLink parser
        while not self.isInterruptionRequested():
            try:
                # tell the parser to parse the next message in the data buffer if available
                msg = self.mav.parse_char(b'')
                if msg is not None:
                    # notify listeners with the message when it is parsed
                    self.parsed_message.emit(msg)
                time.sleep(0.001)
            except:
                continue






# creates and manages the main window
class MainApp(QtWidgets.QWidget):
    def __init__(self):
        super(QtWidgets.QWidget, self).__init__()
        
        # initialize window title and size constraints
        self.setWindowTitle('Search and Rescue GCS')
        self.setMinimumSize(500, 500)  
        self.resize(1200, 800)

        # setup the main layout
        layout = QtWidgets.QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.init_config_panel())      # add the config pannel to the top
        layout.addWidget(self.init_webview())           # then the webview to display map below it
        self.setLayout(layout)

        # create the socket thread and connect functions to respond to its signals
        self.socket_thread = SocketThread(port=5760)
        self.socket_thread.thread_started.connect(self.on_thread_started)
        self.socket_thread.client_connected.connect(self.on_client_connected)
        self.socket_thread.client_disconnected.connect(self.on_client_disconnected)
        self.socket_thread.received_data.connect(self.on_socket_data)
        self.socket_thread.finished.connect(self.on_thread_finished)

        # create the serial thread and connect functions to respond to its signals
        self.serial_thread = SerialThread()
        self.serial_thread.thread_started.connect(self.on_thread_started)
        self.serial_thread.received_data.connect(self.on_serial_data)
        self.serial_thread.finished.connect(self.on_thread_finished)

        # create the MAVLink thread and connect the function to handle parsed messages
        self.mav_thread = MAVThread()
        self.mav_thread.parsed_message.connect(self.on_mav_message)
        self.vehicle_armed = False
        # self.vehicle = None
    
    # called by each thread when it is started, to notify if it was successful, and any error if there was one 
    def on_thread_started(self, thread, success, error):
        # show message box if there was an error starting the thread
        # could happen when TCP port is already taken, or serial port is busy
        if not success:
            print(error)
            return
        
        # after both data threads have been started, update the interface
        if self.socket_thread.isRunning() and self.serial_thread.isRunning():
            # clear the current flight path on the map
            self.page.runJavaScript(f'clearFlightPathPoints();')

            # only enable the stop button, so parameters cannot be changed while the system is running
            self.ser_picker.setEnabled(False)
            self.ser_baud_picker.setEnabled(False)
            self.inet_port_entry.setEnabled(False)
            self.connect_btn.setEnabled(False)
            self.disconnect_btn.setEnabled(True)

    # called by each thread when it is finished running
    def on_thread_finished(self):
        # after both threads have stopped, update the interface
        if not (self.socket_thread.isRunning() or self.serial_thread.isRunning()):
            # make every widget enabled, except the stop button
            self.ser_picker.setEnabled(True)
            self.ser_baud_picker.setEnabled(True)
            self.inet_port_entry.setEnabled(True)
            self.connect_btn.setEnabled(True)
            self.disconnect_btn.setEnabled(False)

    # called when a TCP connection is established (QGroundControl connects)
    def on_client_connected(self):
        print('Connected!')
    
    # called when a TCP connection is destroyed (QGroundControl disconnects)
    def on_client_disconnected(self):
        print('Disconnected!')
    
    # called when a TCP connection sends data (QGroundControl trying to send data to the UAV)
    def on_socket_data(self, data):
        # route the data to the telemetry radio via the serial thread
        self.serial_thread.write(data)
    
    # called when data is received from the serial port (UAV sending data to QGroundControl)
    def on_serial_data(self, data):
        # route the data to the MAVLink thread to parse messages 
        self.mav_thread.extend_buffer(data)

    # called when the MAVLink thread parses a message from the telemetry datastream
    def on_mav_message(self, msg): 
        # handle heartbeat messages from the UAV to check if armed / disarmed
        if msg.name == 'HEARTBEAT' and msg.autopilot != 8:
            armed = (msg.base_mode >> 7 & 1) == 1
            if armed != self.vehicle_armed:
                self.vehicle_armed = armed
                if armed:
                    # clear markers and flight path when arming
                    self.page.runJavaScript(f'clearFlightPathPoints();')
                    self.page.runJavaScript(f'clearDetectionMarkers();')

        # show status messages for recording / detection
        elif msg.name == 'STATUSTEXT' and msg.get_srcComponent() == 30:
            print(f'[Jetson Nano] {msg.text}')

            # do not pass message to QGroundControl
            return

        # the message contains the location of the UAV
        elif msg.name == 'GLOBAL_POSITION_INT':
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            print(f'Lat: {lat}, Lon: {lon}, Rel Alt: {msg.relative_alt / 1e3}, Heading: {msg.hdg / 1e2}')
            
            # add the current location to the flight path on the map
            if self.vehicle_armed:
                self.page.runJavaScript(f'addFlightPathPoint({lat}, {lon});')

        # the message contains data from the object detection code sent by the Jetson Nano
        elif msg.name == 'COMMAND_INT' and msg.command == 31000:
            # unpack the detection result from the message
            probability, lat, lon = msg.param1, msg.x * 1e-7, msg.y * 1e-7
            print(f'[Detector]: {probability:.1%} LAT: {lat}, LON: {lon * 1e-7}')

            # place a marker for the detection result on the map
            self.page.runJavaScript(f'addDetectionMarker({probability}, {lat}, {lon});')
            
            # do not pass message to QGroundControl
            return

        # # route message to the TCP connection via the socket thread
        self.socket_thread.write(msg.get_msgbuf())    
                

    # create widget to hold all configuration widgets
    def init_config_panel(self):
        # create a horizontal layout and an empty widget
        layout = QtWidgets.QHBoxLayout()
        panel = QtWidgets.QWidget()
        panel.setLayout(layout)

        # add a label to the panel
        ser_picker_label = QtWidgets.QLabel()
        ser_picker_label.setText('Serial Port:')
        layout.addWidget(ser_picker_label)

        # create the serial port picker widget
        self.ser_picker = ComboBox()
        # connect the popup will show signal to populate the items with the current serial ports detected
        self.ser_picker.popup_will_show.connect(self.ser_picker_populate_list)
        # popuplate the list with the current serial ports by default
        self.ser_picker_populate_list()
        layout.addWidget(self.ser_picker)

        # add a label to the panel
        ser_baud_label = QtWidgets.QLabel()
        ser_baud_label.setText('Baudrate:')
        layout.addWidget(ser_baud_label)

        # create the serial baudrate picker with various baudrates
        baudrates = [4800, 9600, 14400, 19200, 38400, 57600, 115200, 128000]
        self.ser_baud_picker = QtWidgets.QComboBox()
        # set the items to be those from the list of baudrates
        self.ser_baud_picker.addItems([str(b) for b in baudrates])
        # select 57600 by default because most telemetry radios use this baudrate
        self.ser_baud_picker.setCurrentIndex(baudrates.index(57600))
        layout.addWidget(self.ser_baud_picker)

        # add a label to the pannel
        inet_port_label = QtWidgets.QLabel()
        inet_port_label.setText('TCP Port:')
        layout.addWidget(inet_port_label)

        # create the TCP port entry field
        self.inet_port_entry = QtWidgets.QLineEdit()
        # set default port to 5760, as QGroundControl uses this port by default
        self.inet_port_entry.setText('5760')
        layout.addWidget(self.inet_port_entry)

        # add a flexible spacer to make interface more natural when scaled to different sizes
        layout.addStretch(1)

        # create the connect button
        self.connect_btn = QtWidgets.QPushButton()
        self.connect_btn.setText('Start')
        # connect a function to be called when it is clicked
        self.connect_btn.clicked.connect(self.click_connect_btn)
        layout.addWidget(self.connect_btn)

        # create the disconnect button
        self.disconnect_btn = QtWidgets.QPushButton()
        self.disconnect_btn.setText('Stop')
        # should be disabled by default, until you press start
        self.disconnect_btn.setEnabled(False)
        # connect a function to be called when it is clicked
        self.disconnect_btn.clicked.connect(self.click_disconnect_btn)
        layout.addWidget(self.disconnect_btn)

        # return the panel with all elements already created for it
        return panel

    # populates the serial port picker with the list of current serial ports
    def ser_picker_populate_list(self):
        # get a list of serial ports available to the system
        ports = [port.device for port in serial.tools.list_ports.comports()]

        next_index = None
        # when the list of ports changes (device connected or disconnected),
        # find the index of the current selected device if possible, so user does not notice that the selected port changes without their input
        if self.ser_picker.count() > 0 and self.ser_picker.currentText()  in ports:
            # find the index of the current selected port in the new list of ports if possible
            next_index = ports.index(self.ser_picker.currentText())

        # remove all items from the picker
        self.ser_picker.clear()
        # add the current list of ports to the picker
        self.ser_picker.addItems(ports)

        # if the old selection was found in the new list, select that item so the user does not notice any odd behavior
        if next_index is not None:
            self.ser_picker.setCurrentIndex(next_index)

    # called when the connect button is pressed
    def click_connect_btn(self):
        try:
            # set the port for the socket thread based on the port entered in the text field
            self.socket_thread.port = int(self.inet_port_entry.text())
        except:
            # catch error if text entered is not a valid
            # (only numbers, no letters)
            print(f'INVALID TCP port: {self.inet_port_entry.text()}')
            return

        # set the serial port and baudrate based on the current user selections
        self.serial_thread.port = self.ser_picker.currentText()
        self.serial_thread.baudrate = int(self.ser_baud_picker.currentText())

        # start all the threads
        self.socket_thread.start()
        self.serial_thread.start()
        self.mav_thread.start()
        
    # called when the disconnect button is pressed
    def click_disconnect_btn(self):
        # request an interruption on all of the threads to stop them gracefully
        self.socket_thread.requestInterruption()
        self.serial_thread.requestInterruption()
        self.mav_thread.requestInterruption()
        
    # create the webview widget that displays the map
    def init_webview(self):
        # setup a custom page (subclass of QWebEnginePage)
        self.page = WebPage(self)

        # create the webview
        self.webview = QtWebEngineWidgets.QWebEngineView()
        # set the webview to expand as large as possible in the layout
        self.webview.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        # set the page to the custom page
        self.webview.setPage(self.page)
        # make the webview run faster
        self.webview.setAttribute(QtCore.Qt.WA_NativeWindow, True)
        # connect the load finished signal to a local function
        self.webview.loadFinished.connect(self.web_load_finished)
        
        # start loading the local page that displays the Leaflet.js map
        self.webview.load(QtCore.QUrl.fromLocalFile(os.path.abspath('web/main.html')))

        # return the created webview
        return self.webview

    # called when the webview finishes loading the page
    def web_load_finished(self):
        print('load finished!')
        


# main function of the app
def main():
    # create a QAppllication
    app = QtWidgets.QApplication([])

    # create an instance of the GUI app, and show the window
    main_app = MainApp()
    main_app.show()

    # start the QApplication run loop
    app.exec_()

if __name__ == '__main__':
    main()