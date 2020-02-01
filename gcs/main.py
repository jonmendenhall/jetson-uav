from PyQt5 import QtCore, QtGui, QtWidgets, QtWebEngineWidgets
from pymavlink.dialects.v20 import ardupilotmega as mavlink
import serial
import serial.tools.list_ports
import socket
import time
import os


class WebPage(QtWebEngineWidgets.QWebEnginePage):
    def javaScriptConsoleMessage(self, level, msg, line_num, source_id):
        print(f'[{line_num}] {msg}')


class ComboBox(QtWidgets.QComboBox):
    popup_will_show = QtCore.pyqtSignal()

    def showPopup(self):
        self.popup_will_show.emit()
        super(ComboBox, self).showPopup()


class SocketThread(QtCore.QThread):
    thread_started = QtCore.pyqtSignal(QtCore.QThread, bool, str)
    client_connected = QtCore.pyqtSignal()
    client_disconnected = QtCore.pyqtSignal()
    received_data = QtCore.pyqtSignal(bytes)

    def __init__(self, port=5760):
        super(QtCore.QThread, self).__init__()
        self.port = port
        self.conn = None
    
    def write(self, data):
        if self.conn is not None:
            self.conn.send(data)

    def run(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(0.5)
            self.sock.bind(('0.0.0.0', self.port))
            self.sock.listen(1)
            self.thread_started.emit(self, True, None)
        except Exception as e:
            self.thread_started.emit(self, False, str(e))
            return

        while not self.isInterruptionRequested():
            try:
                conn, addr = self.sock.accept()
            except:
                continue
            self.conn = conn
            self.client_connected.emit()
            while not self.isInterruptionRequested():
                data = self.conn.recv(50)
                if not data:
                    self.conn.close()
                    self.conn = None
                    self.client_disconnected.emit()
                    break
                self.received_data.emit(data)
        
        if self.conn is not None:
            self.conn.close()
            self.conn = None
            self.client_disconnected.emit()
        self.sock.close()


class SerialThread(QtCore.QThread):
    thread_started = QtCore.pyqtSignal(QtCore.QThread, bool, str)
    received_data = QtCore.pyqtSignal(bytes)

    def __init__(self, port='', baudrate=57600):
        super(QtCore.QThread, self).__init__()
        self.port = port
        self.baudrate = baudrate
        self.ser = serial.Serial()

    def write(self, data):
        if self.ser.is_open:
            self.ser.write(data)

    def run(self):
        self.ser.port = self.port
        self.ser.baudrate = self.baudrate
        try:
            self.ser.open()
            self.thread_started.emit(self, True, None)
        except Exception as e:
            self.thread_started.emit(self, False, str(e))
            return

        while not self.isInterruptionRequested():
            if self.ser.in_waiting > 0:
                try:
                    data = self.ser.read(50)
                    self.received_data.emit(data)
                except:
                    pass

        self.ser.close()

class MAVThread(QtCore.QThread):
    parsed_message = QtCore.pyqtSignal(mavlink.MAVLink_message)

    def __init__(self):
        super(QtCore.QThread, self).__init__()
        self.mav = mavlink.MAVLink(None, srcSystem=255, srcComponent=25, use_native=False)

    def parse_data(self, data):
        self.mav.buf.extend(data)

    def run(self):
        while not self.isInterruptionRequested():
            try:
                msg = self.mav.parse_char(b'')
                if msg is not None:
                    self.parsed_message.emit(msg)
            except:
                continue



class MainApp(QtWidgets.QWidget):
    def __init__(self):
        super(QtWidgets.QWidget, self).__init__()
        self.setWindowTitle('Search and Rescue GCS')
        self.setMinimumSize(500, 500)  
        self.resize(1200, 800)

        layout = QtWidgets.QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.init_config_panel())
        layout.addWidget(self.init_webview())
        self.setLayout(layout)

        self.socket_thread = SocketThread(port=5760)
        self.socket_thread.thread_started.connect(self.on_thread_started)
        self.socket_thread.client_connected.connect(self.on_client_connected)
        self.socket_thread.client_disconnected.connect(self.on_client_disconnected)
        self.socket_thread.received_data.connect(self.on_socket_data)
        self.socket_thread.finished.connect(self.on_thread_finished)

        self.serial_thread = SerialThread()
        self.serial_thread.thread_started.connect(self.on_thread_started)
        self.serial_thread.received_data.connect(self.on_serial_data)
        self.serial_thread.finished.connect(self.on_thread_finished)

        self.mav_thread = MAVThread()
        self.mav_thread.parsed_message.connect(self.on_mav_message)
        
    def on_thread_started(self, thread, success, error):
        if not success:
            print(error)
            return
        
        if self.socket_thread.isRunning() and self.serial_thread.isRunning():
            self.page.runJavaScript(f'clearFlightPathPoints();')
            self.ser_picker.setEnabled(False)
            self.ser_baud_picker.setEnabled(False)
            self.inet_port_entry.setEnabled(False)
            self.connect_btn.setEnabled(False)
            self.disconnect_btn.setEnabled(True)

    def on_thread_finished(self):
        if not (self.socket_thread.isRunning() or self.serial_thread.isRunning()):
            self.ser_picker.setEnabled(True)
            self.ser_baud_picker.setEnabled(True)
            self.inet_port_entry.setEnabled(True)
            self.connect_btn.setEnabled(True)
            self.disconnect_btn.setEnabled(False)

    def on_client_connected(self):
        print('Connected!')
    
    def on_client_disconnected(self):
        print('Disconnected!')
    
    def on_socket_data(self, data):
        self.serial_thread.write(data)
    
    def on_serial_data(self, data):
        self.socket_thread.write(data)
        self.mav_thread.parse_data(data)

    def on_mav_message(self, msg):
        if msg.name == 'GLOBAL_POSITION_INT':
            print(msg.time_boot_ms/1e3)
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            print(f'Lat: {lat}, Lon: {lon}, Rel Alt: {msg.relative_alt / 1e3}, Heading: {msg.hdg / 1e2}')
            self.page.runJavaScript(f'addFlightPathPoint({lat}, {lon});')
        elif msg.name == 'COMMAND_INT' and msg.target_component != self.mav.srcComponent:
            if msg.command == 31000:
                probability, lat, lon = msg.param1, msg.x, msg.y
                print(f'[Detector]: {probability:.1%} LAT: {lat * 1e-7}, LON: {lon * 1e-7}')
                self.page.runJavaScript(f'addDetectionMarker({probability}, {lat}, {lon});')


    def init_config_panel(self):
        layout = QtWidgets.QHBoxLayout()
        panel = QtWidgets.QWidget()
        panel.setLayout(layout)

        ser_picker_label = QtWidgets.QLabel()
        ser_picker_label.setText('Serial Port:')
        self.ser_picker = ComboBox()
        self.ser_picker.popup_will_show.connect(self.ser_picker_populate_list)
        self.ser_picker_populate_list()
        layout.addWidget(ser_picker_label)
        layout.addWidget(self.ser_picker)

        ser_baud_label = QtWidgets.QLabel()
        ser_baud_label.setText('Baudrate:')
        baudrates = [4800, 9600, 14400, 19200, 38400, 57600, 115200, 128000]
        self.ser_baud_picker = QtWidgets.QComboBox()
        self.ser_baud_picker.addItems([str(b) for b in baudrates])
        self.ser_baud_picker.setCurrentIndex(baudrates.index(57600))
        layout.addWidget(ser_baud_label)
        layout.addWidget(self.ser_baud_picker)

        inet_port_label = QtWidgets.QLabel()
        inet_port_label.setText('TCP Port:')
        self.inet_port_entry = QtWidgets.QLineEdit()
        self.inet_port_entry.setText('5760')
        layout.addWidget(inet_port_label)
        layout.addWidget(self.inet_port_entry)

        layout.addStretch(1)

        self.connect_btn = QtWidgets.QPushButton()
        self.connect_btn.setText('Start')
        self.connect_btn.clicked.connect(self.click_connect_btn)
        layout.addWidget(self.connect_btn)

        self.disconnect_btn = QtWidgets.QPushButton()
        self.disconnect_btn.setText('Stop')
        self.disconnect_btn.setEnabled(False)
        self.disconnect_btn.clicked.connect(self.click_disconnect_btn)
        layout.addWidget(self.disconnect_btn)

        return panel

    def ser_picker_populate_list(self):
        next_index = None
        ports = [port.device for port in serial.tools.list_ports.comports()]
        if self.ser_picker.count() > 0 and self.ser_picker.currentText()  in ports:
            next_index = ports.index(self.ser_picker.currentText() )
        self.ser_picker.clear()
        self.ser_picker.addItems(ports)
        if next_index is not None:
            self.ser_picker.setCurrentIndex(next_index)

    def click_connect_btn(self):
        try:
            self.socket_thread.port = int(self.inet_port_entry.text())
        except:
            print(f'INVALID TCP port: {self.inet_port_entry.text()}')
            return

        self.serial_thread.port = self.ser_picker.currentText()
        self.serial_thread.baudrate = int(self.ser_baud_picker.currentText())

        self.socket_thread.start()
        self.serial_thread.start()
        self.mav_thread.start()
        

    def click_disconnect_btn(self):
        self.socket_thread.requestInterruption()
        self.serial_thread.requestInterruption()
        self.mav_thread.requestInterruption()
        

    def init_webview(self):
        self.page = WebPage(self)
        self.webview = QtWebEngineWidgets.QWebEngineView()
        self.webview.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.webview.setPage(self.page)
        self.webview.setAttribute(QtCore.Qt.WA_NativeWindow, True)
        self.webview.loadFinished.connect(self.web_load_finished)
        self.webview.load(QtCore.QUrl.fromLocalFile(os.path.abspath('web/main.html')))
        return self.webview

    def web_load_finished(self):
        print('load finished!')
        


def main():
    app = QtWidgets.QApplication([])
    main_app = MainApp()
    main_app.show()
    app.exec_()

if __name__ == '__main__':
    main()