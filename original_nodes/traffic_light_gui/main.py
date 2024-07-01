#!/usr/bin/env python3
# qt5 libraries
from PyQt5.QtCore import Qt, QObject, QThread, pyqtSignal, QRunnable, QThreadPool, QTimer
from PyQt5.QtGui import QCursor
from PyQt5 import QtWidgets, uic

import socket, struct, time

class UDPTrafficSignalStatus:
    def __init__(self, class_id: int, confidence: float, xmin: float, xmax: float, ymin: float, ymax: float, class_name: str):
        # yolo data itself
        self.class_id = class_id
        self.confidence = confidence
        self.xmin = xmin
        self.xmax = xmax
        self.ymin = ymin
        self.ymax = ymax
        self.class_name = class_name

        # autoware TrafficSignalElement
        self.bytes = self.create_encoded_bytestring()

    def create_encoded_bytestring(self) -> bytearray:
        data = struct.pack("Bfffff32s", self.class_id, self.confidence, self.xmin, self.xmax, self.ymin, self.ymax, self.class_name.encode('ascii'))
        return data

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        uic.loadUi('main.ui', self)

        self.startBtn.clicked.connect(self.startBtn_callback_)
        self.stopBtn.clicked.connect(self.stopBtn_callback_)
        self.confidenceBox.setValue(1.00)
        self.xminBox.setValue(0.0)
        self.xmaxBox.setValue(1.0)
        self.yminBox.setValue(0.0)
        self.ymaxBox.setValue(1.0)
        #self.textEdit.textChanged.connect
        # show window

        self.publishing_ = False
        self.publish_timer_ = QTimer(interval=100, timeout=self.publish_timer_callback_)
        self.ip_addr_ = self.targetIpBox.text()
        self.port_ = self.targetPortBox.value()
        self.socket_ = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
        self.show()

    def publish_timer_callback_(self):
        header = struct.pack('B', 0x6a)
        length = struct.pack('B', 1)
        final_bytestring = header + length
        final_bytestring += struct.pack("Bfffff32s", 
            self.classBox.currentIndex(),
            self.confidenceBox.value(),
            self.xminBox.value(),
            self.xmaxBox.value(),
            self.yminBox.value(),
            self.ymaxBox.value(),
            "".encode('ascii')
        )
        #print(final_bytestring)
        self.socket_.sendto(final_bytestring, (self.ip_addr_, self.port_))

    def publish_override_cancel_(self):
        for i in range(0, 10): # publish 10 times for good measure
            header = struct.pack('B', 0x6b)
            length = struct.pack('B', 0)
            final_bytestring = header + length
            self.socket_.sendto(final_bytestring, (self.ip_addr_, self.port_))
            time.sleep(0.1)

    def startBtn_callback_(self):
        self.publishing_ = True
        self.startBtn.setDisabled(True)
        self.stopBtn.setDisabled(False)
        self.ip_addr_ = self.targetIpBox.text()
        self.port_ = self.targetPortBox.value()
        self.targetIpBox.setDisabled(True)
        self.targetPortBox.setDisabled(True)
        self.publish_timer_.start()
        self.statusLabel.setText(f"Publishing to {self.ip_addr_}:{self.port_} ...")

    def stopBtn_callback_(self):
        self.publishing_ = False
        self.publish_timer_.stop()
        self.publish_override_cancel_()
        self.startBtn.setDisabled(False)
        self.stopBtn.setDisabled(True)
        self.targetIpBox.setDisabled(False)
        self.targetPortBox.setDisabled(False)
        self.statusLabel.setText(f"Idle.")
            
if __name__ == "__main__":
    import sys

    app = QtWidgets.QApplication(sys.argv)
    mainWindow = MainWindow()

    sys.exit(app.exec_())
