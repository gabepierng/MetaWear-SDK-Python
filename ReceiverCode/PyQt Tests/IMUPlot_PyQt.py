import sys
import time
from pyqtgraph.Qt import QtCore, QtGui
import numpy as np
import pyqtgraph as pg
import socket

UDP_IP = '192.168.0.17'
UDP_PORT = 2390
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
stream_freq = 100
total_time = 1
numPts = stream_freq * total_time

class App(QtGui.QMainWindow):
    def __init__(self, parent=None):
        super(App, self).__init__(parent)

        #### Create Gui Elements ###########
        self.mainbox = QtGui.QWidget()
        self.setCentralWidget(self.mainbox)
        self.mainbox.setLayout(QtGui.QVBoxLayout())

        self.canvas = pg.GraphicsLayoutWidget()
        self.mainbox.layout().addWidget(self.canvas)
        self.canvas1 = pg.GraphicsLayoutWidget()
        self.mainbox.layout().addWidget(self.canvas1)

        self.label = QtGui.QLabel()
        self.mainbox.layout().addWidget(self.label)

        self.IMUplot = self.canvas.addPlot()
        self.IMUplot.setYRange(-300, 300, padding=0)
        self.h2 = self.IMUplot.plot(pen='w')

        #### Set Data  #####################

        self.fps = 0.
        self.lastupdate = time.time()

        self.x = np.linspace(0, total_time, numPts)    # x seconds of data where numPts = total time * (1/sampling rate)
        self.y = [0] * numPts
        self.counter = 0
        self.start = 0
        self.end = 0

        #### Start  #####################
        self._update()

    def _update(self):

        self.data, _ = sock.recvfrom(1024)
        self.data = float(self.data)
        self.y.pop(0)
        self.y.append(self.data)

        self.h2.setData(self.y)

        now = time.time()
        dt = (now-self.lastupdate)
        if dt <= 0:
            dt = 0.000000000001
        fps2 = 1.0 / dt
        self.lastupdate = now
        self.fps = self.fps * 0.9 + fps2 * 0.1
        tx = 'Mean Frame Rate:  {fps:.3f} FPS'.format(fps=self.fps )
        self.label.setText(tx)
        QtCore.QTimer.singleShot(1, self._update)
        self.counter += 1


if __name__ == '__main__':

    app = QtGui.QApplication(sys.argv)
    thisapp = App()
    thisapp.show()
    sys.exit(app.exec_())