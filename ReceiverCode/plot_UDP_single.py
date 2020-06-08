# Can be used with plot_UDP_thread.py to stream multiple sensors at once

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import socket
import sys 
import numpy as np
import time

UDP_IP = '192.168.2.18' # Receiving computer IP address
UDP_PORT = int(sys.argv[1])
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

class State:
    def __init__(self):
        self.fig = plt.figure()
        self.ax1 = plt.axes(xlim=(0, 1), ylim=(-300, 300))
        self.IMUData, = self.ax1.plot([], [], lw = 2)
        self.xs = np.linspace(0, 1, 300)    # x seconds of data where x = 300 * (1/sampling rate)
        self.ys = [0] * 300
        self.counter = 0
        self.end = 0

        plt.xlabel('Sample number')
        plt.ylabel('Z-axis (deg)')
        if(UDP_PORT == 5005): plt.title('Sensor 1 Gyroscope Data') # Change depending on which ports in use for which sensor
        elif(UDP_PORT == 5010): plt.title('Sensor 2 Gyroscope Data')
        elif(UDP_PORT == 5011): plt.title('Sensor 3 Gyroscope Data')
        elif(UDP_PORT == 5025): plt.title('Sensor 4 Gyroscope Data')

    def ani_update(self, i):
        # print((time.perf_counter()-self.end)*1000)
        # self.end = time.perf_counter()
        data, addr = sock.recvfrom(1024) #buffer size is 1024 bytes
        # print("received message: %s" %data)
        data = str(data)
        data = data[2:(len(data)-1)] #get rid of b's and ' '
        self.counter += 1

        if(self.counter % 2 == 0): # plot at 50 Hz
            # print("counter sample no:", self.counter)
            self.ys.pop(0)
            self.ys.append(float(data))
            self.IMUData.set_data(self.xs, self.ys)
            
        return self.IMUData,
    
    def animate(self):
        ani = animation.FuncAnimation(self.fig, self.ani_update, interval=1, blit=True)
        plt.show()

s = State()
s.animate()

    
