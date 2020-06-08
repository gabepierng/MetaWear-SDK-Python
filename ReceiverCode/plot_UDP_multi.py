import matplotlib.pyplot as plt
import matplotlib.animation as animation
import socket
import numpy as np
from time import sleep

UDP_IP = '192.168.2.216'

class Sensor:
    def __init__(self, UDP):
        self.xs = np.linspace(0, 1, 300)
        self.ys = [0] * 300
        self.counter = 0
        self.UDP_PORT = UDP
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((UDP_IP, self.UDP_PORT))

##    def data_handler(self):
##        data, addr = self.sock.recvfrom(1024) #buffer size is 1024 bytes
##        data = str(data)
##        data = data[2:(len(data)-1)] #get rid of b's and ' '
##        self.counter += 1
##        print("sample no:", self.counter)
##
##        if(self.counter % 3 == 0):
##            self.ys.pop(0)
##            self.ys.append(float(data))

def animate(sensors):
    num_sensors = len(sensors)
    fig = plt.figure()
    axn = []
    IMUData = [0] * num_sensors

    for i in range(num_sensors):
        plotNum = 100*num_sensors + 10*1 + i + 1
        axn.append(fig.add_subplot(plotNum, xlim = (0,1), ylim = (-500, 500)))
        IMUData[i], = axn[i].plot([], [], lw = 1)

        plt.xlabel('Time (s)')
        plt.ylabel('Z-axis (deg)')
        if(sensors[i].UDP_PORT == 5005): plt.title('Sensor 1 Gyroscope Data')
        elif(sensors[i].UDP_PORT == 5010): plt.title('Sensor 2 Gyroscope Data')
        elif(sensors[i].UDP_PORT == 5011): plt.title('Sensor 3 Gyroscope Data')
        elif(sensors[i].UDP_PORT == 5025): plt.title('Sensor 4 Gyroscope Data')
        #fig.set_tight_layout(True) #fixes layout, adjusts axes and titles, takes too long though
        fig.tight_layout(pad=0.01)

    def ani_update(i):
        for idx, s in enumerate(sensors):
            # s.data_handler()
            data, addr = s.sock.recvfrom(1024) #buffer size is 1024 bytes
            data = str(data)
            data = data[2:(len(data)-1)] #get rid of b's and ' '
            s.counter += 1
            #print("sample no:", s.counter)

            if(s.counter % 4 == 0):
                s.ys.pop(0)
                s.ys.append(float(data))
                IMUData[idx].set_data(s.xs, s.ys)

    anim = animation.FuncAnimation(fig, ani_update, interval=1)
    plt.show()


available_sensors = [Sensor(5005), Sensor(5010), Sensor(5011), Sensor(5025)] # associated UDP ports for each sensor, respectively
sensors = []

print('Please indicate which sensors are in use:') # User input to indicate sensors for this trial
for i in range(4):
    print("Sensor", i+1, "- y or n: ")
    if(input()=='y'):
        sensors.append(available_sensors[i])

# sensors = [Sensor(5010), Sensor(5011)] # OR code in directly yourself

animate(sensors)
    
