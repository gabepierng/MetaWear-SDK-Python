# usage: sudo python3 data_fuser.py [mac1] [mac2] ... [mac(n)]
from __future__ import print_function
from ctypes import c_void_p, cast, POINTER
from mbientlab.metawear import MetaWear, libmetawear, parse_value, create_voidp
from mbientlab.metawear import cbindings as metacbindings
from mbientlab.warble import *
from time import sleep
from threading import Event
from sys import argv
import numpy as np
import os

import platform
import six

import socket

UDP_IP = '192.168.2.216'

# Available output data rates on the BMI160 gyro
MBL_MW_GYRO_BMI160_ODR_25Hz= 6
MBL_MW_GYRO_BMI160_ODR_50Hz= 7
MBL_MW_GYRO_BMI160_ODR_100Hz= 8
MBL_MW_GYRO_BMI160_ODR_200Hz= 9
MBL_MW_GYRO_BMI160_ODR_400Hz= 10
MBL_MW_GYRO_BMI160_ODR_800Hz= 11
MBL_MW_GYRO_BMI160_ODR_1600Hz= 12
MBL_MW_GYRO_BMI160_ODR_3200Hz= 13

# Available degrees per second ranges on the BMI160 gyro
MBL_MW_GYRO_BMI160_RANGE_2000dps= 0      # +/-2000 degrees per second
MBL_MW_GYRO_BMI160_RANGE_1000dps= 1      # +/-1000 degrees per second
MBL_MW_GYRO_BMI160_RANGE_500dps= 2       # +/-500 degrees per second
MBL_MW_GYRO_BMI160_RANGE_250dps= 3       # +/-250 degrees per second
MBL_MW_GYRO_BMI160_RANGE_125dps= 4       # +/-125 degrees per second

states = []
ports = [5005, 5010, 5011, 5025]
macIDs = sys.argv[1:]

class State:
    #initialize object for a connected device
    def __init__(self, device):
        self.device = device
        self.callback = metacbindings.FnVoid_VoidP_DataP(self.data_handler)
        self.processor = None
        self.sensor_data = []
        self.samples = 0
        
        #assign unique port from ports list based on device address's index in macIDs list
        self.UDP_PORT = [ports[i] for i, val in enumerate(macIDs) if val == device.address][0]
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) #UDP packet sending


    # once subscribe is called, data_handler linked to device data stream
    # data_handler continuously reads data buffer
    def data_handler(self, ctx, data):
        values = parse_value(data, n_elem = 2)
        
        # f.write('%.4f,%.4f,%.4f\n' % (values[1].x, values[1].y, values[1].z))
        # print("here:", self.samples)

        if( (self.samples == 0) or ((abs(data.contents.epoch - self.sensor_data[self.samples-1][0])) < 50 ) ): #add initial point, compare current epoch to previous epoch to ensure timestamp not erroneous
            self.sensor_data.append([data.contents.epoch, 0, values[0].x, 
                    values[0].y, values[0].z, values[1].x, values[1].y, values[1].z]) 
            
        else:
            epochEstimate = self.sensor_data[self.samples-1][0] + 10 # use previous epoch to estimate missing timestamp, add 10ms
            self.sensor_data.append([ epochEstimate, 0, values[0].x, 
                    values[0].y, values[0].z, values[1].x, values[1].y, values[1].z])
            
        self.samples += 1

        MESSAGE = str(values[1].z)
        
        self.sock.sendto(bytes(MESSAGE, "utf-8"), (UDP_IP, self.UDP_PORT))
       
        # print("time: %s\tacc: (%.4f,%.4f,%.4f),\tgyro; (%.4f,%.4f,%.4f)" 
        #     % (data.contents.epoch, values[0].x, values[0].y, values[0].z, values[1].x, values[1].y, values[1].z))


    def setup(self):
        # set BLE connection params (min connect interval, max interval,
        # latency, timeout) all in milliseconds
        libmetawear.mbl_mw_settings_set_connection_parameters(self.device.board, 7.5, 7.5, 0, 6000)
        # set BLE advertising strength (higher strength = higher power consumption
        # but theoretically better connectivity)
        libmetawear.mbl_mw_settings_set_tx_power(self.device.board, 4)
        sleep(1.5)

        e = Event()

        def processor_created(context, pointer):
            self.processor = pointer
            e.set()
        fn_wrapper = metacbindings.FnVoid_VoidP_VoidP(processor_created)

        # set accelerometer to 100Hz sampling rate and range to +/- 16 g's
        libmetawear.mbl_mw_acc_set_odr(self.device.board, 100.0) 
        libmetawear.mbl_mw_acc_set_range(self.device.board, 16.0)
        libmetawear.mbl_mw_acc_write_acceleration_config(self.device.board)
        
        # set gyro to 100Hz sampling rate and +/- 1000 deg/sec. 
        libmetawear.mbl_mw_gyro_bmi160_set_odr(self.device.board, MBL_MW_GYRO_BMI160_ODR_100Hz) 
        libmetawear.mbl_mw_gyro_bmi160_set_range(self.device.board, MBL_MW_GYRO_BMI160_RANGE_1000dps)
        libmetawear.mbl_mw_gyro_bmi160_write_config(self.device.board)

        # get pointers referencing the acc and gyro data signals
        acc = libmetawear.mbl_mw_acc_get_acceleration_data_signal(self.device.board)
        gyro = libmetawear.mbl_mw_gyro_bmi160_get_rotation_data_signal(self.device.board)

        signals = (c_void_p * 1)()
        signals[0] = gyro

        # //libmetawear.mbl_mw_dataprocessor_accounter_create(signals, None, fn_wrapper)
        
        # chain two processors together (fuser and accounter) to get timestamped acc+gyro data
        # create a fuser "data processor" which packages the acc and gyro signals into same packets before sending
        fuser = create_voidp(lambda fn: libmetawear.mbl_mw_dataprocessor_fuser_create(acc, signals, 1, None, fn), resource = "fuser", event = e)
        # accounter processor adds correct epoch data to BLE packets, necessary for timestamping stream-mode data
        accounter = create_voidp(lambda fn: libmetawear.mbl_mw_dataprocessor_accounter_create(fuser, None, fn), resource = "accounter", event = e)

        # //libmetawear.mbl_mw_datasignal_subscribe(self.processor, None, self.callback) 
        libmetawear.mbl_mw_datasignal_subscribe(accounter, None, self.callback)

    # begin sampling from gyro and acc signals
    def start(self):
        libmetawear.mbl_mw_gyro_bmi160_enable_rotation_sampling(self.device.board)
        libmetawear.mbl_mw_acc_enable_acceleration_sampling(self.device.board)

        libmetawear.mbl_mw_gyro_bmi160_start(self.device.board)
        libmetawear.mbl_mw_acc_start(self.device.board)
    
    def reconfigure(self):
        self.sensor_data = []
        self.samples = 0
    
    def pause(self):
        libmetawear.mbl_mw_acc_stop(self.device.board)
        libmetawear.mbl_mw_gyro_bmi160_stop(self.device.board)
        
        # libmetawear.mbl_mw_acc_disable_acceleration_sampling(self.device.board)

# iterates through MAC addresses provided as arguments in command line and attempts to connect to them
for i in range(len(argv) - 1):
    d = MetaWear(argv[i + 1])
    d.connect()
    print("Connected to " + d.address)
    states.append(State(d))

for s in states:
    print("Configuring %s" % (s.device.address))
    s.setup()
    
def streamData():
    input("Press enter to start streaming...")

    # start streaming for all connected devices in states, reconfigure to clear previous data
    for s in states:
        s.reconfigure()
        s.start()

    input("Press enter to stop streaming...")
    for s in states:
        s.pause()
    
    print("\nPausing streaming...")
    sleep(2)

    trial_name = input("\nPlease enter trial name: ")

    for idx, s in enumerate(states):
        s.sensor_data = np.asarray(s.sensor_data, dtype=np.float64)
        
        # time based off epoch, stored in ms.
        # remove offset and convert elapsed time to seconds
        timeStart = s.sensor_data[0,0]
        s.sensor_data[:,1] = [(x - timeStart) / 1000 for x in s.sensor_data[:,0]]

        filename = '../TestData/' + trial_name + '_' + str(idx) + '.csv'
        os.makedirs(os.path.dirname(filename), exist_ok=True)

        # write each sensor's data to a separate .csv file
        with open(filename, 'w') as f:
            f.write('epoch, elapseds, xacc, yacc, zacc, xgyro, ygyro, zgyro\n')
            for row in s.sensor_data:
                f.write('%d,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n' % (row[0], row[1], row[2], row[3], row[4], row[5], row[6], row[7]))

        print("Data saved to \'" + filename + "\'")

    print("\nTotal Samples Received")
    for s in states:
        print("%s -> %d" % (s.device.address, s.samples))

def reset():
    print("Resetting devices")
    events = []
    for s in states:
        e = Event()
        events.append(e)

        s.device.on_disconnect = lambda s: e.set()
        libmetawear.mbl_mw_debug_reset(s.device.board)

    for e in events:
        e.wait()


while True:
    streamData()
    if(input("\nWould you like to quit? y or n: \n")=='y'):
        reset()
        break


