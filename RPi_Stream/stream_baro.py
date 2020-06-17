# usage: sudo python3 data_fuser.py [mac1] [mac2] ... [mac(n)]
# gyro values are still wack, lots of noise at rest and delayed/laggy plotting, will look into it 
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

UDP_IP = '192.168.2.18'

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

MBL_MW_BARO_BMP280_STANDBY_TIME_0_5ms= 0    # default, 10ms
MBL_MW_BARO_BMP280_STANDBY_TIME_62_5ms= 1
MBL_MW_BARO_BMP280_STANDBY_TIME_125ms= 2
MBL_MW_BARO_BMP280_STANDBY_TIME_250ms= 3
MBL_MW_BARO_BMP280_STANDBY_TIME_500ms= 4
MBL_MW_BARO_BMP280_STANDBY_TIME_1000ms= 5
MBL_MW_BARO_BMP280_STANDBY_TIME_2000ms= 6
MBL_MW_BARO_BMP280_STANDBY_TIME_4000ms= 7

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
        self.counter = 0
        self.offset = 0
        self.pcount = 0 #for storing previous count value

        # assign unique port from ports list based on device address's index in macIDs list
        self.UDP_PORT = [ports[i] for i, val in enumerate(macIDs) if val == device.address][0]
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) #UDP packet sending

    # once subscribe is called, data_handler linked to device data stream
    # data_handler continuously reads data buffer
    def data_handler(self, ctx, data):
        values = parse_value(data, n_elem = 3)
        
        count = cast(data.contents.extra, POINTER(c_uint8)).contents.value 
        # count = 8 bit, only goes to 255 
        # comparing to previous count value to taking into account that packet loss can occur at count = 0 or 255
        if(count - self.pcount < 0):      
            self.offset += 256
        self.counter = count + self.offset

        print(values, count, self.counter, self.samples)

        if((self.samples == 0) or (8 < (data.contents.epoch - self.sensor_data[self.samples-1][0]) < 12 )): #add initial point, compare current epoch to previous epoch to ensure timestamp not erroneous
            self.sensor_data.append([data.contents.epoch, 0, self.counter, values[0].x, 
                                        values[0].y, values[0].z, values[1].x, values[1].y, values[1].z, values[2]]) 
                
        else:
            epochEstimate = self.sensor_data[self.samples-1][0] + 10*(self.counter - self.sensor_data[self.samples-1][2]) # use previous epoch to estimate missing timestamp, add 10ms * the difference of counted packets
            self.sensor_data.append([ epochEstimate, 0, self.counter, values[0].x, 
                                        values[0].y, values[0].z, values[1].x, values[1].y, values[1].z, values[2]])
            
        self.samples += 1

        MESSAGE = str(values[1].z)

        # sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) #UDP packet sending
        self.sock.sendto(bytes(MESSAGE, "utf-8"), (UDP_IP, self.UDP_PORT))
        
        self.pcount = count 

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
        libmetawear.mbl_mw_acc_set_odr(s.device.board, 100.0) 
        libmetawear.mbl_mw_acc_set_range(s.device.board, 16.0)
        libmetawear.mbl_mw_acc_write_acceleration_config(s.device.board)
        
        # set gyro to 100Hz sampling rate and +/- 1000 deg/sec. 
        libmetawear.mbl_mw_gyro_bmi160_set_odr(s.device.board, MBL_MW_GYRO_BMI160_ODR_100Hz) 
        libmetawear.mbl_mw_gyro_bmi160_set_range(s.device.board, MBL_MW_GYRO_BMI160_RANGE_1000dps)
        libmetawear.mbl_mw_gyro_bmi160_write_config(s.device.board)

        # set baro to 10ms (= 100Hz sampling rate)
        #libmetawear.mbl_mw_baro_bosch_set_standby_time(s.device.board, 10.0)
        libmetawear.mbl_mw_baro_bmp280_set_standby_time(s.device.board, MBL_MW_BARO_BMP280_STANDBY_TIME_125ms)
        libmetawear.mbl_mw_baro_bosch_write_config(s.device.board)

        # get pointers referencing the acc and gyro data signals
        acc = libmetawear.mbl_mw_acc_get_acceleration_data_signal(self.device.board)
        gyro = libmetawear.mbl_mw_gyro_bmi160_get_rotation_data_signal(self.device.board)
        baro = libmetawear.mbl_mw_baro_bosch_get_pressure_data_signal(self.device.board)

        signals = (c_void_p * 2)()
        signals[0] = gyro
        signals[1] = baro
     
        # chain two processors together (fuser and accounter) to get timestamped acc+gyro data
        # create a fuser "data processor" which packages the acc and gyro signals into same packets before sending
        fuser = create_voidp(lambda fn: libmetawear.mbl_mw_dataprocessor_fuser_create(acc, signals, 2, None, fn), resource = "fuser", event = e)
        
        # accounter processor adds correct epoch data to BLE packets, necessary for timestamping stream-mode data
        accounter = create_voidp(lambda fn: libmetawear.mbl_mw_dataprocessor_accounter_create_count(fuser, None, fn), resource = "accounter", event = e)

        # //libmetawear.mbl_mw_datasignal_subscribe(self.processor, None, self.callback) 
        libmetawear.mbl_mw_datasignal_subscribe(accounter, None, self.callback)


    # begin sampling from gyro and acc signals
    def start(self):

        libmetawear.mbl_mw_gyro_bmi160_enable_rotation_sampling(self.device.board)
        libmetawear.mbl_mw_acc_enable_acceleration_sampling(self.device.board)

        libmetawear.mbl_mw_gyro_bmi160_start(self.device.board)
        libmetawear.mbl_mw_acc_start(self.device.board)
        libmetawear.mbl_mw_baro_bosch_start(self.device.board)

# iterates through MAC addresses provided as arguments in command line 
# and attempts to connect to them
for i in range(len(argv) - 1):
    d = MetaWear(argv[i + 1])
    d.connect()
    print("Connected to " + d.address)
    states.append(State(d))

for s in states:
    print("Configuring %s" % (s.device.address))
    s.setup()

sleep(0.5)
input("Press enter to start streaming...")

# start streaming for all connected devices in states
for s in states:
    s.start()

#sleep(20.0)
input("Press enter to stop streaming...")

print("Resetting devices\n")
events = []
for s in states:
    e = Event()
    events.append(e)

    s.device.on_disconnect = lambda s: e.set()
    libmetawear.mbl_mw_debug_reset(s.device.board)

for e in events:
    e.wait()


trial_name = input("Please enter trial name: ")

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
        f.write('epoch, elapseds, count, xacc, yacc, zacc, xgyro, ygyro, zgyro, pressure\n')
        for row in s.sensor_data:
            f.write('%d,%.3f,%d, %.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f\n' % (row[0], row[1], row[2], row[3], row[4], row[5], row[6], row[7], row[8], row[9]))

    print("Data saved to \'" + filename + "\'\n")

print("Total Samples Received")
for s in states:
    print("%s -> %d" % (s.device.address, s.samples))

