# usage: python stream_acc.py [mac1] [mac2] ... [mac(n)]
from __future__ import print_function
from ctypes import c_void_p, cast, POINTER
from mbientlab.metawear import MetaWear, libmetawear, parse_value, create_voidp
from mbientlab.metawear import cbindings as metacbindings
from mbientlab.metawear.cbindings import *
from mbientlab.warble import *
from time import sleep
from threading import Event
from sys import argv
import numpy as np
import os
import csv
import platform
import six

if sys.version_info[0] == 2:
    range = xrange

class State:
    def __init__(self, device):
        self.device = device
        self.samples = 0
        self.processor = None
        
        self.stream_gyro = []
        self.log_acc_gyro = []
        self.log_euler = []
        self.log_baro = []
        self.log_signals = []

        self.callback = metacbindings.FnVoid_VoidP_DataP(self.stream_data_handler)

    def stream_data_handler(self, ctx, data):
        values = parse_value(data)
        #~ print("time: %s" % (data.contents.epoch), values)
        self.stream_gyro.append([data.contents.epoch, 0, values.x, values.y, values.z])
        self.samples+= 1
        
    def log_acc_gyro_handler(self, ctx, p):
        values = parse_value(p, n_elem = 2)
        self.log_acc_gyro.append([p.contents.epoch, 0, values[0].x, values[0].y, values[0].z, values[1].x, values[1].y, values[1].z]) 

    def log_euler_handler(self, ctx, p):
        values = parse_value(p)
        self.log_euler.append([p.contents.epoch, 0, values.heading, values.pitch, values.roll, values.yaw]) 
        
    def log_baro_handler(self, ctx, p):
        values = parse_value(p)
        self.log_baro.append([p.contents.epoch, 0, values]) 
    
    def setup(self):
        libmetawear.mbl_mw_settings_set_connection_parameters(self.device.board, 7.5, 7.5, 0, 6000)
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
        libmetawear.mbl_mw_gyro_bmi160_set_odr(self.device.board, GyroBmi160Odr._100Hz) 
        libmetawear.mbl_mw_gyro_bmi160_set_range(self.device.board, GyroBmi160Range._2000dps)
        libmetawear.mbl_mw_gyro_bmi160_write_config(self.device.board)
        
        # set sensor_fusion settings 
        libmetawear.mbl_mw_sensor_fusion_set_mode(self.device.board, SensorFusionMode.NDOF);
        libmetawear.mbl_mw_sensor_fusion_write_config(self.device.board)
    
        #~ # set baro to 10ms (= 100Hz sampling rate)
        libmetawear.mbl_mw_baro_bosch_set_standby_time(self.device.board, 10.0)
        libmetawear.mbl_mw_baro_bmp280_set_standby_time(self.device.board, BaroBmp280StandbyTime._62_5ms)
        libmetawear.mbl_mw_baro_bosch_write_config(self.device.board)
    
        # get pointers referencing the acc and gyro data signals
        acc = libmetawear.mbl_mw_acc_get_acceleration_data_signal(self.device.board)
        gyro = libmetawear.mbl_mw_gyro_bmi160_get_rotation_data_signal(self.device.board)
        euler = libmetawear.mbl_mw_sensor_fusion_get_data_signal(self.device.board, SensorFusionData.EULER_ANGLE)
        baro = libmetawear.mbl_mw_baro_bosch_get_pressure_data_signal(self.device.board)
            
        signals = (c_void_p * 1)()
        signals[0] = gyro
     
        # chain two processors together (fuser and accounter) to get timestamped acc+gyro data
        # create a fuser "data processor" which packages the acc and gyro signals into same packets before sending
        fuser = create_voidp(lambda fn: libmetawear.mbl_mw_dataprocessor_fuser_create(acc, signals, 1, None, fn), resource = "fuser", event = e)
        
        # accounter processor adds correct epoch data to BLE packets, necessary for timestamping stream-mode data
        accounter = create_voidp(lambda fn: libmetawear.mbl_mw_dataprocessor_accounter_create(gyro, None, fn), resource = "accounter", event = e)
    
        libmetawear.mbl_mw_datasignal_subscribe(accounter, None, self.callback)
        
        logger_acc_gyro = create_voidp(lambda fn: libmetawear.mbl_mw_datasignal_log(fuser, None, fn), resource = "acc_gyro_logger")
        logger_euler = create_voidp(lambda fn: libmetawear.mbl_mw_datasignal_log(euler, None, fn), resource = "euler_logger")
        logger_baro = create_voidp(lambda fn: libmetawear.mbl_mw_datasignal_log(baro, None, fn), resource = "baro_logger")
        self.log_signals = [logger_acc_gyro, logger_euler, logger_baro]

    def start(self):
        libmetawear.mbl_mw_logging_start(self.device.board, 0)
        
        libmetawear.mbl_mw_acc_enable_acceleration_sampling(self.device.board)
        libmetawear.mbl_mw_gyro_bmi160_enable_rotation_sampling(self.device.board)
        libmetawear.mbl_mw_sensor_fusion_enable_data(self.device.board, SensorFusionData.EULER_ANGLE)
        
        libmetawear.mbl_mw_acc_start(self.device.board)
        libmetawear.mbl_mw_gyro_bmi160_start(self.device.board)
        libmetawear.mbl_mw_sensor_fusion_start(self.device.board)
        libmetawear.mbl_mw_baro_bosch_start(self.device.board)
        
        print("Logging data for 15s")
        
    def stop(self):
        libmetawear.mbl_mw_acc_stop(self.device.board)
        libmetawear.mbl_mw_acc_disable_acceleration_sampling(self.device.board)

        libmetawear.mbl_mw_gyro_bmi160_stop(self.device.board)
        libmetawear.mbl_mw_gyro_bmi160_disable_rotation_sampling(self.device.board)
        
        libmetawear.mbl_mw_sensor_fusion_stop(self.device.board);
        signal = libmetawear.mbl_mw_sensor_fusion_get_data_signal(self.device.board, SensorFusionData.EULER_ANGLE);
        libmetawear.mbl_mw_datasignal_unsubscribe(signal)
     
        libmetawear.mbl_mw_baro_bosch_stop(self.device.board)
        
        libmetawear.mbl_mw_logging_stop(self.device.board)

    def download(self):
        print("Downloading data")

        e = Event()
        def progress_update_handler(context, entries_left, total_entries):
            if (entries_left == 0):
                e.set()
        
        fn_wrapper = FnVoid_VoidP_UInt_UInt(progress_update_handler)
        download_handler = LogDownloadHandler(context = None, \
            received_progress_update = fn_wrapper, \
            received_unknown_entry = cast(None, FnVoid_VoidP_UByte_Long_UByteP_UByte), \
            received_unhandled_entry = cast(None, FnVoid_VoidP_DataP))
        
        log_callback = [FnVoid_VoidP_DataP(self.log_acc_gyro_handler), FnVoid_VoidP_DataP(self.log_euler_handler), FnVoid_VoidP_DataP(self.log_baro_handler)]
        
        for i, signal in enumerate(self.log_signals):
            libmetawear.mbl_mw_logger_subscribe(signal, None, log_callback[i])
            libmetawear.mbl_mw_logging_download(self.device.board, 0, byref(download_handler))   
            
        e.wait()
        
states = []
for i in range(len(sys.argv) - 1):
    d = MetaWear(sys.argv[i + 1])
    d.connect()
    print("Connected to " + d.address)
    states.append(State(d))

for s in states:
    print("Configuring %s" % (s.device.address))
    s.setup()

input("Press enter to start streaming...")
for s in states:
    s.start()

input("Press enter to stop streaming...")

for s in states:
    s.stop()
    
for s in states:
    s.download()
    
trial_name = input("\nPlease enter trial name: ")

for idx, s in enumerate(states):
    sets = [s.stream_gyro, s.log_acc_gyro, s.log_euler, s.log_baro]
    set_name = ['stream_gyro', 'log_acc_gyro', 'log_euler', 'log_baro']
            
    for i, array in enumerate(sets): 
        # time based off epoch, stored in ms.
        # remove offset and convert elapsed time to seconds
        array = np.asarray(array, dtype=np.float64)
        timeStart = array[0,0]
        array[:,1] = [(x - timeStart) / 1000 for x in array[:,0]]

        filename = '../TestData/' + trial_name + '_' + set_name[i] + '_' + str(idx) + '.csv'
        os.makedirs(os.path.dirname(filename), exist_ok=True)

        # write each sensor's data to a separate .csv file
        with open(filename, 'w') as f:
            if i == 0:
                f.write('epoch, elapseds, xgyro, ygyro, zgyro\n')
                for row in array:
                    f.write('%d,%.3f,%.4f,%.4f,%.4f\n' % (row[0], row[1], row[2], row[3], row[4]))
            elif i == 1:
                f.write('epoch, elapseds, xacc, yacc, zacc, xgyro, ygyro, zgyro\n') 
                for row in array:
                    f.write('%d,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n' % (row[0], row[1], row[2], row[3], row[4], row[5], row[6], row[7]))
            elif i == 2:
                f.write('epoch, elapseds, heading, pitch, roll, yaw\n')
                for row in array:
                    f.write('%d,%.3f,%.4f,%.4f,%.4f,%.4f\n' % (row[0], row[1], row[2], row[3], row[4], row[5]))
            else:
                f.write('epoch, elapseds, pressure\n')
                for row in array:
                    f.write('%d,%.3f,%.4f\n' % (row[0], row[1], row[2]))

        print("Data saved to \'" + filename + "\'", len(array))
            
print("\nTotal Samples Received")
for s in states:
    print("%s -> %d" % (s.device.address, s.samples))
    
print("Resetting devices")
events = []
for s in states:
    e = Event()
    events.append(e)

    s.device.on_disconnect = lambda s: e.set()
    libmetawear.mbl_mw_debug_reset(s.device.board)

for e in events:
    e.wait()
    


