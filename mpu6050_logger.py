from board import SDA,SCL
from custom_imu_mpu6050 import MPU6050
import busio
import os
import threading
import queue
import traceback

import time

class mpu6050_logger:
    def __init__(self, filename):
        self.q = queue.Queue(10000)
        self.outfile = filename + '.csv'
        self.csvfile = open(self.outfile, 'w+')
        self.csvfile.writelines("{:5s} , {:5s} , {:5s} , {:5s} , \n".format("Time", "Xa","Ya","Za"))

        self.shutdown = False
        self.runner_th = threading.Thread(target=self.runner)
        self.saver_th = threading.Thread(target=self.saver)
        self.saver_th.start()
        self.runner_th.start()
        
    def stop(self):
        self.shutdown = True
        self.runner_th.join()
        self.saver_th.join()
        print("closed all vibration threads")
    
    def saver(self):
        while not self.shutdown:
            try:
                data = self.q.get(timeout = .2)
                self.csvfile.writelines(" {0:.5f} , {1:.7f}  , {2:.7f}  , {3:.7f} , \n".format(data['time'], data['x'], data['y'], data['z']) )
                self.q.task_done()
            except Exception as e:
                pass             
        
    def runner(self):
        self.i2c = busio.I2C(SCL,SDA)
        self.IMU = MPU6050(self.i2c)
        
        print ("Identification 0x{:X} ".format(self.IMU.whoami))

        self.chunksize = 10
        while not self.shutdown:
            data = self.IMU.get_accel_data_fast(g=True)
            data['time'] = time.time()
            self.q.put(data)
            time.sleep(1.0/1000)
