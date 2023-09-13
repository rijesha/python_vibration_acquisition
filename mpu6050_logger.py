from board import SDA,SCL
from custom_imu_mpu6050 import MPU6050
import busio
import os
import threading
import numpy as np
import csv
import time

class mpu6050_logger:
    def __init__(self, filename):
        self.outfile = filename + '.csv'
        self.csvfile = open(self.outfile, 'w+')
        self.csvWriter = csv.writer(self.csvfile,delimiter=',')

        self.shutdown = False
        self.runner_th = threading.Thread(target=self.runner)
        self.saver_th = threading.Thread(target=self.saver)
        self.saver_th.start()
        self.runner_th.start()
        
    def stop(self):
        self.shutdown = True
        self.runner_th.join()
        self.saver_th.join()
        self.csvfile.close()
        print("closed all vibration threads")
    
    def saver(self):
        pass          
        
    def runner(self):
        self.i2c = busio.I2C(SCL,SDA)
        self.IMU = MPU6050(self.i2c)
        
        print ("Identification 0x{:X} ".format(self.IMU.whoami))
        time_start = time.time()
        data = np.empty((2000, 4))
        count = 0
        while not self.shutdown:
            data[count] = self.IMU.get_accel_data_fast(g=True)
            time.sleep(1.0/1500)
            count = count + 1

            if (time.time() - time_start) > 1:
                self.csvWriter.writerows(data)
                time.sleep(1)
                data = np.empty((2000, 4))
                count = 0
                time_start = time.time()