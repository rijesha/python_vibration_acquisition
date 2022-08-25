from board import SDA,SCL
from custom_imu_mpu6050 import MPU6050
import busio
import os
import threading

import time

class mpu6050_logger:
    def __init__(self, filename):
        self.i2c = busio.I2C(SCL,SDA)
        self.IMU = MPU6050(self.i2c)
        self.shutdown = False

        print ("Identification 0x{:X} ".format(self.IMU.whoami))

        outfile = filename + '.csv'
        self.chunksize = 10
        self.data = []
        self.csvfile = open(outfile, 'w+')
        self.csvfile.writelines("{:5s} , {:5s} , {:5s} , {:5s} , \n".format("Time", "Xa","Ya","Za"))
        self.runner_th = threading.Thread(target=self.runner)
        self.runner_th.start()
        
    def stop(self):
        self.shutdown = True
        self.runner_th.join()
        
    def runner(self):
        while not self.shutdown: 
            self.data.append(self.IMU.get_accel_data_fast(g=True))
            
            if len(self.data) == self.chunksize*3:
                y=0
                for y in range ( 0, len(self.data) , 3):
                    self.csvfile.writelines(" {0:.5f} , {d1[x]} , {d1[y]} , {d1[z]}, \n".format(time.time(), d1=self.data[y+1]) )
                self.data.clear()
            time.sleep(1.0/1000)
