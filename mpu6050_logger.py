from board import SDA,SCL
from custom_imu_mpu6050 import MPU6050
import busio
import os
import threading
import queue

import time

class mpu6050_logger:
    def __init__(self, filename):
        self.q = queue.Queue()
        self.outfile = filename + '.csv'
        self.csvfile = open(self.outfile, 'w+')
        self.csvfile.writelines("{:5s} , {:5s} , {:5s} , {:5s} , \n".format("Time", "Xa","Ya","Za"))

        self.shutdown = False
        self.runner_th = threading.Thread(target=self.runner)
        self.runner_th.start()
        
    def stop(self):
        self.shutdown = True
        self.runner_th.join()
        self.saver_th.join()
    
    def saver(self):
        timed_out = False
        almost_timed_out = False
        while not self.shutdown and timed_out:
            try:
                data = q.get(timeout = 200)
                almost_timed_out = False
                self.csvfile.writelines(" {0:.5f} , {0:.7f}  , {0:.7f}  , {0:.7f} , \n".format(time.time(), data.x, data.y, data.z) )
            except:
                time.sleep(500)
                if almost_timed_out:
                    timed_out = True
                almost_timed_out = True
                
        
    def runner(self):
        self.i2c = busio.I2C(SCL,SDA)
        self.IMU = MPU6050(self.i2c)
        
        print ("Identification 0x{:X} ".format(self.IMU.whoami))

        self.chunksize = 10
        while not self.shutdown:
            q.put(self.IMU.get_accel_data_fast(g=True))
            time.sleep(1.0/1000)
