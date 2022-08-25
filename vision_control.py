
import os
import threading

import time

class vision_control:
    def __init__(self, drone_connection, filename):
        self.shutdown = False
        self.drone_connection = drone_connection

        outfile = filename + '.csv'
        self.chunksize = 10
        self.data = []
        self.csvfile = open(outfile, 'w+')
        self.csvfile.writelines("{:5s} , {:5s} , {:5s} , {:5s} , \n".format("Time", "Xa","Ya","Za"))
        
        self.last_tar_update_time = 0
        self.last_yaw_update_time = 0
        self.pos = [0,0,0]
        self.yaw = 0
        
        self.runner_th = threading.Thread(target=self.runner)
        self.runner_th.start()
        
    def stop(self):
        self.shutdown = True
        self.runner_th.join()

    def update_drone_attitude(self, msg):
        if msg.get_type() == "ATTITDUE":
            self.last_yaw_update_time = time.time()
            self.yaw = msg.yaw
            
    def update_target_position(self, x, y, z):
        last_tar_update_time = time.time()
        self.pos = [x, y, z]
        
    def runner(self):
        while not self.shutdown: 
            
            if len(self.data) == self.chunksize*3:
                y=0
                for y in range ( 0, len(self.data) , 3):
                    self.csvfile.writelines(" {0:.5f} , {d1[x]} , {d1[y]} , {d1[z]}, \n".format(time.time(), d1=self.data[y+1]) )
                self.data.clear()
            time.sleep(1.0/1000)
