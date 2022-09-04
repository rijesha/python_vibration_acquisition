
import os
import threading
import numpy as np
import time
from enum import Enum 

end_position_z = .4
staging_position_z = 1.5

class ControlState(Enum):
    WAITING_FOR_INIT = 1
    CENTERING_ON_TARGET = 2
    MAKING_CONTACT = 3
    ACQUIRING_SAMPLE = 4
    LEAVING_TARGET = 5
    FINISHED = 6

class VisionControl:
    last_mode = 0
    time_start_acquring = time.time()
    data = []

    def __init__(self, got_to_position_func, filename):
        self.shutdown = False
        self.got_to_position_func = got_to_position_func

        outfile = filename + '.csv'
        self.chunksize = 10
        self.data = []
        self.csvfile = open(outfile, 'w+')
        self.csvfile.writelines("{:5s} , {:5s} , {:5s} , {:5s} , \n".format("Time", "Xa","Ya","Za"))
        
        self.last_tar_update_time = 0
        self.last_yaw_update_time = 0
        self.pos = {'right':0, 'down':0, 'forward':0}
        self.yaw = 0
        
        self.control_state = ControlState.WAITING_FOR_INIT
        
        self.runner_th = threading.Thread(target=self.timeout)
        self.runner_th.start()
        
    def stop(self):
        self.shutdown = True
        self.runner_th.join()

    def update_mavlink_msg(self, msg):
        if msg.get_type() == "ATTITUDE":
            self.last_yaw_update_time = time.time()
            self.yaw = msg.yaw
        elif msg.get_type() == "HEARTBEAT":
            self.update_mode(msg.custom_mode)

    def update_target_position(self, x, y, z):
        self.last_tar_update_time = time.time()
        #self.pos = {'right':x, 'down':y, 'forward':z}
        self.pos['right'] = x
        self.pos['down'] = y
        self.pos['forward'] = z
        self.runner()

    def update_mode(self, mode):
        if (time.time() - self.last_tar_update_time) < .5 and self.last_mode != 4 and mode == 4:
            self.control_state = ControlState.CENTERING_ON_TARGET
            print("Changing Mode to Centring On Target")
        elif mode != 4:
            if mode == 4:
                print("Resetting Control To Wait")
            self.control_state = ControlState.WAITING_FOR_INIT
            self.send_desired_position(0,0,0, self.yaw)
        
        print(mode)
        self.last_mode = mode

    def send_desired_position(self, forward, right, down, yaw):
        print(forward, right, down, yaw)
        #self.data.append[{'right': self.pos['right'], 'down': self.pos['down'], 'forward': self.pos['forward'], 'yaw': self.yaw, 'des_forward': forward, 'des_right': right, 'des_down': down, 'des_yaw': yaw, 'des_mode' : self.last_mode}]
        #print("sending data")
        #self.got_to_position_func(forward, right, down, yaw)

    def waiting_for_init(self):
        self.send_desired_position(0,0,0, self.yaw)
        return False

    def centring_on_target(self):
        self.send_desired_position(self.pos['forward'] - staging_position_z,self.pos['right'],self.pos['down'],self.yaw)
        if abs(self.pos['right']) < 0.05 and abs(self.pos['down']) < 0.05:
            return True
        return False

    def making_contact(self):
        self.send_desired_position(self.pos['forward'] - end_position_z,self.pos['right'],self.pos['down'],self.yaw)
        if self.pos['forward'] < (end_position_z - 0.05):
            self.time_start_acquring = time.time()
            return True
        return False

    def acquiring_sample(self):
        self.send_desired_position(self.pos['forward'] - end_position_z,self.pos['right'],self.pos['down'],self.yaw)
        if (time.time() - self.time_start_acquring) > 4:
            return True 
        return False

    def leaving_target(self):
        self.send_desired_position(self.pos['forward'] - staging_position_z,self.pos['right'],self.pos['down'],self.yaw)
        if self.pos['forward'] > (staging_position_z - 0.05):
            return True
        return False
    
    def timeout(self):
        while not self.shutdown:
            
            #if len(self.data) == self.chunksize*3:
            #    y=0
            #    for y in range ( 0, len(self.data) , 3):
            #        self.csvfile.writelines(" {0:.5f} , {d1[right]} , {d1[down]} , {d1[forward]}, {d1[yaw]} , {d1[des_right]} , {d1[des_down]} , {d1[des_forward]}, {d1[des_yaw]}, {d1[mode]},  \n".format(time.time(), d1=self.data[y+1]) )
            #    self.data.clear()
            
            if (time.time() - self.last_tar_update_time) > .5:
                if self.control_state != ControlState.WAITING_FOR_INIT:
                    print("[LOST TARGET] Resetting state")
                self.control_state = ControlState.WAITING_FOR_INIT
                self.runner()
            time.sleep(1.0/20)


    def runner(self):
        if self.control_state is ControlState.WAITING_FOR_INIT:
            print("[LOOP] waiting for init")
            if self.waiting_for_init():
                self.control_state = ControlState.CENTERING_ON_TARGET
            else:
                pass
        elif self.control_state == ControlState.CENTERING_ON_TARGET:
            print("[LOOP] centring_state")
            if self.centring_on_target():
                self.control_state = ControlState.MAKING_CONTACT
            else:
                pass
        elif self.control_state == ControlState.MAKING_CONTACT:
            print("[LOOP] making contact")
            if self.making_contact():
                self.control_state = ControlState.ACQUIRING_SAMPLE
            else:
                pass
        elif self.control_state == ControlState.ACQUIRING_SAMPLE:
            print("[LOOP] sampling")
            if self.acquiring_sample():
                self.control_state = ControlState.LEAVING_TARGET
            else:
                pass
        elif self.control_state == ControlState.LEAVING_TARGET:
            print("[LOOP] leaving")
            if self.leaving_target():
                self.control_state = ControlState.WAITING_FOR_INIT
            else:
                pass

    #def init_target_kalman_filter(self):
    #    # Process Noise
    #    q = np.eye(6)
    #    q[0][0] = 0.01
    #    q[1][1] = 0.01
    #    q[2][2] = 0.01
    #    q[3][3] = 0.25
    #    q[4][4] = 0.25
    #    q[5][5] = 0.25
#
    #    # create measurement noise covariance matrices
    #    r_imu = np.zeros([2, 2])
    #    r_imu[0][0] = 0.01
    #    r_imu[1][1] = 0.03
#
    #    r_compass = np.zeros([1, 1])
    #    r_compass[0][0] = 0.02
#
    #    r_encoder = np.zeros([1, 1])
    #    r_encoder[0][0] = 0.001
#
    #    # pass all the parameters into the UKF!
    #    # number of state variables, process noise, initial state, initial coariance, three tuning paramters, and the iterate function
    #    state_estimator = UKF(6, q, np.zeros(6), 0.0001*np.eye(6), 0.04, 0.0, 2.0, self.iterate_x)
#
    #def iterate_x(x_in, timestep, inputs):
    #    '''this function is based on the x_dot and can be nonlinear as needed'''
    #    ret = np.zeros(len(x_in))
    #    ret[0] = x_in[0] + timestep * x_in[3]
    #    ret[1] = x_in[1] + timestep * x_in[4]
    #    ret[2] = x_in[2] + timestep * x_in[5]
    #    ret[3] = x_in[3]
    #    ret[4] = x_in[4]
    #    ret[5] = x_in[5]
    #    return ret
