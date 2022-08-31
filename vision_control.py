
import os
import threading
import numpy as np
import time

end_position = .4

class ControlState(Enum):
    WAITING_FOR_INIT = 1
    CENTERING_ON_TARGET = 2
    MAKING_CONTACT = 3
    ACQUIRING_SAMPLE = 4
    LEAVING_TARGET = 5
    FINISHED = 6

class VisionControl:
    control_state = ControlState.WAITING_FOR_INIT
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
        self.pos = [0,0,0]
        self.yaw = 0
        
        self.runner_th = threading.Thread(target=self.timeout)
        self.runner_th.start()
        
    def stop(self):
        self.shutdown = True
        self.runner_th.join()

    def update_mavlink_msg(self, msg):
        if msg.get_type() == "ATTITDUE":
            self.last_yaw_update_time = time.time()
            self.yaw = msg.yaw
        elif msg.get_type() == "HEARTBEAT":
            self.update_mode(msg.custom_mode)        

    def update_target_position(self, x, y, z):
        self.last_tar_update_time = time.time()
        self.pos = {'x':x, 'y':y, 'z':z}
        self.runner()

    def update_mode(self, mode):
        if (time.time() - self.last_tar_update_time) < .5 and self.last_mode is not 4 and mode is 4:
            self.control_state = ControlState.CENTERING_ON_TARGET
        elif mode is not 4:
            self.control_state = ControlState.WAITING_FOR_INIT

        self.last_mode = mode

    def send_desired_position(self, forward, right, down, yaw):
        self.data.append[{'x': self.pos.x, 'y': self.pos.y, 'z': self.pos.z, 'yaw': self.yaw, 'des_x': forward, 'des_y': right, 'des_z': down, 'des_yaw': yaw, 'des_mode' : self.last_mode}]
        self.got_to_position_func(forward, right, down, yaw)

    def waiting_for_init(self):
        return False

    def centring_on_target(self):
        self.send_desired_position(-self.pos[0],y,z,yaw)
        if self.pos[0] < 0.05 and self.pos[1] < 0.05:
            return True
        return False

    def making_contact(self):
        self.send_desired_position(-self.pos[0],y,z,yaw)
        if self.pos[2] < end_position:
            self.time_start_acquring = time.time()
            return True
        return False

    def acquiring_sample(self):
        self.send_desired_position(-self.pos[0],y,z,yaw)
        if time.time() - self.time_start_acquring > 4:
            return True 
        return False

    def leaving_target(self):
        self.send_desired_position(-self.pos[0],y,z,yaw)
        if self.pos[2] > 1.2:
            return True
        return False
    
    def timeout(self):
        while not self.shutdown:
            if len(self.data) == self.chunksize*3:
                y=0
                for y in range ( 0, len(self.data) , 3):
                    self.csvfile.writelines(" {0:.5f} , {d1[x]} , {d1[y]} , {d1[z]}, {d1[yaw]} , {d1[des_x]} , {d1[des_y]} , {d1[des_z]}, {d1[des_yaw]}, {d1[mode]},  \n".format(time.time(), d1=self.data[y+1]) )
                self.data.clear()
            if (time.time() - self.last_tar_update_time) > .5:
                self.control_state = self.waiting_for_init
            time.sleep(1.0/20)


    def runner(self):
        if self.control_state == ControlState.WAITING_FOR_INIT:
            if self.waiting_for_init():
                self.control_state = ControlState.CENTERING_ON_TARGET
            else:
                pass
        elif self.control_state == ControlState.CENTERING_ON_TARGET:
            if self.centring_on_target():
                self.control_state = ControlState.MAKING_CONTACT
            else:
                pass
        elif self.control_state == ControlState.MAKING_CONTACT:
            if self.making_contact():
                self.control_state = ControlState.ACQUIRING_SAMPLE
            else:
                pass
        elif self.control_state == ControlState.ACQUIRING_SAMPLE:
            if self.acquiring_sample():
                self.control_state = ControlState.LEAVING_TARGET
            else:
                pass
        elif self.control_state == ControlState.LEAVING_TARGET:
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
