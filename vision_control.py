
import os
import threading
import numpy as np
import time
from enum import Enum 
import queue


end_position_z = .5
end_position_down = .1
staging_position_z = 1.5
finished_z = 2
final_yaw = 0

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
    data = queue.Queue()
    last_control_state = ControlState.WAITING_FOR_INIT
    control_state = ControlState.WAITING_FOR_INIT
    lock = threading.Lock()

    def __init__(self, got_to_position_func, filename):
        self.shutdown = False
        self.got_to_position_func = got_to_position_func

        outfile = filename + '.csv'
        self.chunksize = 10
        self.csvfile = open(outfile, 'w+')
        self.csvfile.writelines("time,right,down,forward,yaw,des_r,des_d,des_f,des_y, mode, drone_mode \n".format("Time", "Xa","Ya","Za"))
        
        self.last_tar_update_time = 0
        self.last_yaw_update_time = 0
        self.pos = {'time':0, 'right':0, 'down':0, 'forward':0, 'mode':0, 'drone_mode':0, 'des_r':0, 'des_d':0, 'des_f':0, 'yaw':0, 'des_y': final_yaw}
        self.yaw = 0
        
        self.writer_th = threading.Thread(target=self.writer)
        self.runner_th = threading.Thread(target=self.timeout)
    
    def start(self):
        print("INFO: Starting Vision Control")
        self.runner_th.start()
        self.writer_th.start()
    
    def stop(self):
        self.shutdown = True
        self.runner_th.join()
        self.writer_th.join()

    def update_mavlink_msg(self, msg):
        if msg.get_type() == "ATTITUDE":
            self.last_yaw_update_time = time.time()
            self.yaw = msg.yaw
            self.pos['yaw'] = self.yaw
        elif msg.get_type() == "HEARTBEAT":
            self.update_mode(msg.custom_mode)

    def update_target_position(self, north, east, down):
        self.last_tar_update_time = time.time()
        self.pos['right'] = east
        self.pos['down'] = down
        self.pos['forward'] = north
        self.runner()

    def update_mode(self, mode):
        if (time.time() - self.last_tar_update_time) < .5 and self.last_mode != 4 and mode == 4:
            print("Drone changing state to centring on target")
            self.control_state = ControlState.CENTERING_ON_TARGET
        elif mode != 4:
            if self.last_mode == 4:
                print("Drone no longer in guided")
            self.control_state = ControlState.WAITING_FOR_INIT
            self.send_desired_position(0,0,0, final_yaw)
        self.pos['drone_mode'] = mode
        self.last_mode = mode

    def send_desired_position(self, forward, right, down, yaw):
        self.pos['des_r'] = right
        self.pos['des_d'] = down
        self.pos['des_f'] = forward
        self.pos['time'] = time.time()
        self.got_to_position_func(forward, right, down, yaw)
        self.data.put_nowait(self.pos)

    def waiting_for_init(self):
        self.send_desired_position(0,0,0, final_yaw)
        return False

    def centring_on_target(self):
        self.send_desired_position(self.pos['forward'] - staging_position_z,self.pos['right'],self.pos['down']+ end_position_down,final_yaw)
        if abs(self.pos['right']) < 0.05 and abs(self.pos['down'] + end_position_down) < 0.05:
            return True
        return False

    def making_contact(self):
        self.send_desired_position(self.pos['forward'] - end_position_z,self.pos['right'],self.pos['down']+ end_position_down,final_yaw)
        if abs(self.pos['forward'] - 0.05) < end_position_z :
            self.time_start_acquring = time.time()
            return True
        return False

    def acquiring_sample(self):
        self.send_desired_position(self.pos['forward'] - end_position_z,self.pos['right'],self.pos['down']+ end_position_down,final_yaw)
        if (time.time() - self.time_start_acquring) > 15:
            return True 
        return False

    def leaving_target(self):
        self.send_desired_position(self.pos['forward'] - finished_z,self.pos['right'],self.pos['down']+ end_position_down,final_yaw)
        if self.pos['forward'] > (finished_z - 0.05):
            return True
        return False
    
    def writer(self):
        while not self.shutdown:
            d = self.data.get()
            self.csvfile.write(" {d1[time]} , {d1[right]} , {d1[down]} , {d1[forward]}, {d1[yaw]} , {d1[des_r]} , {d1[des_d]} , {d1[des_f]}, {d1[des_y]}, {d1[mode]}, {d1[drone_mode]} \n".format(d1=d) )
            
    def timeout(self):
        while not self.shutdown:
            if (time.time() - self.last_tar_update_time) > .5:
                if self.control_state != ControlState.WAITING_FOR_INIT:
                    print("[LOST TARGET] Resetting state")
                self.control_state = ControlState.WAITING_FOR_INIT
                self.runner()
            time.sleep(1.0/20)


    def runner(self):
        if self.last_control_state != self.control_state:
            print("Chaging States from: " + str(self.last_control_state) + " to: " + str(self.control_state) )
            self.last_control_state = self.control_state
            self.pos['mode'] = self.control_state.value
        if self.control_state is ControlState.WAITING_FOR_INIT:
            #print("[LOOP] waiting for init")
            if self.waiting_for_init():
                self.control_state = ControlState.CENTERING_ON_TARGET
            else:
                pass
        elif self.control_state == ControlState.CENTERING_ON_TARGET:
            #print("[LOOP] centring_state")
            if self.centring_on_target():
                self.control_state = ControlState.MAKING_CONTACT
            else:
                pass
        elif self.control_state == ControlState.MAKING_CONTACT:
            #print("[LOOP] making contact")
            if self.making_contact():
                self.control_state = ControlState.ACQUIRING_SAMPLE
            else:
                pass
        elif self.control_state == ControlState.ACQUIRING_SAMPLE:
            #print("[LOOP] sampling")
            if self.acquiring_sample():
                self.control_state = ControlState.LEAVING_TARGET
            else:
                pass
        elif self.control_state == ControlState.LEAVING_TARGET:
            #print("[LOOP] leaving")
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
