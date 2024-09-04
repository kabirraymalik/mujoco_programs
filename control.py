import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

class Bot():
    def __init__(self, init_motor_pos):
        self.move_queue = []
        self.current_position = State(init_motor_pos)
        self.loop_states = True
        #PID stuff
        self.time = 0
        self.integral = 0
        self.time_prev = -1e-6
        self.e_prev = 0
        self.Kp = 0.6
        self.Ki = 0.2
        self.Kd = 0.1
    
    #controller modified from https://softinery.com/blog/implementation-of-pid-controller-in-python/
    def PID(self, goal, curr):
        # PID calculations
        e = goal - curr
            
        P = self.Kp*e
        self.integral = self.integral + self.Ki*e*(self.time - self.time_prev)
        D = self.Kd*(e - self.e_prev)/(self.time - self.time_prev)

        # calculate manipulated variable - MV 
        MV = P + self.integral + D
        
        # update stored data for next iteration
        self.e_prev = e
        return MV
    
    def update_time(self, time):
        self.time_prev = self.time
        self.time = time


class State():
     #only one being used rn
    def __init__(self, motors):
        self.motor_positions = motors
    
    def compare_to(self, other_position, accuracy):
        if len(self.motor_positions) == len(other_position.motor_positions):
            deviations = []
            for id in range(0, len(self.motor_positions) - 1):
                deviations.append(abs(self.motor_positions[id] - other_position.motor_positions[id])/self.motor_positions[id])
            avg_percentage_deviation = 0
            for deviation in deviations:
                avg_percentage_deviation += deviation
            avg_percentage_deviation /= (len(deviations) - 1)
            if avg_percentage_deviation < accuracy:
                return True
        print('error: invalid state comparison')
        return False