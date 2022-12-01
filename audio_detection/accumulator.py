#!/usr/bin/env python3

import numpy as np

class Accumulator():

    """
        Adjust the k,h and z values depending on the func
        k => to adjust rate of decrease`x`
        h => to adjust the rate of change of accumulation
        z => to adjust for the noise level
    """
    def __init__(self, k = 2, h = 0.005, z=0.1):
        self.k = k
        # this will be updated by the accumulator func itself
        self.curr_x = 0

        self.input = None
        self.h = h
        self.z = z

    """
        The function is used to change the initial values if 
        we would like to continue from some point
        x => current accumulation point
    """
    def set_x(self, x):
        self.curr_x = x

    """
        Calculation for the accumulator func
    """
    def accumulator_func(self):
        # to calculate the change in x
        f = lambda x, input: -self.k*x + 10*input + self.z*np.random.randn()
        self.curr_x = self.curr_x + self.h*f(self.curr_x, self.input)
    
    """
        To set input (0,1) and calculate
    """
    def check_sound(self, input):
        self.input = input
        self.accumulator_func()
        return self.curr_x