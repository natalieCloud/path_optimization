#!/usr/bin/env python

import numpy as np
from scipy.optimize import minimize

# Takes in the thickness vector and calculates the 
# scalar error
def error_func (thickness_vec):
    #Calculates the standard deviation thickness
    
# Calls noether via a client call to the noether caller
def call_noether(line_spacing, point_spacing, tool_liftoff,
                 min_hole_size, min_seg_size, search_radius):
    #Construct the client

    #Call the service

    # Return the toolpath to be used via exleys' code, wrapped in 
    # a tuple with the boolean sucess - this function may just be 
    # represented in the final optmize code w/o a seperate call, 
    # however for the purpose of layout it will be represented as a 
    # seperate function right now

def call_thickness(toolpath):

    # Calls exley's code -again like above this would be wrapped into 
    # the optimize function
    # overall would return the thickness vector

def optimize_thickness(v0, min_seg_size, min_hole_size, search_radius):
    
    toolpath = call_noether(v0[0], v0[1], v0[2], min_hole_size, min_seg_size, search_radius)
    thickness_vec = call_thickness(toolpath)
    return error_func(thickness_vec)

def main():

    # Start of the variables that we wish to optimize
    line_spacing = 0.5 # Or roughly ~1/2 the fan width
    point_spaing = 0.5 # Or roguhly ~1/2 the minor axis of the above
    liftoff = 0.1

    # The variables we wish to pass as constant
    min_seg_size = 0.1
    min_hole_size = 0.1
    search_radius = 0.1

    v0 = [line_spacing, point_spaing, liftoff]
    res = minimize(optimize_thickness, v0, method='TODO',
                   args=(min_seg_size, min_hole_size, search_radius), options={'xatol': 1e-8, 'disp': True})


if __name__ == "__main__":
    main()
