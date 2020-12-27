from math import *
import numpy as np
from utility import vec
import time
class IntegratorSystem:
    '''
    def example_second_order_derivative_function(t : )
    '''
    def __init__(self):
        self.f_d2 = 0.0
        self.f_d1 = 0.0
        self.f = 0.0
        self.t = 0.0
    def update(self, delta_time, f_d2_parameter):
        t_start = self.t
        t_end = self.t + delta_time
        
        f_d2_start = self.f_d2
        f_d2_end = f_d2_parameter # this fames accel is given in parameter

        f_d1_start = self.f_d1
        f_d1_end = f_d1_start + (f_d2_end + f_d2_start) * 0.5 * delta_time # add average accel of last frametime to velocity

        f_start = self.f
        f_end = f_start + (f_d1_start + f_d1_end) * 0.5 * delta_time # add average velocity of last frametime to posiiton


        # update fields
        self.t = t_end
        self.f_d2 = f_d2_parameter
        self.f_d1 = f_d1_end
        self.f = f_end



def example():
    np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

    t_0 = 0.0
    t_1 = 10.0
    t_mid = (t_0 + t_1) * 0.5

    integrator = IntegratorSystem()
    delta_time = 0.01
    integrator.f = 0.0
    integrator.f_d1 = 0.0
    integrator.f_d2 = 0.0
    while(True):
        accel = 4 * (180.0 - 0.0) / ((t_1 - t_0) * (t_1 - t_0))
        if(integrator.t > t_mid):
            accel = -accel
        if(integrator.t > t_1):
            accel = 0.0

        integrator.update(delta_time, accel)
        print(str(integrator.f) + "\t" + str(integrator.f_d1) + "\t T:" + str(integrator.t) + "\t Accel: " + str(accel))
        time.sleep(delta_time)


    pass

#example()