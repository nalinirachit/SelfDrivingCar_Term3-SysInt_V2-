
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

from pid import pid
from yaw_controller import yaw_controller

# Nalini - added 8/18/2018


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.wheel_base = kwargs[wheel_base]


        pass

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        return 1., 0., 0.
