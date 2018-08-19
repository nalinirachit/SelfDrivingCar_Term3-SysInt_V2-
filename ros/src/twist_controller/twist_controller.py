

from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

# Nalini - added 8/19/2018


class Controller(object):
	def __init__(self, *args, **kwargs):
		# TODO: Implement
		self.wheel_base = kwargs['wheel_base']
		self.steer_ratio = kwargs['steer_ratio']
		self.min_speed = 0
		self.max_lat_accel = kwargs['max_lat_accel']
		self.max_steer_angle = kwargs['max_steer_angle']
		self.decel_limit = kwargs['decel_limit']
		self.accel_limit = kwargs['accel_limit']
		self.wheel_radius = kwargs['wheel_radius']

		Kp = 1
		Ki = 0
		Kd = 0.05

		self.pid_controller = PID(Kp, Ki, Kd, mn = self.decel_limit, mx = self.accel_limit)

		self.yaw_controller = Yaw_Controller(self.wheel_base, self.steer_ratio, self.min_speed, self.max_lat_accel, self.max_steer_angle)

		tau = 0.5
		ts = .02

		self.vel_lpf = LowPassFilter(tau, ts)

		self.vehicle_mass = kwargs['vehicle_mass'] + kwargs['fuel_capacity'] * GAS_DENSITY
		self.brake_deadband = kwargs['brake_deadband']



		

	def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
		# TODO: Change the arg, kwarg list to suit your needs
		# Return throttle, brake, steer
		if not dbw_enabled:
			self.pid_controller.reset()
			return 0., 0., 0.


		current_vel = self.vel_lpf.filt(current_vel)


		return 1., 0., 0.
