

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
		self.fuel_capacity = kwargs['fuel_capacity']
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
		

		self.last_time = rospy.get_time()
		

	def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
		# TODO: Change the arg, kwarg list to suit your needs
		# Return throttle, brake, steer
		if not dbw_enabled:
			self.pid_controller.reset()
			return 0., 0., 0.


		current_vel = self.vel_lpf.filt(current_vel)

		steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

		vel_error = linear_vel - current_vel
		self.last_vel = current_vel

		current_time = rospy.get_time()
		sample_time = current_time - self.last_time
		self.last_time = current_time

		throttle = self.pid_controller.step(vel_error, sample_time)
		brake = 0

		If linear_vel == 0 and current_vel < 0.1:
			throttle = 0
			break = 400 # car stops at light

		elif throttle < .1 and vel_error < 0:
			throttle = 0
			decel = max(vel_error, self.decel_limit)
			brake = abs(decel)*self.vehicle_mass*self.wheel_radius







		return throttle, brake, steering
