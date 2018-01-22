from yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        self.fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        self.brake_deadband = rospy.get_param('~brake_deadband', .1)
        self.decel_limit = rospy.get_param('~decel_limit', -5)
        self.accel_limit = rospy.get_param('~accel_limit', 1.)
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        self.wheel_base = rospy.get_param('~wheel_base', 2.8498)
        self.steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        self.max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        self.max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

	self.min_speed = ONE_MPH # TODO

	self.pid_throttle = PID(0.5,0.0005,0.05,mn=self.decel_limit,mx=self.accel_limit)
	self.yaw_control = YawController(self.wheel_base, self.steer_ratio, self.min_speed, self.max_lat_accel, self.max_steer_angle)
	#self.lowpass = LowPassFilter(1,1)
	self.prev_time = None

    def control(self,proposed_linear_velocity, proposed_angular_velocity, current_linear_velocity,dbw_enabled, dt):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
	throttle = 0.5
	brake = 0.0
	steering = 0.0
	
	steering = self.yaw_control.get_steering(proposed_linear_velocity, proposed_angular_velocity, current_linear_velocity)

	linear_velocity_error = proposed_linear_velocity - current_linear_velocity

	throttle = self.pid_throttle.step(linear_velocity_error, dt)
	rospy.loginfo("throttle: %s"%throttle)

	if throttle < 0:
		if abs(throttle)<self.brake_deadband:
			brake = 0.0 #todo
		else:
			brake = (self.vehicle_mass+(self.fuel_capacity*GAS_DENSITY))* abs(throttle)*self.wheel_base
	
	throttle = max(0.0,min(1.0,throttle))
	
        return throttle, brake, steering
