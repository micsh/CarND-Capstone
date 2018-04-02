from pid import PID
import rospy
from yaw_controller import YawController
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
LOGGING = False


class Controller(object):
    def __init__(self, **parameters):

	self.parameters = parameters
        self.main_vehicle_mass = parameters['vehicle_mass']
        self.main_fuel_capacity = parameters['fuel_capacity']
        self.main_brake_deadband = parameters['brake_deadband']
        self.main_decel_limit = parameters['decel_limit']
        self.main_accel_limit = parameters['accel_limit']
        self.main_wheel_radius = parameters['wheel_radius']
        self.main_wheel_base = parameters['wheel_base']
        self.main_steer_ratio = parameters['steer_ratio']
        self.main_lat_accel = parameters['max_lat_accel']
        self.main_max_steer_angle = parameters['max_steer_angle']

        # Caluclate kerb_mass i.e., mass of vehicle with liquids (fuel, etc) minus accessories
        # Assume a fuel level of 100 percent. If you have access to changing Fuel levels, multply value by percentage of fuel present

        self.kerb_mass = self.main_vehicle_mass + (self.main_fuel_capacity * GAS_DENSITY)

        # Calculate Max braking force allowed.
        # assume a 0.5 braking force ratio (including brake pedal to brake force of gear system)
        # mutliply value by wheel radius to account for angular decelartion forces on wheel
        # where angular decelaration is deceleration velocity/wheel radius
        self.max_braking_force =(0.5 * self.kerb_mass* self.main_wheel_radius)

        # Intiate LowPass filters for Throttle, Steering and brake
	# Create longitudinal pid controller
        #self.throttle_pid = PID(2.0,0.00001,0.1,parameters['accel_limit'],parameters['decel_limit'])
        self.throttle_pid = PID(0.3, 0.1, 0.02, 0.0, 0.2)

	# Create provided yaw controller object for lateral control
	self.min_speed = 0.1
        self.yaw = YawController(self.main_wheel_base,self.main_steer_ratio, self.min_speed, self.main_lat_accel, self.main_max_steer_angle)
        tau = 0.5 # 1/(2pi * tau) = cutoff frequency
        ts = 0.02 # Sample time
        #self.steerLowPass= LowPassFilter(0.007,2.0)
        #self.steer_lp = LowPassFilter(tau, ts)
        #self.throttle_lp = LowPassFilter(0.5,0.1)
        #self.throttle_lp = LowPassFilter(tau, ts)
        self.vel_lp = LowPassFilter(tau, ts)
	self.Call_number = 0

    def control(self, target_linear_vel, target_angular_vel, current_vel, sampletime, dbw_enabled):
        # Return throttle, brake, steer

        if not dbw_enabled:
            self.throttle_pid.reset()
            return 0.0, 0.0, 0.0

        current_vel = self.vel_lp.filt(current_vel)

	#wait for three initial runs before sending out Throttle for PID to settle.
	if (self.Call_number <= 3):
		self.Call_number += 1
		return 0.0, 1.0, 0.0

	vel_error = target_linear_vel - current_vel
        #self.throttle_pid.update(vel_error)
        #throttle = 0.095*self.throttle_pid.get_val()
        #throttle = self.throttle_lp.filt(throttle)
        

        throttle = self.throttle_pid.step(vel_error, sampletime)
        brake = 0.0

        if target_linear_vel == 0.0 and current_vel < 0.1:
            throttle = 0.0
            brake = 400 # N*m to hold the car in place if we are stopped at a red light. Acceleration ~ 1m/s^2
        elif throttle < 0.1 and vel_error < 0:
            throttle = 0.0
            decel = max(vel_error, self.parameters['decel_limit'])
            brake = abs(decel) * (self.parameters['vehicle_mass'] + self.parameters['fuel_capacity'] * GAS_DENSITY) * self.parameters['wheel_radius']

        
        '''
        if (throttle > 0.0):
            brake = 0.0
            if (LOGGING): rospy.loginfo('throttle: %s, brake: %s', throttle, brake)
        #elif (throttle <= 0.0):
            # brake proportional to throttle value in opposite direction)
            #brake = min(0.1*throttle, self.main_decel_limit)
            #throttle = 0
        else:
            decel = abs(throttle)
            if decel < self.parameters['brake_deadband']:
                decel = 0.0
            brake = decel * (self.parameters['vehicle_mass'] + self.parameters['fuel_capacity'] * GAS_DENSITY) * self.parameters['wheel_radius']

            throttle = 0.0
            
            if (LOGGING): rospy.loginfo('brake %s, throttle: %s', brake, throttle)

        '''
        steer = self.yaw.get_steering(target_linear_vel, target_angular_vel, current_vel)
        #steer = self.steerLowPass.filt(steer)

        return throttle, brake, steer
