from pid import PID
import rospy
from yaw_controller import YawController
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):

        self.main_vehicle_mass = vehicle_mass
        self.main_fuel_capacity = fuel_capacity
        self.main_brake_deadband = brake_deadband
        self.main_decel_limit = decel_limit
        self.main_accel_limit = accel_limit
        self.main_wheel_radius = wheel_radius
        self.main_wheel_base = wheel_base
        self.main_steer_ratio = steer_ratio
        self.main_lat_accel = max_lat_accel
        self.main_max_steer_angle = max_steer_angle

        # Caluclate kerb_mass i.e., mass of vehicle with liquids (fuel, etc) minus accessories
        # Assume a fuel level of 100 percent. If you have access to changing Fuel levels, multply value by percentage of fuel present

        self.kerb_mass = self.main_vehicle_mass +(self.main_fuel_capacity * GAS_DENSITY)

        # Calculate Max braking force allowed.
        # assume a 0.5 braking force ratio (including brake pedal to brake force of gear system)
        # mutliply value by wheel radius to account for angular decelartion forces on wheel
        # where angular decelaration is deceleration velocity/wheel radius
        self.max_braking_force =(0.5 * self.kerb_mass* self.main_decel_limit * self.main_wheel_radius)

        # Intiate LowPass filters for Throttle, Steering and brake
        #self.throttle_pid = PID(0.1,0,0.34674,0,1)
        self.throttle_pid = PID(0.5,0.00001,0.0,0,1)
        self.min_speed = 0
        self.yaw = YawController(self.main_wheel_base,self.main_steer_ratio, self.min_speed, self.main_lat_accel, self.main_max_steer_angle)
        self.steerLowPass= LowPassFilter(0.2,0.1)

        # Temp Variables
        self.Call_number = 0
        self.prev_timestamp =  None

    def control(self, proposed_linear_vel, proposed_angular_vel, current_linear_vel, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if self.prev_timestamp is None or not dbw_enabled:
            self.prev_timestamp = rospy.get_time()
            return 0.0,0.0,0.0

        # Initialize throttle,  brake and steer values
        throttle = 0
        brake = 0
        steer = 0

        sampletime = rospy.get_time()-self.prev_timestamp
        if self.Call_number ==0:
            self.vel = current_linear_vel
            self.last_step_throttle = 0
            while True:
                self.prev_timestamp = rospy.get_time()
                if self.prev_timestamp!=0:
                     break
            self.Call_number+=1

        else:
             self.prev_timestamp=rospy.get_time()

        if (proposed_linear_vel < 0.): propsed_linear_vel = 0
        if(dbw_enabled):
             throttle_error = proposed_linear_vel-current_linear_vel
             throttle = self.throttle_pid.step(throttle_error, sampletime)
             if (throttle_error < 0.):
                   brake = abs(throttle_error) * self.max_braking_force
                   brake = min(min(brake, self.max_braking_force), 1.0)
                   throttle = 0.0
             else:
                   brake = 0.0
             
             if (brake <= self.main_brake_deadband ):
                   brake = self.main_brake_deadband *5
             rospy.loginfo('lin: %s , Ang:  %s ,  Cur:  %s' , proposed_linear_vel, proposed_angular_vel, current_linear_vel)

             steer_temp = self.yaw.get_steering(proposed_linear_vel, proposed_angular_vel, current_linear_vel)
             steer =self.steerLowPass.filt(steer_temp) 
             rospy.loginfo('Steering Value: %s ', steer)
        return throttle, steer,brake
