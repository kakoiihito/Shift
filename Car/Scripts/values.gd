extends Node

# used for tuning the car's behavior, data.gd is for real time calculation

	#######
	# CAR #
	#######

var mass = 1000.0
var wheel_base = 4.0
var track = 2.0

	########################
	# SUSPENSION VARIABLES #
	########################

var rest_length = [0.18, 0.18, 0.18, 0.18]
var spring_stiffness = [28700.0, 28700.0, 17000.0, 17000.0]
var max_compression = [0.12, 0.12, 0.12, 0.12]
var wheel_spring_force = [Vector3(), Vector3(), Vector3(), Vector3()]
var weight_distribution = [0.25, 0.25, 0.25, 0.25]
var velocity_exponent = 1.1

	######################
	# STEERING VARIABLES #
	######################

var max_tire_turn_angle = 40.0
var tire_turn_speed = 3.0

	###################
	# BRAKE VARIABLES #
	###################
	
var max_brake_torque = 200.0 # How much the car can brake



	####################
	# ENGINE VARIABLES #
	####################

var max_torque = 151.0 # used to convert the torque value on the curve to a proper force amount.
var max_rpm = 7500.0 # Max amount of engine rotations
var idle_rpm = 850.0 # Lowest amount of engine rotations


var engine_inertia = 0.75
# Torque can be applied at any of the wheels. So, these vars allow the torque to be applied at any wheels neccessary.

var FR_torque_engine = false
var FL_torque_engine = false
var RR_torque_engine = true
var RL_torque_engine = true

var FR_torque_brake = true
var FL_torque_brake = true
var RR_torque_brake = true
var RL_torque_brake = true

	##########################
	# TRANSMISSION VARIABLES #
	##########################
	
var is_shifting = false
var shift_timer = 0.0
var drive_train_efficeny = 0.9
var final_drive = 3.63 # Final gear to multiple torque.
var gear_ratio = [-3.27, 0.0, 3.64, 2.6, 1.53, 1.16, 0.94]   # power multiplyer for engine
var current_gear = 1
var max_clutch_torque = 302.0 # max amount of engine torque that can be transfered to the wheels
var lock_threshold = 5.0
var unlock_threshold = 12.0
var clutch_stiffness = 64.0


	###################
	# WHEEL VARIABLES #
	###################
	


var wheel_radius = 0.3 # How big the wheel is.
var wheel_mass = 20.0 # How much the wheel takes up
var rolling_resistance_coeff = 0.015
var friction_coefficient = 1.2
var cornering_stiffness = 60000.0
var wheel_inertia = 0.8 * wheel_mass * wheel_radius * wheel_radius
