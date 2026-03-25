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
var max_compression = [0.085, 0.085, 0.090, 0.090]
var wheel_spring_force = [Vector3(), Vector3(), Vector3(), Vector3()]
var weight_distribution = [0.25, 0.25, 0.25, 0.25]
var velocity_exponent = 1.1
var rear_antiroll_bar = true
var front_antiroll_bar = true
var rear_antiroll_bar_stiffness = 2870.0
var front_antiroll_bar_stiffness = 1700.0

	######################
	# STEERING VARIABLES #
	######################

var max_tire_turn_angle = 40.0
var tire_turn_speed = 3.0

	###################
	# BRAKE VARIABLES #
	###################
	
var max_brake_torque = 200.0 

	####################
	# ENGINE VARIABLES #
	####################

var max_torque = 151.0 
var max_rpm = 7500.0 
var idle_rpm = 850.0
var stall_rpm = 400.0
var engine_inertia = 0.75

# Torque can be applied at any of the wheels. So, these vars allow the torque to be applied at any wheels neccessary.
var FL_torque_engine = false
var FR_torque_engine = false
var RL_torque_engine = true
var RR_torque_engine = true

var FR_torque_brake = true
var FL_torque_brake = true
var RR_torque_brake = true
var RL_torque_brake = true

	##########################
	# TRANSMISSION VARIABLES #
	##########################
	
var is_shifting = false
var shift_timer = 0.0
var drive_train_efficeny = 0.86
var final_drive = 4.3
var gear_ratio = [-3.968, 0.0, 3.136, 1.888, 1.330, 1.0, 0.814]
var current_gear = 1
var max_clutch_torque = 140.0
var lock_threshold = 5.0
var unlock_threshold = 12.0
var clutch_stiffness = 64.0

	###################
	# WHEEL VARIABLES #
	###################

var wheel_radius = 0.3
var wheel_mass = 20.0
var rolling_resistance_coeff = 0.015
var friction_coefficient = 1.9
var wheel_inertia = 0.8 * wheel_mass * wheel_radius * wheel_radius

	#################
	# LSD VARIABLES #
	#################
	
var TBR = 2.0
var torsen_lsd = true
var clutch_lsd = false
var electronic_lsd = false
var minimum_lsd_force = 100.0
var ramp_factor = 1.0
