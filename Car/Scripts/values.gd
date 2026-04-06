extends Node

# used for tuning the car's behavior, data.gd is for real time calculation

	#######
	# CAR #
	#######
	
var car_value_tres: Resource


var wheel_base = 2.3
var track = 1.5

	########################
	# SUSPENSION VARIABLES #
	########################

var rest_length = [0.18, 0.18, 0.18, 0.18]
var spring_stiffness = [28700.0, 28700.0, 17000.0, 17000.0]
var max_compression = [0.085, 0.085, 0.090, 0.090]
var wheel_spring_force = [Vector3(), Vector3(), Vector3(), Vector3()]
var weight_distribution = [0.25, 0.25, 0.25, 0.25]
var velocity_exponent = 1.0
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
	
var max_brake_torque = 20000000000000000000000000000000000.0 

	####################
	# ENGINE VARIABLES #
	####################

var max_torque = 351.0 
var max_rpm = 7500.0 
var idle_rpm = 850.0
var stall_rpm = 400.0
var engine_inertia = 0.75

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
var max_clutch_torque = 340.0
var lock_threshold = 5.0
var unlock_threshold = 12.0
var clutch_stiffness = 64.0

	###################
	# WHEEL VARIABLES #
	###################

var wheel_radius = 0.3
var wheel_mass = 15.0
var rolling_resistance_coeff = 0.013
var friction_coefficient = 1.1
var wheel_inertia = 0.6 * wheel_mass * wheel_radius * wheel_radius

# Longitudinal
var b0 = 1.3882
var b1 = -8.0
var b2 = 1150.0
var b3 = 0.0
var b4 = 320.0
var b5 = 0.0
var b6 = 0.0
var b7 = 0.0
var b8 = -2.0
var b9 = 0.0
var b10 = 0.0
var b11 = 0.0
var b12 = 0.0
var b13 = 0.0

# Lateral
var a0 = 1.7379
var a1 = -9.0
var a2 = 1250.0
var a3 = 1400.0
var a4 = 6.5
var a5 = 0.0
var a6 = -0.08
var a7 = -2.0
var a8 = 0.0
var a9 = 0.0
var a10 = 0.0
var a11 = 0.0
var a12 = 0.0
var a13 = 0.0
var a14 = 0.0
var a15 = 0.0
var a16 = 0.0
var a17 = 0.0

# Longitduinal G Functions
var rBx1 = 10.0
var rBx2 = 8.0
var rBx3 = 0.0
var rCx1 = 1.05
var rEx1 = -0.2
var rEx2 = 0.0
var rHx1 = 0.02
var lambda_xalpha = 1.0

# Lateral G Functions
var rBy1 = 7.0
var rBy2 = 2.5
var rBy3 = 0.05
var rBy4 = 0.0
var rCy1 = 1.0
var rEy1 = -0.3
var rEy2 = 0.0
var rHy1 = 0.02
var rHy2 = 0.0
var rVy1 = 0.05
var rVy2 = -0.025
var rVy3 = 0.0
var rVy4 = 4.0
var rVy5 = 1.9
var rVy6 = 10.0
var lambda_ykappa = 1.0
var lambda_Vyk = 1.0

	#################
	# LSD VARIABLES #
	#################
	
	# lsd does not apply to abnormal drivetrain configurations (Example: Front Right, Left Back Wheel drive)
	
var TBR = 2.0
var torsen_lsd = false
var clutch_lsd = false
var electronic_lsd = false
var open_diff = true
var minimum_lsd_force = 100.0
var ramp_factor = 1.0
var center_diff_split = 0.5 # front bias

func _ready() -> void:
	if car_value_tres != null:
		pass # make it so all values are set to the tres ones
