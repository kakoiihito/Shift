class_name CarValues
extends Resource



@export_group("Car")

@export var wheel_base = 6.0
@export var track = 2.5

@export_group("Suspension")

@export var rest_length = [0.32, 0.32, 0.31, 0.31]
@export var spring_stiffness = [28700, 28700, 17000, 17000]
@export var max_compression = [0.10, 0.10, 0.09, 0.09]
@export var wheel_spring_force = [Vector3(), Vector3(), Vector3(), Vector3()]
@export var weight_distribution = [0.250, 0.250, 0.250, 0.250]
@export var velocity_exponent = 1.0
@export var rear_antiroll_bar = true
@export var front_antiroll_bar = true
@export var front_antiroll_bar_stiffness = 1900.0
@export var rear_antiroll_bar_stiffness = 1100.0
@export var damper_ratio = [0.35, 0.35, 0.35, 0.35]

@export_group("Steering")

@export var max_tire_turn_angle = 38.0
@export var steering_wheel_travel = 1000.0
@export var steering_ratio = 15.0
@export var tire_turn_speed = 3.5
@export var steering_stiffness = 720.0
@export var speed_factor_coeff = 0.035
@export var ackermann_factor = 0.90

@export_group("Brake")


@export var max_brake_torque = 900.0

@export_group("Assists")

@export_subgroup("ABS")

@export var ABS = false 
@export var ABS_Rate = 0.0
@export var abs_slip_threshold = -0.15
@export var abs_decay_rate = -25.0

@export_subgroup("TC")

@export var TC = false 
@export var tc_slip_threshold = 0.12
@export var tc_decay_rate = -20.0

@export_subgroup("Stability")

var Stability = false

@export_group("Motor")


@export var max_torque = 135.0
@export var max_rpm = 7200.0
@export var idle_rpm = 850.0
@export var stall_rpm = 500.0
@export var engine_inertia = 0.11 
@export var FL_torque_engine = false
@export var FR_torque_engine = false
@export var RL_torque_engine = true
@export var RR_torque_engine = true
@export var FR_torque_brake = true
@export var FL_torque_brake = true
@export var RR_torque_brake = true
@export var RL_torque_brake = true
@export var friction_c0 = 6.0
@export var friction_c1 = 9.0
@export var friction_c2 = 14.0
@export var torque_curve: Curve

@export_subgroup("Limited Slip Differential")


@export var torsen_lsd = false
@export var clutch_lsd = false
@export var electronic_lsd = false
@export var open_diff = true
@export var minimum_clutch_lsd_force = 100.0
@export var clutch_lsd_ramp_factor = 1.0
@export var center_diff_split = 0.5
@export var TBR = 2.0
@export var SLIP_THRESHOLD = 0.01

@export_group("Transmission")

@export var is_shifting = false
@export var shift_timer = 0.0
@export var drive_train_efficeny = 0.85 
@export var final_drive = 4.300
@export var gear_ratio = [-3.758, 0.0, 3.136, 1.888, 1.330, 1.000, 0.814]
@export var current_gear = 1
@export var max_clutch_torque = 210.0 
@export var unlock_threshold = 5.0
@export_group("Wheel")
@export var wheel_radius = 0.288
@export var wheel_mass = 10.5
@export var rolling_resistance_coeff = 0.013
@export var camber_angles = [-1.2, -1.2, -1.7, -1.7]
@export var camber_gain = [-18.0, -18.0, -25.0, -25.0]

# MF 5.2 model

@export_subgroup("Pacejka Longitudinal")
@export var b0 = 1.5
@export var b1 = -5.0
@export var b2 = 950.0
@export var b3 = 0.0
@export var b4 = 170.0
@export var b5 = 0.0
@export var b6 = 0.0008
@export var b7 = 0.006
@export var b8 = -2.0
@export var b9 = 0.0
@export var b10 = 0.0
@export var b11 = 0.0
@export var b12 = 0.0
@export var b13 = 0.0

@export_subgroup("Pacejka Lateral")
@export var a0  = 1.0     
@export var a1  = 0.0 
@export var a2  = 2400.0  
@export var a3  = 28080.0
@export var a4  = 232.0  
@export var a5  = 0.0    
@export var a6  = -1.2   
@export var a7  = -1.2    
@export var a8  = 0.0
@export var a9  = 0.0
@export var a10 = 0.0
@export var a11 = 0.0
@export var a12 = 0.0
@export var a13 = 0.0
@export var a14 = 0.0
@export var a15 = 0.0
@export var a16 = 0.0
@export var a17 = 0.0

@export_subgroup("Pacejka Aligning Torque")
@export var Ro = 0.299
@export var FNzo = 2200.0
@export var ssz1 = 0.007
@export var ssz2 = 0.012
@export var ssz3 = -0.008
@export var ssz4 = 0.003
@export var lambda_s = 1.0
@export var Br = 5.5
@export var Cr = 1.5
@export var Dr = 0.008
@export var Bt = 7.5
@export var Ct = 1.35
@export var Dt = 0.12
@export var Et = -1.2
@export var Ky1 = 14.5
@export var Ky2 = 1.65
@export var Ky3 = 0.25

@export_subgroup("Pacejka Longitudinal G-Function")
@export var rBx1 = 5.0
@export var rBx2 = 8.0
@export var rBx3 = 0.0
@export var rCx1 = 1.05
@export var rEx1 = -0.25
@export var rEx2 = 0.0
@export var rHx1 = 0.0
@export var lambda_xalpha = 1.0

@export_subgroup("Pacejka Lateral G-Function")
@export var rBy1 = 7.0
@export var rBy2 = 2.5
@export var rBy3 = 0.0
@export var rBy4 = 0.0
@export var rCy1 = 1.05
@export var rEy1 = 0.0
@export var rEy2 = 0.0
@export var rHy1 = 0.0
@export var rHy2 = 0.0
@export var rVy1 = 0.0
@export var rVy2 = 0.0
@export var rVy3 = 0.0
@export var rVy4 = 0.0
@export var rVy5 = 1.9
@export var rVy6 = 0.0
@export var lambda_ykappa = 1.0
@export var lambda_Vyk = 1.0
