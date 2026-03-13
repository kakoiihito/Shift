extends Node

@export var car: RigidBody3D


var max_torque = Values.max_torque
var max_rpm = Values.max_rpm
var idle_rpm = Values.idle_rpm
var engine_rpm: float
var wheel_engine_torque = Data.wheel_engine_torque
var engine_angular_velocity: float
var engine_inertia = Values.engine_inertia
var FR_torque_engine = Values.FR_torque_engine
var FL_torque_engine = Values.FL_torque_engine
var RR_torque_engine = Values.RR_torque_engine
var RL_torque_engine = Values.RL_torque_engine
	##########################
	# TRANSMISSION VARIABLES #
	##########################
	
var wheel_angular_velocity = Data.wheel_angular_velocity
	
var is_shifting = Data.is_shifting
var drive_train_efficeny = Values.drive_train_efficeny
var final_drive = Values.final_drive
var gear_ratio = Values.gear_ratio
var current_gear_ratio = Data.current_gear_ratio
var max_clutch_torque = Values.max_clutch_torque
var lock_threshold = Values.lock_threshold
var unlock_threshold = Values.unlock_threshold
var clutch_stiffness = Values.clutch_stiffness
var is_clutch_locked: bool


func motor_process(delta: float) -> void:
	var torque_curve = car.torque_curve
	
	var clutch_torque_on_engine := 0.0
	var angular_velocity_sum: float = 0.0
	var driven_count: int = 0
	var throttle_input := Input.get_action_strength("Gas")
	var clutch_input := Input.get_action_strength("Clutch")
	var clutch_engagement = (1.0 - clutch_input) * (1.0 - clutch_input)
	var target_engine_ang_vel: float
	var drivetrain_ratio = current_gear_ratio * final_drive
	
	
	var driven_wheels = [FR_torque_engine, FL_torque_engine, RR_torque_engine, RL_torque_engine]
	for i in range(4):
		if driven_wheels[i] == true:
			angular_velocity_sum += wheel_angular_velocity[i]
			
			driven_count += 1
	
 # torque
	var normalized_rpm = engine_rpm / max_rpm
	var engine_torque: float
	if is_shifting == false:
		engine_torque = torque_curve.sample(normalized_rpm) * max_torque * throttle_input
	else: # if shifting you dont have torque due to engine not being in use
		clutch_engagement = 0.0
		engine_torque = 0.0
		
	var base_friction = 0.01 * max_torque
	var linear_friction = 0.03 * max_torque * normalized_rpm
	var quadratic_friction = 0.02 * max_torque * normalized_rpm * normalized_rpm
	var engine_friction = base_friction + linear_friction + quadratic_friction
	
	# clutch_torque calc, dry model
			
	if driven_count > 0:
		target_engine_ang_vel = (angular_velocity_sum / driven_count) * drivetrain_ratio
		var speed_difference = engine_angular_velocity - target_engine_ang_vel
		var max_transferable_torque = max_clutch_torque * clutch_engagement
					
		if abs(speed_difference) > unlock_threshold:
			clutch_torque_on_engine = clamp(
				-speed_difference * clutch_stiffness,
				-max_transferable_torque,
				max_transferable_torque
			)
		else:
			clutch_torque_on_engine = -sign(speed_difference) * max_transferable_torque
			
	var net_engine_torque = engine_torque - engine_friction + clutch_torque_on_engine
	var engine_angular_accel = net_engine_torque / engine_inertia
	engine_angular_velocity += engine_angular_accel * delta
	engine_angular_velocity = clamp(engine_angular_velocity, idle_rpm * TAU / 60.0, max_rpm * TAU / 60.0)
	engine_rpm = engine_angular_velocity * 60.0 / TAU
	# final calc
	
	var clutch_torque_to_wheels = -clutch_torque_on_engine
	var torque_at_wheels = clutch_torque_to_wheels * (drivetrain_ratio) * drive_train_efficeny
	var per_wheel_torque = torque_at_wheels / driven_count if driven_count > 0 else 0.0
	
	
	for i in range(4):
		if driven_wheels[i]:
			wheel_engine_torque[i] = per_wheel_torque
		else:
			wheel_engine_torque[i] = 0.0
