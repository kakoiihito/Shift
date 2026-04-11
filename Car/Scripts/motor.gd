extends Node

@export var car: RigidBody3D
@onready var Values: Resource

	####################
	# ENGINE VARIABLES #
	####################

var engine_rpm: float
var wheel_engine_torque = Data.wheel_engine_torque
var engine_angular_velocity: float

var engine_stalled: bool

	##########################
	# TRANSMISSION VARIABLES #
	##########################
	
var is_shifting = Data.is_shifting

	###################
	# WHEEL VARIABLES #
	###################
	
var wheel_angular_velocity = Data.wheel_angular_velocity
var longitude_force = Data.longitude_force





func motor_process(delta: float) -> void:
	var torque_curve = car.torque_curve
	
	var throttle_input := Input.get_action_strength("Gas")
	var clutch_input := Input.get_action_strength("Clutch")
	
	var clutch_torque_on_engine := 0.0
	var angular_velocity_sum: float = 0.0
	var driven_count: int = 0
	var target_engine_ang_vel: float
	var drivetrain_ratio = Data.current_gear_ratio * Values.final_drive
	
	var normalized = clamp((1.0 - clutch_input - 0.3) / 0.4, 0.0, 1.0)
	var clutch_engagement = normalized * normalized * (3.0 - 2.0 * normalized)
	
	var front_axle = [0,1]
	var rear_axle = [2,3]
	var driven_axle = []
	
	var driven_wheels = [Values.FL_torque_engine, Values.FR_torque_engine, Values.RL_torque_engine, Values.RR_torque_engine]
	for i in range(4):
		if driven_wheels[i] == true:
			angular_velocity_sum += wheel_angular_velocity[i]
			driven_count += 1
	
	var normalized_rpm = engine_rpm / Values.max_rpm
	var engine_torque: float

	engine_torque = torque_curve.sample(normalized_rpm) * Values.max_torque * throttle_input

	if is_shifting:
		clutch_engagement = 0.0
		
	var base_friction = Values.friction_c0                      
	var linear_friction = Values.friction_c1 * normalized_rpm       
	var quadratic_friction = Values.friction_c2 * normalized_rpm * normalized_rpm
	var engine_friction = base_friction + linear_friction + quadratic_friction
	
			
	if driven_count > 0:
		target_engine_ang_vel = (angular_velocity_sum / driven_count) * drivetrain_ratio
		var speed_difference = engine_angular_velocity - target_engine_ang_vel
		var max_transferable_torque = Values.max_clutch_torque * clutch_engagement
					
		if abs(speed_difference) > Values.unlock_threshold:
			clutch_torque_on_engine = -sign(speed_difference) * max_transferable_torque
		else:
			var required_torque = (Values.engine_inertia * (target_engine_ang_vel - engine_angular_velocity)) / delta
			
			if abs(required_torque) <= max_transferable_torque:
				engine_angular_velocity = target_engine_ang_vel
				clutch_torque_on_engine = 0.0
			else:
				clutch_torque_on_engine = -sign(speed_difference) * max_transferable_torque
			
	var net_engine_torque = engine_torque - engine_friction + clutch_torque_on_engine
	var engine_angular_accel = net_engine_torque / Values.engine_inertia
	engine_angular_velocity += engine_angular_accel * delta
	engine_angular_velocity = clamp(engine_angular_velocity, Values.stall_rpm * TAU / 60.0, Values.max_rpm * TAU / 60.0)
	engine_rpm = engine_angular_velocity * 60.0 / TAU
	
	if engine_rpm <= Values.stall_rpm + 50.0 and clutch_engagement > 0.1 and not engine_stalled:
		engine_stalled = true
		
	if engine_stalled:
		engine_torque = 0.0
		clutch_engagement = 0.0
		engine_angular_velocity = 0.0
		
	if Input.is_action_pressed("Ignition"):
		engine_stalled = false
		engine_angular_velocity = Values.idle_rpm * TAU / 60.0
	
	var clutch_torque_to_wheels = -clutch_torque_on_engine 
	var torque_at_wheels = clutch_torque_to_wheels * (drivetrain_ratio) * Values.drive_train_efficeny
	
	if Values.FL_torque_engine and Values.FR_torque_engine and Values.RL_torque_engine and Values.RR_torque_engine:
		driven_axle =[front_axle, rear_axle]
	elif Values.FL_torque_engine and Values.FR_torque_engine:
		driven_axle = [front_axle]
	elif Values.RL_torque_engine and Values.RR_torque_engine:
		driven_axle = [rear_axle]
	else:
		var per_wheel_torque = torque_at_wheels / driven_count if driven_count > 0 else 0.0
		for i in range(4):
			if driven_wheels[i]:
				wheel_engine_torque[i] = per_wheel_torque
			else:
				wheel_engine_torque[i] = 0.0
	
	for axle in driven_axle:
		
		var axle_torque: float
		if driven_axle.size() == 2:
			axle_torque = torque_at_wheels * Values.center_diff_split if axle == front_axle else torque_at_wheels * (1.0 - Values.center_diff_split)
		else:
			axle_torque = torque_at_wheels
		
		var slip_a = longitude_force[axle[0]]
		var slip_b = longitude_force[axle[1]]
		
		var T_lock: float
		var T_high: float
		var T_low: float
		
		if Values.torsen_lsd:
			if min(slip_a, slip_b) <= 0.0:
				T_high = axle_torque / 2.0
				T_low  = axle_torque / 2.0
			else:
				T_high = axle_torque * (Values.TBR / (Values.TBR + 1.0))
				T_low  = axle_torque * (1.0 / (Values.TBR + 1.0))
		elif Values.clutch_lsd:
			T_lock = Values.minimum_clutch_lsd_force + (axle_torque * Values.clutch_lsd_ramp_factor)
			T_lock = min(T_lock, axle_torque / 2.0) 
			T_high = (axle_torque / 2.0) + T_lock
			T_low  = (axle_torque / 2.0) - T_lock
		elif Values.electronic_lsd:
			pass # will write logic
		elif Values.open_diff:
			T_high = axle_torque / 2.0
			T_low  = axle_torque / 2.0
		
		if (slip_a - slip_b) > Values.SLIP_THRESHOLD:
			wheel_engine_torque[axle[0]] = T_low
			wheel_engine_torque[axle[1]] = T_high
		elif (slip_b - slip_a) > Values.SLIP_THRESHOLD:
			wheel_engine_torque[axle[0]] = T_high
			wheel_engine_torque[axle[1]] = T_low
		else:
			wheel_engine_torque[axle[0]] = axle_torque / 2.0
			wheel_engine_torque[axle[1]] = axle_torque / 2.0
