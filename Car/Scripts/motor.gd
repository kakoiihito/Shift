extends Node



func motor_process(delta: float, EngineData: RuntimeData.engine, TransmissionData: RuntimeData.transmission, WheelData: RuntimeData.wheels, Values: Resource) -> void:

	var driven_wheels = [Values.FL_torque_engine, Values.FR_torque_engine, Values.RL_torque_engine, Values.RR_torque_engine]
	var driven_count = 0

	for i in range(4):
		if driven_wheels[i]:
			driven_count += 1

	EngineData.engine_driven_count = driven_count
	
	# engine torque calculation
	
	engine_torque_calc(EngineData, Values)
	
	# engine friciton calculation
	
	engine_friction_calc(EngineData, Values)
	
	# clutch torque calculation
		
	clutch_torque_calc(TransmissionData, WheelData, EngineData, Values, delta)
			
	# engine rpm calculation

	engine_rpm_calc(EngineData, Values, delta)
	
	# engine stalling
	
	engine_stall_behavior(EngineData, Values)
	
	#Torque division (lsds, open diff)
	
	wheel_torque_divison(TransmissionData, EngineData, WheelData, Values)

func angular_velocity_sum_calc(WheelData: RuntimeData.wheels, EngineData: RuntimeData.engine, Values: Resource):
	
	var driven_wheels = [Values.FL_torque_engine, Values.FR_torque_engine, Values.RL_torque_engine, Values.RR_torque_engine]
	var driven_count: int = 0
	var angular_velocity_sum: float = 0.0
	
	for i in range(4):
		if driven_wheels[i]:
			angular_velocity_sum += WheelData.wheel_angular_velocity[i]
			driven_count += 1

	EngineData.engine_driven_count = driven_count
	
	return angular_velocity_sum
	
func engine_torque_calc(EngineData: RuntimeData.engine, Values: Resource):
	
	var throttle_input = Input.get_action_strength("Gas")
	
	var normalized_rpm = EngineData.engine_rpm / Values.max_rpm

	EngineData.engine_torque = Values.torque_curve.sample(normalized_rpm) * Values.max_torque * throttle_input
	

func engine_friction_calc(EngineData: RuntimeData.engine, Values: Resource):
	
	
	var normalized_rpm = EngineData.engine_rpm / Values.max_rpm
	var base_friction = Values.friction_c0                      
	var linear_friction = Values.friction_c1 * normalized_rpm       
	var quadratic_friction = Values.friction_c2 * normalized_rpm * normalized_rpm
	return base_friction + linear_friction + quadratic_friction
	
	
func clutch_torque_calc(TransmissionData: RuntimeData.transmission, WheelData: RuntimeData.wheels, EngineData: RuntimeData.engine, Values: Resource, delta: float):

	if TransmissionData.is_shifting or TransmissionData.current_gear == 1:
		EngineData.clutch_torque_on_engine = 0.0
		return

	var drivetrain_ratio = TransmissionData.current_gear_ratio * Values.final_drive
	if abs(drivetrain_ratio) < 0.0001:
		EngineData.clutch_torque_on_engine = 0.0
		return

	if EngineData.engine_driven_count <= 0:
		EngineData.clutch_torque_on_engine = 0.0
		return

	var clutch_input := Input.get_action_strength("Clutch")
	var normalized = clamp((1.0 - clutch_input - 0.3) / 0.4, 0.0, 1.0)
	var clutch_engagement = normalized * normalized * (3.0 - 2.0 * normalized)
	var max_transferable_torque = Values.max_clutch_torque * clutch_engagement

	var angular_velocity_sum = angular_velocity_sum_calc(WheelData, EngineData, Values)
	var target_engine_ang_vel = (angular_velocity_sum / EngineData.engine_driven_count) * drivetrain_ratio
	var speed_difference = EngineData.engine_angular_velocity - target_engine_ang_vel

	if abs(speed_difference) <= Values.unlock_threshold:
		var required_torque = -speed_difference * Values.engine_inertia / max(delta, 0.0001)
		EngineData.clutch_torque_on_engine = clamp(required_torque, -max_transferable_torque, max_transferable_torque)
	else:
		EngineData.clutch_torque_on_engine = sign(speed_difference) * max_transferable_torque

				

func engine_rpm_calc(EngineData: RuntimeData.engine, Values: Resource, delta: float):
	
	var throttle = Input.get_action_strength("Gas")
	var friction = engine_friction_calc(EngineData, Values)
	
	var net_engine_torque = EngineData.engine_torque - (friction * (1.0 - throttle)) - EngineData.clutch_torque_on_engine
	var engine_angular_accel = net_engine_torque / Values.engine_inertia
	EngineData.engine_angular_velocity += engine_angular_accel * delta
	EngineData.engine_angular_velocity = clamp(EngineData.engine_angular_velocity, Values.stall_rpm * TAU / 60.0, Values.max_rpm * TAU / 60.0)
	EngineData.engine_rpm = EngineData.engine_angular_velocity * 60.0 / TAU
	
func engine_stall_behavior(EngineData: RuntimeData.engine, Values: Resource):
	
	var clutch_input := Input.get_action_strength("Clutch")
	var normalized = clamp((1.0 - clutch_input - 0.3) / 0.4, 0.0, 1.0)
	var clutch_engagement = normalized * normalized * (3.0 - 2.0 * normalized)
	
	if EngineData.engine_rpm <= Values.stall_rpm + 50.0 and clutch_engagement > 0.1 and not EngineData.engine_stalled:
		EngineData.engine_stalled = true
		
	if EngineData.engine_stalled:
		EngineData.engine_torque = 0.0
		clutch_engagement = 0.0
		EngineData.engine_angular_velocity = 0.0
		
	if Input.is_action_pressed("Ignition"):
		EngineData.engine_stalled = false
		EngineData.engine_angular_velocity = Values.idle_rpm * TAU / 60.0
		
func wheel_torque_divison(TransmissionData: RuntimeData.transmission, EngineData: RuntimeData.engine, WheelData: RuntimeData.wheels, Values: Resource):

	var drivetrain_ratio = TransmissionData.current_gear_ratio * Values.final_drive

	var torque_at_wheels = EngineData.clutch_torque_on_engine * drivetrain_ratio * Values.drive_train_efficeny

	# finding axle usage (refuses abnormal configurations, more info in values.gd)

	var front_driven = Values.FL_torque_engine and Values.FR_torque_engine
	var rear_driven = Values.RL_torque_engine and Values.RR_torque_engine

	var do_front = false
	var do_rear = false

	if front_driven and rear_driven:
		do_front = true
		do_rear = true
	elif front_driven:
		do_front = true
	elif rear_driven:
		do_rear = true
	else:
		var per_wheel_torque = torque_at_wheels / EngineData.driven_count if EngineData.driven_count > 0 else 0.0
		for i in range(4):
			EngineData.wheel_engine_torque[i] = per_wheel_torque if EngineData.driven_wheels[i] else 0.0

	var both_driven = do_front and do_rear

	for axle_index in range(2):
		var active = do_front if axle_index == 0 else do_rear
		if not active:
			continue

		var idx_a = 0 if axle_index == 0 else 2
		var idx_b = 1 if axle_index == 0 else 3

		# awd behavior

		var axle_torque: float
		if both_driven:
			axle_torque = torque_at_wheels * Values.center_diff_split if axle_index == 0 else torque_at_wheels * (1.0 - Values.center_diff_split)
		else:
			axle_torque = torque_at_wheels

		# variable definitions

		var slip_a = WheelData.slip_ratio[idx_a]
		var slip_b = WheelData.slip_ratio[idx_b]

		var T_lock: float
		var T_high: float
		var T_low: float

		# type of lsd or open diff

		if Values.differential == Values.DiffType.TORSEN_LSD:
			if min(slip_a, slip_b) <= 0.0:
				T_high = axle_torque / 2.0
				T_low  = axle_torque / 2.0
			else:
				T_high = axle_torque * (Values.TBR / (Values.TBR + 1.0))
				T_low  = axle_torque * (1.0 / (Values.TBR + 1.0))
		elif Values.differential == Values.DiffType.CLUTCH_LSD:
			T_lock = Values.minimum_clutch_lsd_force + (axle_torque * Values.clutch_lsd_ramp_factor)
			T_lock = min(T_lock, axle_torque / 2.0)
			T_high = (axle_torque / 2.0) + T_lock
			T_low  = max((axle_torque / 2.0) - T_lock, 0.0)
		elif Values.differential == Values.DiffType.ELECTRONIC_LSD:
			pass # will write logic
		elif Values.differential == Values.DiffType.OPEN:
			T_high = axle_torque / 2.0
			T_low  = axle_torque / 2.0

		# actual torque division

		var diff = slip_a - slip_b
		if diff > Values.SLIP_THRESHOLD:
			EngineData.wheel_engine_torque[idx_a] = T_low
			EngineData.wheel_engine_torque[idx_b] = T_high
		elif -diff > Values.SLIP_THRESHOLD:
			EngineData.wheel_engine_torque[idx_a] = T_high
			EngineData.wheel_engine_torque[idx_b] = T_low
		else:
			EngineData.wheel_engine_torque[idx_a] = axle_torque / 2.0
			EngineData.wheel_engine_torque[idx_b] = axle_torque / 2.0
