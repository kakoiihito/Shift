## Engine, clutch and drivetrain torque simulation for a vehicle.
##
## This script runs the powertrain half of a vehicle simulation each physics
## frame. It does not own any state itself; everything it reads and writes
## lives in the shared runtime data objects and the tuning [Resource] that are
## passed in.
##
## [b]Pipeline (see [method motor_process]):[/b][br]
## 1. Count how many wheels receive engine torque.[br]
## 2. Compute the torque the engine produces from the throttle and torque curve.[br]
## 3. Compute internal engine friction.[br]
## 4. Compute the torque transferred through the clutch.[br]
## 5. Integrate engine angular velocity and derive RPM.[br]
## 6. Convert clutch torque to wheel torque and split it between wheels
## (center split for AWD, then open / Torsen / clutch-type LSD per axle).
##
## [b]Units:[/b] torque in N·m, angular velocity in rad/s, inertia in kg·m²,
## RPM in revolutions per minute, [code]delta[/code] in seconds.
##
## [b]Input actions used:[/b] [code]"Gas"[/code] (throttle, 0..1) and
## [code]"Clutch"[/code] (clutch pedal, 0..1, where 1 is fully pressed/disengaged).
##
## [b]Wheel index order[/b] used by all per-wheel arrays: 0 = front left (FL),
## 1 = front right (FR), 2 = rear left (RL), 3 = rear right (RR).
extends Node


## Runs one simulation step of the whole engine and drivetrain model.
##
## Call this once per physics frame. It updates [param EngineData] in place:
## [code]engine_driven_count[/code], [code]engine_torque[/code],
## [code]clutch_torque_on_engine[/code], [code]engine_angular_velocity[/code],
## [code]engine_rpm[/code] and [code]wheel_engine_torque[/code].
## [param TransmissionData] and [param WheelData] are only read.
##
## [param delta] is the time step in seconds.[br]
## [param EngineData] is the runtime engine state (read and written).[br]
## [param TransmissionData] is the runtime gearbox state (current gear, gear
## ratio, shifting flag).[br]
## [param WheelData] is the runtime wheel state (angular velocities and slip
## ratios).[br]
## [param Values] is the vehicle tuning resource (torque curve, inertias,
## final drive, differential type and so on).
##
func motor_process(delta: float, EngineData: RuntimeData.engine, TransmissionData: RuntimeData.transmission, WheelData: RuntimeData.wheels, Values: Resource) -> void:
	
	# driven count calc
	
	driven_count_calc(Values, EngineData)
	
	# engine torque calculation
	
	engine_torque_calc(EngineData, Values)
	
	# engine friciton calculation
	
	engine_friction_calc(EngineData, Values)
	
	# clutch torque calculation
		
	clutch_torque_calc(TransmissionData, WheelData, EngineData, Values, delta)
			
	# engine rpm calculation

	engine_rpm_calc(EngineData, Values, delta)
	
	# engine stalling
	
	# removed for the time being
	
	#Torque division (lsds, open diff)
	
	wheel_torque_divison(TransmissionData, EngineData, WheelData, Values)
	

## Counts how many wheels are driven by the engine and stores the result in
## [code]EngineData.engine_driven_count[/code].
##
## A wheel counts as driven when its flag in [param Values]
## ([code]FL_torque_engine[/code], [code]FR_torque_engine[/code],
## [code]RL_torque_engine[/code], [code]RR_torque_engine[/code]) is true.
func driven_count_calc(Values: Resource, EngineData: RuntimeData.engine):
	
	var driven_wheels = [Values.FL_torque_engine, Values.FR_torque_engine, Values.RL_torque_engine, Values.RR_torque_engine]
	var driven_count = 0

	for i in range(4):
		if driven_wheels[i]:
			driven_count += 1

	EngineData.engine_driven_count = driven_count


## Sums the angular velocities of all driven wheels.
##
## Only wheels flagged as driven in [param Values] contribute. Divide the
## result by the driven wheel count to get the average driven wheel speed.
## Reads [code]WheelData.wheel_angular_velocity[/code].
##
## Returns the sum of driven wheel angular velocities in rad/s.
func angular_velocity_sum_calc(WheelData: RuntimeData.wheels, Values: Resource):
	
	var driven_wheels = [Values.FL_torque_engine, Values.FR_torque_engine, Values.RL_torque_engine, Values.RR_torque_engine]

	var angular_velocity_sum: float = 0.0
	
	for i in range(4):
		if driven_wheels[i]:
			angular_velocity_sum += WheelData.wheel_angular_velocity[i]

	
	return angular_velocity_sum
	

## Calculates the torque the engine produces this frame and stores it in
## [code]EngineData.engine_torque[/code].
##
## The result is the torque curve sampled at the normalized RPM
## ([code]engine_rpm / max_rpm[/code]), scaled by [code]max_torque[/code] and by
## the throttle position read from the [code]"Gas"[/code] input action.
##
func engine_torque_calc(EngineData: RuntimeData.engine, Values: Resource):
	
	var throttle_input = Input.get_action_strength("Gas")
	
	var normalized_rpm = EngineData.engine_rpm / Values.max_rpm

	EngineData.engine_torque = Values.torque_curve.sample(normalized_rpm) * Values.max_torque * throttle_input
	

## Calculates the internal friction torque of the engine at its current RPM.
##
## Friction is modeled as a quadratic polynomial of the normalized RPM
## ([code]engine_rpm / max_rpm[/code]):
## [code]friction_c0 + friction_c1 * n + friction_c2 * n²[/code].
##
## Returns the friction torque in N·m.
##
## [b]Note:[/b] the returned value is currently discarded by
func engine_friction_calc(EngineData: RuntimeData.engine, Values: Resource):
	
	
	var normalized_rpm = EngineData.engine_rpm / Values.max_rpm
	var base_friction = Values.friction_c0                      
	var linear_friction = Values.friction_c1 * normalized_rpm       
	var quadratic_friction = Values.friction_c2 * normalized_rpm * normalized_rpm
	return base_friction + linear_friction + quadratic_friction
	
	
## Calculates the torque the clutch transfers between engine and drivetrain and
## stores it in [code]EngineData.clutch_torque_on_engine[/code].
##
## The clutch is modeled as a friction element with a torque limit. The
## function computes the torque that would be needed to bring the engine and the
## wheels (reflected through the gearing) to a common speed within one time step
## (a "full lock" condition), then clamps it to the maximum the clutch can
## transmit: [code]max_clutch_torque * (1 - Clutch)[/code]. A fully pressed
## clutch pedal therefore transmits no torque, and a partially pressed pedal
## slips once the required torque exceeds the limit.
##
## The reported torque is positive when the clutch is slowing the engine down
## (engine faster than the driveline).
##
## The clutch torque is set to zero, and no calculation is done, when:[br]
## - the gearbox is shifting,[br]
## - the current gear is [code]1[/code] (treated as neutral by this code),[br]
## - the total drivetrain ratio is effectively zero,[br]
## - no wheels are driven.
##
## [param delta] is the time step in seconds.
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
	
	var clutch_input := 1.0 - Input.get_action_strength("Clutch")
	var max_transferable_torque = Values.max_clutch_torque * clutch_input

	var angular_velocity_sum = angular_velocity_sum_calc(WheelData, Values)
	var clutch_ang_vel = (angular_velocity_sum / EngineData.engine_driven_count) * drivetrain_ratio
	
	var reflected_inertia = (Values.wheel_inertia / (drivetrain_ratio * drivetrain_ratio)) * EngineData.engine_driven_count
	var combined_inertia = Values.engine_inertia + reflected_inertia
	var total_angular_momentum = (EngineData.engine_angular_velocity * Values.engine_inertia) + (clutch_ang_vel * reflected_inertia)
	var full_lock_ang_vel = total_angular_momentum / combined_inertia
	var required_torque = (Values.engine_inertia * (EngineData.engine_angular_velocity - full_lock_ang_vel)) / delta
	var clutch_torque = clampf(required_torque, -1 * max_transferable_torque, max_transferable_torque)
	
	EngineData.clutch_torque_on_engine = clutch_torque


## Integrates the engine's angular velocity and updates its RPM.
##
## Net torque is the engine torque minus the clutch torque acting on the
## engine. That torque is divided by [code]engine_inertia[/code] to get angular
## acceleration, which is integrated over [param delta] (explicit Euler).
##
## Behavior details:[br]
## - At or above [code]max_rpm[/code] the engine's own torque is cut (rev
## limiter), leaving only the clutch torque.[br]
## - Negative angular velocity is clamped to zero, so the engine never spins
## backwards.[br]
## - [code]EngineData.engine_rpm[/code] is derived from the new angular velocity
## with [code]rpm = ω * 60 / TAU[/code].
func engine_rpm_calc(EngineData: RuntimeData.engine, Values: Resource, delta: float):
	
	var engine_friction = engine_friction_calc(EngineData, Values)
	var throttle_input = Input.get_action_strength("Gas")
	
	var net_engine_torque = EngineData.engine_torque - ((1.0 - throttle_input) * engine_friction) - (EngineData.clutch_torque_on_engine)
	
	if EngineData.engine_angular_velocity >= Values.max_rpm * TAU / 60.0:
		net_engine_torque = -EngineData.clutch_torque_on_engine
		
	if EngineData.engine_angular_velocity < 0.0:
		EngineData.engine_angular_velocity = 0.0
	
	var engine_angular_accel = net_engine_torque / Values.engine_inertia
	EngineData.engine_angular_velocity += engine_angular_accel * delta
	
	EngineData.engine_rpm = EngineData.engine_angular_velocity * 60.0 / TAU
		

## Unfinished.
func wheel_torque_divison(TransmissionData: RuntimeData.transmission, EngineData: RuntimeData.engine, WheelData: RuntimeData.wheels, Values: Resource):
	var driven_wheels = [Values.FL_torque_engine, Values.FR_torque_engine, Values.RL_torque_engine, Values.RR_torque_engine]
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
		var per_wheel_torque = torque_at_wheels / EngineData.engine_driven_count if EngineData.engine_driven_count > 0 else 0.0
		for i in range(4):
			EngineData.wheel_engine_torque[i] = per_wheel_torque if driven_wheels[i] else 0.0

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
			var sign_val: float = 1.0 if axle_torque >= 0.0 else -1.0
			var abs_axle: float = abs(axle_torque)
			T_lock = Values.minimum_clutch_lsd_force + (abs_axle * Values.clutch_lsd_ramp_factor)
			T_lock = min(T_lock, abs_axle / 2.0)
			T_high = sign_val * ((abs_axle / 2.0) + T_lock)
			T_low  = sign_val * max((abs_axle / 2.0) - T_lock, 0.0)
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
