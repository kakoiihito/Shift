## Brake pedal smoothing and brake torque distribution.
##
## This script turns the player's brake input into a braking torque for each
## wheel. The pedal value is smoothed over time so the brakes do not snap on or
## off, the total brake torque is shared evenly between the wheels that have
## brakes, and any wheel that the ABS has released gets no torque. It keeps no
## state of its own: everything is stored in the brake runtime data, and the
## settings come from the car tuning resource.
##
## [b]Pipeline (see [method brake_process]):[/b][br]
## 1. Smooth the brake pedal input.[br]
## 2. Count the wheels that have brakes.[br]
## 3. Split the brake torque between those wheels, skipping wheels where the ABS
## is active.
##
## [b]Input actions used:[/b] [code]"Brake"[/code] (pedal, 0..1).
##
## [b]Wheel index order[/b] used by the per-wheel arrays: 0 = front left, 1 =
## front right, 2 = rear left, 3 = rear right.
##
## [b]Runtime data fields used:[/b] [code]input_brake[/code],
## [code]active_wheels_brake[/code], [code]brake_torque[/code],
## [code]wheel_brake_torque[/code] (one float per wheel) and
## [code]abs_active[/code] (one bool per wheel, set by the ABS logic elsewhere).
##
## [b]Units:[/b] torque in N·m.
extends Node


## Runs one step of the brake simulation.
##
## Call this once per physics frame. It smooths the pedal input, then updates
## the number of braked wheels and the torque for each wheel.
##
## The smoothed pedal value [code]Data.input_brake[/code] moves toward the raw
## [code]"Brake"[/code] input at a limited rate: 8.0 per second when the pedal is
## being pressed harder and 12.0 per second when it is being released. With
## those rates the brakes take about 0.125 seconds to go from fully released to
## fully applied, and about 0.083 seconds to release again. Releasing is faster
## than applying.
##
## [param delta] is the time step in seconds.[br]
## [param Data] is the runtime brake state (read and written).[br]
## [param Values] is the car tuning resource.
func brake_process(delta: float, BrakeData: RuntimeData.brake, AssistsData: RuntimeData.assists, Values: Resource) -> void:
	
	var target_brake = Input.get_action_strength("Brake")
	var brake_rate = 8.0 if target_brake > BrakeData.input_brake else 12.0
	BrakeData.input_brake = move_toward(BrakeData.input_brake, target_brake, brake_rate * delta)
	
	active_brake_wheels_calc(BrakeData, Values)
	brake_torque_calc(BrakeData, AssistsData, Values)
	

## Counts how many wheels have brakes and stores the result in
## [code]Data.active_wheels_brake[/code].
##
## A wheel counts when its flag in [param Values]
## ([code]FL_torque_brake[/code], [code]FR_torque_brake[/code],
## [code]RL_torque_brake[/code], [code]RR_torque_brake[/code]) is true.
##
func active_brake_wheels_calc(BrakeData: RuntimeData.brake, Values: Resource):
	
	var brake_wheels = [Values.FL_torque_brake, Values.FR_torque_brake, Values.RL_torque_brake, Values.RR_torque_brake]
	
	BrakeData.active_wheels_brake = 0
	for i in range(4):
		if brake_wheels[i]:
			BrakeData.active_wheels_brake += 1
	
	if BrakeData.active_wheels_brake == 0:
		pass


## Calculates the brake torque for each wheel and stores it in
## [code]Data.wheel_brake_torque[/code].
##
## The total brake torque is the smoothed pedal value multiplied by
## [code]max_brake_torque[/code]. It is divided evenly by the number of braked
## wheels ([code]active_wheels_brake[/code]) and stored in
## [code]Data.brake_torque[/code] as the torque for one wheel. So
## [code]max_brake_torque[/code] is the total for the whole car, not the value per
## wheel.
##
## Each wheel then receives that torque if it has brakes and the ABS is not
## active on it ([code]abs_active[i][/code] is false). Wheels without brakes, and
## wheels where the ABS is active, receive 0.0. Torque shared out to an
## ABS-released wheel is not given to the other wheels.
##
## When the pedal is not pressed ([code]input_brake <= 0[/code]), every wheel gets
## 0.0 and the function returns early, so [code]Data.brake_torque[/code] keeps its
## previous value.
##
func brake_torque_calc(BrakeData: RuntimeData.brake, AssistsData: RuntimeData.assists, Values: Resource):
	
	var brake_wheels = [Values.FL_torque_brake, Values.FR_torque_brake, Values.RL_torque_brake, Values.RR_torque_brake]
	
	if BrakeData.input_brake <= 0.0:
		for i in range(4):
			BrakeData.wheel_brake_torque[i] = 0.0
		return
		
	for i in range(4):
		if AssistsData.abs_active[i] != true:
			BrakeData.brake_torque = (BrakeData.input_brake * Values.max_brake_torque) / BrakeData.active_wheels_brake
		elif AssistsData.abs_active[i] == true:
			BrakeData.brake_torque = (BrakeData.abs_input_brake[i] * Values.max_brake_torque) / BrakeData.active_wheels_brake
			
	for i in range(4):
		if brake_wheels[i]:
			BrakeData.wheel_brake_torque[i] = BrakeData.brake_torque
