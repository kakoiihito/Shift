## Front wheel steering with Ackermann geometry.
##
## This script turns the player's steering input into target steering angles for
## the two front wheels, then moves the wheel nodes toward those angles at a
## limited speed. It keeps no state of its own: intermediate results are stored
## in the shared steering runtime data, and the settings come from the car
## tuning resource.
##
## [b]Pipeline (see [method steering_proccess]):[/b][br]
## 1. Read the input and convert it to a steering angle, limited to the maximum
## tire angle.[br]
## 2. Compute the ideal inner and outer wheel angles for that turn.[br]
## 3. Blend between parallel steering and full Ackermann, and assign the angles
## to the front left and front right wheels.[br]
## 4. Rotate the wheel nodes toward their target angles.
##
## [b]Input actions used:[/b] [code]"SteerLeft"[/code] and
## [code]"SteerRight"[/code] (each 0..1).
##
## [b]Units:[/b] all angles inside this script are in radians. The tuning values
## ([code]steering_wheel_travel[/code], [code]max_tire_turn_angle[/code]) are
## stored in degrees and converted here. A positive angle is a turn to the left.
##
## [b]Requirements for the [code]car[/code] node:[/b] it must have the members
## [code]fl_wheel[/code] and [code]fr_wheel[/code], which are the front left and
## front right wheel nodes whose [code]rotation.y[/code] is the steering angle.
extends Node


## Runs one step of the steering simulation.
##
## Call this once per physics frame. It updates the steering runtime data
## ([code]input_turn[/code], [code]max_angle[/code], [code]steering_amount[/code],
## [code]inner[/code], [code]outer[/code], [code]target_fl[/code],
## [code]target_fr[/code]) and rotates the front wheel nodes on [param car].
##
## [param delta] is the time step in seconds.[br]
## [param SteeringData] is the runtime steering state (read and written).[br]
## [param car] is the car body that owns the front wheel nodes
## ([code]fl_wheel[/code] and [code]fr_wheel[/code]).[br]
## [param Values] is the car tuning resource.
##
func steering_proccess(delta: float, SteeringData: RuntimeData.steering, car: RigidBody3D, Values: Resource) -> void:
	
	steering_input_calc(Values, SteeringData)
	inner_outer_turn_calc(Values, SteeringData)
	ackermann_target_calc(Values, SteeringData)
	apply_wheel_steering(delta, Values, SteeringData, car)


## Reads the steering input and converts it to a steering angle.
##
## Sets these fields on [param SteeringData]:[br]
## - [code]input_turn[/code]: [code]SteerLeft - SteerRight[/code], from -1.0
## (full right) to 1.0 (full left).[br]
## - [code]max_angle[/code]: [code]max_tire_turn_angle[/code] converted to
## radians.[br]
## - [code]steering_amount[/code]: the requested tire angle in radians. It is the
## steering wheel travel ([code]steering_wheel_travel[/code], converted from
## degrees) scaled by the input and divided by [code]steering_ratio[/code], then
## clamped to the range [code]-max_angle[/code] to [code]max_angle[/code].
##
## [code]max_angle[/code] is assigned before the clamp so that the clamp uses the
## current tuning value.
func steering_input_calc(Values: Resource, SteeringData: RuntimeData.steering) -> void:
	
	SteeringData.input_turn = Input.get_action_strength("SteerLeft") - Input.get_action_strength("SteerRight")
	SteeringData.max_angle = deg_to_rad(Values.max_tire_turn_angle)
	SteeringData.steering_amount = clampf(
		(SteeringData.input_turn * deg_to_rad(Values.steering_wheel_travel)) / Values.steering_ratio,
		-SteeringData.max_angle,
		SteeringData.max_angle
	)


## Calculates the ideal Ackermann angles for the inner and outer front wheel.
##
## In a turn the inner wheel (closer to the turn center) must steer more than the
## outer wheel so that both wheels follow circles around the same center. The
## requested [code]steering_amount[/code] is treated as the angle of an imaginary
## center wheel, which gives a turn radius of
## [code]R = wheel_base / tan(|steering_amount|)[/code]. The wheel angles are then
## [code]atan(wheel_base / (R - track / 2))[/code] for the inner wheel and
## [code]atan(wheel_base / (R + track / 2))[/code] for the outer wheel.
##
## Sets [code]SteeringData.inner[/code] and [code]SteeringData.outer[/code] as
## positive angles in radians, regardless of turn direction. When the steering
## amount is almost zero (less than 0.0001 rad) both are set to 0.0 to avoid
## dividing by zero.
func inner_outer_turn_calc(Values: Resource, SteeringData: RuntimeData.steering) -> void:
	
	if abs(SteeringData.steering_amount) < 0.0001:
		SteeringData.inner = 0.0
		SteeringData.outer = 0.0
		return

	var L = Values.wheel_base
	var W = Values.track
	var R = L / tan(abs(SteeringData.steering_amount))

	SteeringData.inner = atan(L / (R - W * 0.5))
	SteeringData.outer = atan(L / (R + W * 0.5))


## Chooses the target steering angle of each front wheel.
##
## Each wheel's target is a blend between the plain requested angle
## ([code]steering_amount[/code], both wheels parallel) and its ideal Ackermann
## angle from [method inner_outer_turn_calc]. The blend amount is
## [code]ackermann_factor[/code]: 0.0 gives parallel steering, 1.0 gives full
## Ackermann.
##
## The sign of [code]steering_amount[/code] selects the turn direction, and the
## inner and outer angles are negated for turns in the negative direction.
## Results are stored in [code]SteeringData.target_fl[/code] and
## [code]SteeringData.target_fr[/code] in radians.
##
func ackermann_target_calc(Values: Resource, SteeringData: RuntimeData.steering) -> void:
	
	if SteeringData.steering_amount > 0.0:
		SteeringData.target_fr = lerp(SteeringData.steering_amount, SteeringData.inner, Values.ackermann_factor) # left
		SteeringData.target_fl = lerp(SteeringData.steering_amount, SteeringData.outer, Values.ackermann_factor)
	else:
		SteeringData.target_fl = lerp(SteeringData.steering_amount, -SteeringData.inner, Values.ackermann_factor)
		SteeringData.target_fr = lerp(SteeringData.steering_amount, -SteeringData.outer, Values.ackermann_factor) # r

## Rotates the front wheel nodes toward their target steering angles.
##
## Each wheel's [code]rotation.y[/code] moves toward its target
## ([code]target_fl[/code] or [code]target_fr[/code]) by at most
## [code]tire_turn_speed * delta[/code] radians per call, which makes the wheels
## turn smoothly instead of snapping to the target.
##
## [param delta] is the time step in seconds.[br]
## [param car] must provide [code]fl_wheel[/code] and [code]fr_wheel[/code].
func apply_wheel_steering(delta: float, Values: Resource, SteeringData: RuntimeData.steering, car: RigidBody3D) -> void:
	
	car.fl_wheel.rotation.y = move_toward(
		car.fl_wheel.rotation.y,
		SteeringData.target_fl,
		Values.tire_turn_speed * delta
	)
	car.fr_wheel.rotation.y = move_toward(
		car.fr_wheel.rotation.y,
		SteeringData.target_fr,
		Values.tire_turn_speed * delta
	)
