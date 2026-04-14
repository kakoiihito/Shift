extends Node

@export var car: RigidBody3D
@onready var Values: Resource


	
func steering_proccess(delta: float) -> void:
	
	var input_turn = Input.get_action_strength("SteerLeft") - Input.get_action_strength("SteerRight")
	var steering_amount = input_turn * Values.max_tire_turn_angle

	if abs(input_turn) < 0.01:
		var target = clamp(steering_amount, -Values.max_tire_turn_angle, Values.max_tire_turn_angle)
		car.fl_wheel.rotation.y = move_toward(car.fl_wheel.rotation.y, target, Values.tire_turn_speed * delta)
		car.fr_wheel.rotation.y = move_toward(car.fr_wheel.rotation.y, target, Values.tire_turn_speed * delta)
		return
	

	var L = Values.wheel_base
	var W = Values.track
	var R = L / tan(abs(steering_amount))

	var inner = atan(L / (R - W * 0.5))
	var outer = atan(L / (R + W * 0.5))
	
	var target_fl := 0.0
	var target_fr := 0.0

	if steering_amount > 0.0:
		# Turning left
		target_fr = -inner
		target_fl = -outer
	else:
		# Turning right
		target_fl = inner
		target_fr = outer
		
	car.fl_wheel.rotation.y = move_toward(
		car.fl_wheel.rotation.y,
		target_fl,
		Values.tire_turn_speed * delta
	)
	car.fr_wheel.rotation.y = move_toward(
		car.fr_wheel.rotation.y,
		target_fr,
		Values.tire_turn_speed * delta
	)
