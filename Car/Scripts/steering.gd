extends Node

@export var car: RigidBody3D
@onready var Values: Resource

var speed_factor_coeff = 0.03
var ackermann_factor = 0.8 
var target_fl := 0.0
var target_fr := 0.0
func steering_proccess(delta: float) -> void:
	
	var input_turn = Input.get_action_strength("SteerLeft") - Input.get_action_strength("SteerRight")
	var speed = car.linear_velocity.length()
	var speed_factor = 1.0 / (1.0 + pow(speed * speed_factor_coeff, 2))
	var steering_amount = (input_turn * deg_to_rad(Values.max_tire_turn_angle) * speed_factor) / Values.steering_ratio
	
	var max_angle = deg_to_rad(Values.max_tire_turn_angle)
	var mz_to_angle = 1.0 / (Values.steering_stiffness * Values.steering_ratio)
	

	var L = Values.wheel_base
	var W = Values.track
	var R = L / tan(abs(steering_amount))

	var inner = atan(L / (R - W * 0.5))
	var outer = atan(L / (R + W * 0.5))
		

	if steering_amount > 0.0:
		target_fr = lerp(steering_amount, inner, ackermann_factor)
		target_fl = lerp(steering_amount, outer, ackermann_factor)
	else:
		target_fl = lerp(steering_amount, -inner, ackermann_factor)
		target_fr = lerp(steering_amount, -outer, ackermann_factor)
	
	target_fl = clamp(target_fl - Data.aligning_torque[0] * mz_to_angle, -max_angle, max_angle) * 0.8
	target_fr = clamp(target_fr - Data.aligning_torque[1] * mz_to_angle, -max_angle, max_angle) * 0.8
		
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
