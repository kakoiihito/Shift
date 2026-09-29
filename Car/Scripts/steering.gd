extends Node

func steering_proccess(delta: float, SteeringData: RuntimeData.steering, car: RigidBody3D, Values: Resource) -> void:
	
	steering_input_calc(Values, SteeringData)
	inner_outer_turn_calc(Values, SteeringData)
	ackermann_target_calc(Values, SteeringData)
	apply_wheel_steering(delta, Values, SteeringData, car)

func steering_input_calc(Values: Resource, SteeringData: RuntimeData.steering) -> void:
	
	SteeringData.input_turn = Input.get_action_strength("SteerLeft") - Input.get_action_strength("SteerRight")
	SteeringData.steering_amount = (SteeringData.input_turn * deg_to_rad(Values.steering_wheel_travel)) / Values.steering_ratio
	SteeringData.max_angle = deg_to_rad(Values.max_tire_turn_angle)


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


func ackermann_target_calc(Values: Resource, SteeringData: RuntimeData.steering) -> void:
	
	if SteeringData.steering_amount > 0.0:
		SteeringData.target_fr = lerp(SteeringData.steering_amount, SteeringData.inner, Values.ackermann_factor) # left
		SteeringData.target_fl = lerp(SteeringData.steering_amount, SteeringData.outer, Values.ackermann_factor)
	else:
		SteeringData.target_fl = lerp(SteeringData.steering_amount, -SteeringData.inner, Values.ackermann_factor)
		SteeringData.target_fr = lerp(SteeringData.steering_amount, -SteeringData.outer, Values.ackermann_factor) # r

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
