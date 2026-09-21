extends Node



func brake_process(delta: float, Data: RuntimeData.brake, Values: Resource) -> void:
	
	var target_brake = Input.get_action_strength("Brake")
	var brake_rate = 8.0 if target_brake > Data.input_brake else 12.0
	var input_brake = move_toward(Data.input_brake, target_brake, brake_rate * delta)
	
	active_brake_wheels_calc(Data, Values)
	brake_torque_calc(Data, Values, input_brake)
	
func active_brake_wheels_calc(Data: RuntimeData.brake, Values: Resource):
	
	var brake_wheels = [Values.FL_torque_brake, Values.FR_torque_brake, Values.RL_torque_brake, Values.RR_torque_brake]
	
	Data.active_wheels_brake = 0
	for i in range(4):
		if brake_wheels[i]:
			Data.active_wheels_brake += 1
	
	if Data.active_wheels_brake == 0:
		return

func brake_torque_calc(Data: RuntimeData.brake, Values: Resource, input_brake: float):
	
	var brake_wheels = [Values.FL_torque_brake, Values.FR_torque_brake, Values.RL_torque_brake, Values.RR_torque_brake]

	if input_brake <= 0.0:
		for i in range(4):
			Data.wheel_brake_torque[i] = 0.0
		return
	
	Data.brake_torque = (input_brake * Values.max_brake_torque) / Data.active_wheels_brake
	for i in range(4):
		if brake_wheels[i] and Data.abs_active[i] == false:
			Data.wheel_brake_torque[i] = Data.brake_torque
		else:
			Data.wheel_brake_torque[i] = 0.0
