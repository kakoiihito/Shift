extends Node


func abs_proccess(delta: float, AssistsData: RuntimeData.assists, BrakeData: RuntimeData.brake, WheelData: RuntimeData.wheels, Values: Resource) -> void:

	for i in range(4):
		if not Values.ABS:
			AssistsData.abs_active[i] = false
			BrakeData.abs_input_brake[i] = BrakeData.input_brake
			continue

		var slip = WheelData.slip_ratio[i]

		if AssistsData.abs_active[i]:
			if slip < Values.abs_slip_apply:   # wheel has recovered
				AssistsData.abs_active[i] = false
		elif slip > Values.abs_slip_release:     # wheel slipping too much
			AssistsData.abs_active[i] = true

		var target = BrakeData.input_brake
		if AssistsData.abs_active[i]:
			target *= Values.abs_pedal_target

		var current: float = BrakeData.abs_input_brake[i]
		var rate: float = Values.abs_rate_release if target < current else Values.abs_rate_apply
		BrakeData.abs_input_brake[i] = move_toward(current, target, rate * delta)
		
func tc_proccess(delta: float, EngineData: RuntimeData.engine, WheelData: RuntimeData.wheels, Values: Resource) -> void:
	if Values.TC == true:
		for i in range(4):
			if clamp(WheelData.slip_ratio[i], -1.0, 1.0) > Values.tc_slip_threshold:
				EngineData.wheel_engine_torque[i] *= exp(Values.tc_decay_rate * delta)


	
func stability_proccess() -> void:
	pass # for the future. seems i need to do more research.
