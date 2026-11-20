extends Node


func abs_proccess(delta: float, AssistsData: RuntimeData.assists, BrakeData: RuntimeData.brake, WheelData: RuntimeData.wheels, Values: Resource) -> void:

	for i in range(4):
		if not Values.ABS:
			AssistsData.abs_active[i] = false
			BrakeData.abs_input_brake[i] = BrakeData.input_brake
			continue

		var slip = WheelData.slip_ratio[i]

		if AssistsData.abs_active[i]:
			if slip > Values.abs_slip_apply:   # wheel has recovered
				AssistsData.abs_active[i] = false
		elif slip < Values.abs_slip_release:     # wheel slipping too much
			AssistsData.abs_active[i] = true

		var target = BrakeData.input_brake
		if AssistsData.abs_active[i]:
			target *= Values.abs_pedal_target

		var current: float = BrakeData.abs_input_brake[i]
		var rate: float = Values.abs_rate_release if target < current else Values.abs_rate_apply
		BrakeData.abs_input_brake[i] = move_toward(current, target, rate * delta)
		
func tc_proccess(delta: float, AssistsData: RuntimeData.assists, EngineData: RuntimeData.engine, BrakeData: RuntimeData.brake, WheelData: RuntimeData.wheels, Values: Resource) -> void:
	for i in range(4):
		if not Values.TC:
			AssistsData.tc_active[i] = false
			continue
			
		var slip = WheelData.slip_ratio[i]
		
		if AssistsData.tc_active[i]:
			if slip > Values.tc_slip_apply:
				AssistsData.tc_active[i] = false
		elif slip < Values.tc_slip_release: 
			AssistsData.tc_active[i] = true
			
		var target = BrakeData.input_brake
		if Values.tc_brake == true:
			if AssistsData.tc_active[i]:
				target *= Values.tc_pedal_target
				
	
func stability_proccess() -> void:
	pass # for the future. seems i need to do more research.
