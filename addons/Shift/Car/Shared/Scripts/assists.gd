extends Node


func abs_proccess(delta: float, AssistsData: RuntimeData.assists, BrakeData: RuntimeData.brake, WheelData: RuntimeData.wheels, Values: Resource) -> void:
	
	var channel_type: Array = []
	
	if Values.current_abs_mode == Values.ABSType.ONE_CHANNEL:
		channel_type = [[0, 1], [2, 3]]
	if Values.current_abs_mode == Values.ABSType.TWO_CHANNEL:
		channel_type = [[0, 1, 2, 3]]
	if Values.current_abs_mode == Values.ABSType.THREE_CHANNEL:
		channel_type = [[0], [1], [2, 3]]
	if Values.current_abs_mode == Values.ABSType.FOUR_CHANNEL:
		channel_type = [[0], [1], [2], [3]]
		
	for i in range(4):
		if not Values.ABS:
			AssistsData.abs_active[i] = false
			AssistsData.abs_input_brake[i] = BrakeData.input_brake
			continue

	for channel in channel_type:

		var slip = -INF
		for w in channel:
			slip = maxf(slip, WheelData.slip_ratio[w])

		var active = AssistsData.abs_active[channel[0]]
		if active:
			if slip < Values.abs_slip_apply:      # channel has recovered
				active = false
		elif slip > Values.abs_slip_release:      # channel slipping too much
			active = true

		var target = BrakeData.input_brake
		if active:
			target *= Values.abs_pedal_target

		for w in channel:
			AssistsData.abs_active[w] = active

			var current = AssistsData.abs_input_brake[w]
			var rate = Values.abs_rate_release if target < current else Values.abs_rate_apply
			AssistsData.abs_input_brake[w] = move_toward(current, target, rate * delta)

		
func tc_proccess(delta: float, AssistsData: RuntimeData.assists, BrakeData: RuntimeData.brake, EngineData: RuntimeData.engine, WheelData: RuntimeData.wheels, Values: Resource) -> void:

	if not Values.TC:
		for i in range(4):
			AssistsData.tc_active[i] = false
			AssistsData.tc_factor[i] = 1.0
			AssistsData.tc_input_brake[i] = 0.0
		return

	if Values.current_tc_type == Values.TCTypes.TORQUE_VECTORING:

		for i in range(4):
			var slip = WheelData.slip_ratio[i]
			_tc_engine_step(i, slip, delta, AssistsData, Values)
			_tc_brake_step(i, delta, AssistsData, BrakeData, EngineData, Values)
			print("factor ", AssistsData.tc_factor)
			print("brake ", AssistsData.tc_input_brake)
			
			EngineData.wheel_engine_torque[i] *= AssistsData.tc_factor[i]

	if Values.current_tc_type == Values.TCTypes.WORST_SLIP:

		var worst = -1.0
		for i in range(4):
			if absf(EngineData.wheel_engine_torque[i]) > 0.0:
				worst = maxf(worst, WheelData.slip_ratio[i])

		_tc_engine_step(0, worst, delta, AssistsData, Values)

		for i in range(4):
			AssistsData.tc_active[i] = AssistsData.tc_active[0]
			AssistsData.tc_factor[i] = AssistsData.tc_factor[0]
			EngineData.wheel_engine_torque[i] *= AssistsData.tc_factor[i]


func _tc_engine_step(i: int, slip: float, delta: float, AssistsData: RuntimeData.assists, Values: Resource) -> void:
	if AssistsData.tc_active[i]:
		if slip < Values.tc_slip_apply:      # recovered
			AssistsData.tc_active[i] = false
	elif slip > Values.tc_slip_release:         # spinning too much
		AssistsData.tc_active[i] = true
		
	if Values.tc_use_engine_cut == true:
		var target = Values.tc_torque_factor if AssistsData.tc_active[i] else 1.0
		var current = AssistsData.tc_factor[i]
		var rate = Values.tc_rate_release if target < current else Values.tc_rate_apply
		AssistsData.tc_factor[i] = move_toward(current, target, rate * delta)
	else:
		AssistsData.tc_factor[i] = 1.0

func _tc_brake_step(i: int, delta: float, AssistsData: RuntimeData.assists, BrakeData: RuntimeData.brake, EngineData: RuntimeData.engine, Values: Resource) -> void:
	var driven = absf(EngineData.wheel_engine_torque[i]) > 0.0

	var target: float = 0.0
	if Values.tc_use_brakes and AssistsData.tc_active[i] and driven and BrakeData.input_brake < 0.05:
		target = Values.tc_brake_pedal_target

	var current= AssistsData.tc_input_brake[i]
	var rate = Values.tc_brake_rate_apply if target > current else Values.tc_brake_rate_release
	AssistsData.tc_input_brake[i] = move_toward(current, target, rate * delta)
	
func stability_proccess() -> void:
	pass # for the future. seems i need to do more research.
