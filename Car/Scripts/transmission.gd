extends Node

var is_shifting = false
var shift_timer = 0.0
var drive_train_efficeny = 0.9
var final_drive = 3.63 # Final gear to multiple torque.
var gear_ratio = [-3.27, 0.0, 3.64, 2.6, 1.53, 1.16, 0.94]   # power multiplyer for engine
var current_gear_ratio: float
var current_gear = 1

func transmission_process(delta: float):
	var target_clutch = Input.get_action_strength("Clutch")


	if not is_shifting and target_clutch > 0.3:
		if Input.is_action_just_pressed("ShiftUp"):
			if current_gear < gear_ratio.size() - 1:
				current_gear += 1
				is_shifting = true
				shift_timer = 0.2  # realistic shift time in seconds

# down shift
		if Input.is_action_just_pressed("ShiftDown"):
			if current_gear > 0:
				current_gear -= 1
				is_shifting = true
				shift_timer = 0.2  # realistic shift time

# countdown system before another shift can be made
	if is_shifting:
		shift_timer -= delta
		if shift_timer <= 0.0:
			current_gear_ratio = gear_ratio[current_gear]
			is_shifting = false
