extends Node

@export var car: RigidBody3D


var is_shifting = Data.is_shifting
var shift_timer = Data.shift_timer
var gear_ratio = Values.gear_ratio
var current_gear_ratio = Data.current_gear_ratio
var current_gear = Data.current_gear


func transmission_process(delta: float):
	var target_clutch = Input.get_action_strength("Clutch")


# upshift
	if not Data.is_shifting and target_clutch > 0.3:
		if Input.is_action_just_pressed("ShiftUp"):
			if Data.current_gear < gear_ratio.size() - 1:
				Data.current_gear += 1
				Data.is_shifting = true
				Data.shift_timer = 0.2  # realistic shift time in seconds

# down shift
		if Input.is_action_just_pressed("ShiftDown"):
			if Data.current_gear > 0:
				Data.current_gear -= 1
				Data.is_shifting = true
				Data.shift_timer = 0.2  # realistic shift time

# countdown system before another shift can be made
	if Data.is_shifting:
		Data.shift_timer -= delta
		if Data.shift_timer <= 0.0:
			Data.current_gear_ratio = gear_ratio[Data.current_gear]
			Data.is_shifting = false
