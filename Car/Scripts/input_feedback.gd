extends Node

var longitude_force = Data.longitude_force
var lateral_force = Data.lateral_force
var wheel_spring_force = Data.wheel_spring_force
var friction_coeff = Values.friction_coefficient

var lateral_intensity: float
var longitude_intensity: float

var prev_lateral = 0.0
var prev_longitude = 0.0

func _input_feedback():
	var total_lateral = 0.0
	var total_longitude = 0.0
	
	for i in range(4):
		var spring_len = Data.wheel_spring_force[i].length()
		if spring_len > 0.0:
			total_lateral += abs(Data.lateral_force[i]) / (Values.friction_coefficient * spring_len)
			total_longitude += abs(Data.longitude_force[i]) / (Values.friction_coefficient * spring_len)
	
	lateral_intensity = total_lateral / 4.0
	longitude_intensity = total_longitude / 4.0
	
	var lateral_delta = abs(lateral_intensity - prev_lateral)
	var longitude_delta = abs(longitude_intensity - prev_longitude)
	
	prev_lateral = lateral_intensity
	prev_longitude = longitude_intensity
	
	var grip_intensity = clamp(lateral_delta * 9.0, 0.0, 1.0)
	var slip_intensity = clamp(longitude_delta * 9.0, 0.0, 1.0)
	
	Input.start_joy_vibration(0, grip_intensity, slip_intensity, 0.1)
