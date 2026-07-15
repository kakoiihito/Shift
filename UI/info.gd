extends Control

@onready var info_label: Label = $VBoxContainer/HBoxContainer/Label
@export var car: Node3D
var car_velocity: float

func _process(_delta: float) -> void:
	
	if car != null:
		# Speed
		var wheel_speed = car.wheeldata.wheel_angular_velocity[0] * car.VehicleValues.wheel_radius
		var speed = wheel_speed * 2.237
		
		# Gear
		var gear = car.transmission.current_gear - 1
		
		# RPM
		var engine_rpm = car.engine.engine_rpm
		
		
		info_label.text = "%d MPH | %d RPM | %d Gear" % [int(speed), int(engine_rpm), int(gear)]
