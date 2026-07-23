extends Control
@export var car: RigidBody3D
@export var force_update_interval: float = 0.2
@onready var label = $VBoxContainer/HBoxContainer/Label

var _time_since_force_update := 0.0
var _rounded_forces := []
var _max_traction_forces := []
var _traction_force := 0.0

func _ready() -> void:
	pass

func _process(delta: float) -> void:
	if car == null or label == null:
		return
	
	_update_forces(delta)
	_update_label()

func _update_forces(delta: float) -> void:
	_time_since_force_update += delta
	if _time_since_force_update >= force_update_interval:
		_time_since_force_update = 0.0
		var wheels = car.wheeldata
		var longitude_force = wheels.longitude_force
		_rounded_forces.clear()
		for f in longitude_force:
			_rounded_forces.append(snappedf(f, 0.01))
		
		_update_max_traction_forces()
		_update_traction_force()

func _update_max_traction_forces() -> void:
	var suspension = car.suspension
	_max_traction_forces.clear()
	for i in range(suspension.wheel_spring_force.size()):
		var spring_force_mag = suspension.wheel_spring_force[1].length()
		var max_force = car.VehicleValues.brush_mu * spring_force_mag
		_max_traction_forces.append(snappedf(max_force, 0.01))
		
func _update_traction_force() -> void:
	var engine = car.engine
	var transmission = car.transmission
	
	var gear_ratio = car.VehicleValues.gear_ratio[transmission.current_gear]
	var final_drive = car.VehicleValues.final_drive
	var wheel_radius = 0.3
	
	var force = (engine.clutch_torque_on_engine * gear_ratio * final_drive) / wheel_radius
	_traction_force = snappedf(force, 0.01)

func _update_label() -> void:
	var engine = car.engine
	var transmission = car.transmission
	
	var rpm = engine.engine_rpm
	var clutch_torque = engine.clutch_torque_on_engine
	var engine_torque = engine.engine_torque
	var current_gear = transmission.current_gear
	
	var clutch_input := Input.get_action_strength("Clutch")
	var normalized = clamp((1.0 - clutch_input - 0.3) / 0.4, 0.0, 1.0)
	var clutch_engagement = normalized * normalized * (3.0 - 2.0 * normalized)
	
	var throttle_input = Input.get_action_strength("Gas")
	
	label.text = "RPM: %.0f\nClutch Torque: %.2f\nEngine Torque: %.2f\nGear: %d\nClutch Engagement: %.2f\nThrottle: %.2f\nLongitude Force: %s\nMax Traction Force: %s\nTraction Force: %.2f" % [rpm, clutch_torque, engine_torque, current_gear, clutch_engagement, throttle_input, str(_rounded_forces), str(_max_traction_forces), _traction_force]
