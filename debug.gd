extends Control
@export var car: RigidBody3D
@onready var label = $VBoxContainer/HBoxContainer/Label
func _ready() -> void:
	pass
func _process(_delta: float) -> void:
	if car == null or label == null:
		return
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
	
	label.text = "RPM: %.0f\nClutch Torque: %.2f\nEngine Torque: %.2f\nGear: %d\nClutch Engagement: %.2f\nThrottle: %.2f" % [rpm, clutch_torque, engine_torque, current_gear, clutch_engagement, throttle_input]
