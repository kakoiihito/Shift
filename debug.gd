extends Control
@export var car: RigidBody3D
@onready var label = $VBoxContainer/HBoxContainer/Label
func _ready() -> void:
	pass
func _process(_delta: float) -> void:
	if car == null or label == null:
		return
	var engine = car.engine
	var rpm = engine.engine_rpm
	var clutch_torque = engine.clutch_torque_on_engine
	var engine_torque = engine.engine_torque
	label.text = "RPM: %.0f\nClutch Torque: %.2f\nEngine Torque: %.2f" % [rpm, clutch_torque, engine_torque]
