extends Control

@onready var info_label: Label = $VBoxContainer/HBoxContainer/Label
@export var car: Node3D # yar
@onready var engine_rpm
@onready var wheel_diameter
@onready var gear_ratio
@onready var final_drive

func _ready() -> void:
	engine_rpm = car.engine_rpm
	wheel_diameter = car.wheel_radius * 2.0
	gear_ratio = car.gear_ratio
	final_drive = car.final_drive

func _physics_process(delta: float) -> void:
	var calc = (engine_rpm * wheel_diameter) / (gear_ratio * final_drive * 336)
	var string = str(engine_rpm)
	info_label.text = string
