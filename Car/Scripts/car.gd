# Note: All comments are to explain the code below the comment like this one.
# Note: why the hell did i make a comment for basic logic
# Note: idk man i just work here
# Note: realized i shouldnt spam comments to learn.
extends RigidBody3D

@export var VehicleValues: Resource
var car = self

	################
	# Runtime Data #
	################

var engine := RuntimeData.engine.new()
var transmission := RuntimeData.transmission.new()
var wheeldata := RuntimeData.wheels.new()
var suspension := RuntimeData.suspension.new()
var brake := RuntimeData.brake.new()
var steering := RuntimeData.steering.new()

	###########
	# SCRIPTS #
	###########

var SteeringScript = load("res://Car/Scripts/steering.gd")
var TransmissionScript = load("res://Car/Scripts/transmission.gd")
var SuspensionScript = load("res://Car/Scripts/suspension.gd")
var MF52_LiteScript = load("res://Car/Scripts/TireModels/mf52_lite.gd")
var MF52_FullScript = load("res://Car/Scripts/TireModels/mf52_full.gd")
var BrushModel_Script = load("res://Car/Scripts/TireModels/brushtire.gd")
var MotorScript = load("res://Car/Scripts/motor.gd")
var BrakeScript = load("res://Car/Scripts/brake.gd")
var InputFeedbackScript = load("res://Car/Scripts/input_feedback.gd")
var AssistsScript = load("res://Car/Scripts/assists.gd")

var Steering = SteeringScript.new()
var Transmission = TransmissionScript.new()
var Suspension = SuspensionScript.new()
var MF52_LiteProcess = MF52_LiteScript.new()
var MF52_FullProcess = MF52_FullScript.new()
var BrushModel_Process = BrushModel_Script.new()
var Motor = MotorScript.new()
var Brake = BrakeScript.new()
var InputFeedback = InputFeedbackScript.new()
var Assists = AssistsScript.new()

	##########
	# WHEELS #
	##########
	
@export var wheels: Array[RayCast3D]

@onready var fl_wheel = wheels[0]
@onready var fr_wheel = wheels[1]
@onready var rr_wheel = wheels[2]
@onready var rl_wheel = wheels[3]

@onready var fl_wheel_mesh = fl_wheel.get_child(0)
@onready var fr_wheel_mesh = fr_wheel.get_child(0)
@onready var rr_wheel_mesh = rr_wheel.get_child(0)
@onready var rl_wheel_mesh = rl_wheel.get_child(0)

	#########
	# DEBUG #
	#########

var DEBUG: bool = true

@onready var Debug_Suspension = [rr_wheel_mesh.get_node("Suspension/ForceDebug"),
								  rl_wheel_mesh.get_node("Suspension/ForceDebug"),
								  fr_wheel_mesh.get_node("Suspension/ForceDebug"),
								  fl_wheel_mesh.get_node("Suspension/ForceDebug")]
@onready var Debug_Longtiude = [rr_wheel_mesh.get_node("Longitude/ForceDebug"),
								 rl_wheel_mesh.get_node("Longitude/ForceDebug"),
								 fr_wheel_mesh.get_node("Longitude/ForceDebug"),
								 fl_wheel_mesh.get_node("Longitude/ForceDebug")]
@onready var Debug_Lateral = [rr_wheel_mesh.get_node("Lateral/ForceDebug"),
							   rl_wheel_mesh.get_node("Lateral/ForceDebug"),
							   fr_wheel_mesh.get_node("Lateral/ForceDebug"),
							   fl_wheel_mesh.get_node("Lateral/ForceDebug")]

func _ready() -> void:
	
	var Wheels = [fl_wheel, fr_wheel, rl_wheel, rr_wheel]
		
	for i in range(Wheels.size()):
		Wheels[i].set_meta("wheel_index", i) # wheel identification
	


	
func _physics_process(delta: float) -> void:
	Transmission.transmission_process(delta, transmission, VehicleValues)
	
	for wheel in wheels:
		Suspension.suspension_proccess(wheel, suspension, car, VehicleValues)
	
	Steering.steering_proccess(delta, steering, wheeldata, car, VehicleValues)
	Brake.brake_process(delta, brake, VehicleValues)
	

	for wheel in wheels:
		if VehicleValues.TireModel == VehicleValues.TireModelType.MF52_Lite:
			MF52_LiteProcess._get_wheel_forces(wheel, wheeldata, suspension, car, VehicleValues)
		elif VehicleValues.TireModel == VehicleValues.TireModelType.MF52_Full:
			MF52_FullProcess._get_wheel_forces(wheel, wheeldata, suspension, car, VehicleValues)
		elif VehicleValues.TireModel == VehicleValues.TireModelType.Brush_Model:
			BrushModel_Process._get_wheel_forces(wheel, wheeldata, suspension, car, VehicleValues)
		
	Motor.motor_process(delta, engine, transmission, wheeldata, VehicleValues)
	
	for wheel in wheels:
		if VehicleValues.TireModel == VehicleValues.TireModelType.MF52_Lite:
			MF52_LiteProcess._get_wheel_angular_velocity(wheel, delta, wheeldata, engine, brake, suspension, car, VehicleValues)
		elif VehicleValues.TireModel == VehicleValues.TireModelType.MF52_Full:
			MF52_FullProcess._get_wheel_angular_velocity(wheel, delta, wheeldata, engine, brake, suspension, car, VehicleValues)
		elif VehicleValues.TireModel == VehicleValues.TireModelType.Brush_Model:
			MF52_FullProcess._get_wheel_angular_velocity(wheel, delta, wheeldata, engine, brake, suspension, car, VehicleValues)

	Assists.abs_proccess(delta, brake, wheeldata, VehicleValues)
	Assists.tc_proccess(delta, engine, wheeldata, VehicleValues)
	Debug_Arrows()


func Debug_Arrows():

	if DEBUG == true:

		for i in range(Debug_Suspension.size()):
			Debug_Suspension[i].force_path = suspension.wheel_spring_force[i]
		for i in range(Debug_Longtiude.size()):
			Debug_Longtiude[i].force_path = wheeldata.longitude_force_vector[i]
		for i in range(Debug_Lateral.size()):
			Debug_Lateral[i].force_path = wheeldata.lateral_force_vector[i]
	
	
