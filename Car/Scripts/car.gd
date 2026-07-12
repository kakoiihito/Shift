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
	
var Force_Arrow_Scene = load("res://UI/force_debug.tscn")

@onready var Debug_Suspension = []
@onready var Debug_Longtiude = []
@onready var Debug_Lateral = []

func _ready() -> void:
	
	var Wheels = [fl_wheel, fr_wheel, rl_wheel, rr_wheel]
		
	for i in range(Wheels.size()):
		Wheels[i].set_meta("wheel_index", i) # wheel identification
	
	if VehicleValues.DEBUG == true:
		var mesh_wheels = [fl_wheel_mesh, fr_wheel_mesh, rr_wheel_mesh, rl_wheel_mesh]

		for i in range(wheels.size()):
			var offset = -VehicleValues.Position_Offset if (i == 0 or i == 3) else VehicleValues.Position_Offset

			var Suspension_Arrow = Force_Arrow_Scene.instantiate()
			mesh_wheels[i].add_child(Suspension_Arrow)
			Suspension_Arrow.color = VehicleValues.Suspension_Arrow_Color
			Suspension_Arrow.Position_Offset = offset
			Suspension_Arrow.Display_Force_Magntiude = VehicleValues.Display_Force_Magntiude
			Debug_Suspension.append(Suspension_Arrow)

			var Longitude_Arrow = Force_Arrow_Scene.instantiate()
			mesh_wheels[i].add_child(Longitude_Arrow)
			Longitude_Arrow.color = VehicleValues.Longtiude_Arrow_Color
			Longitude_Arrow.Position_Offset = offset
			Longitude_Arrow.Display_Force_Magntiude = VehicleValues.Display_Force_Magntiude
			Debug_Longtiude.append(Longitude_Arrow)

			var Lateral_Arrow = Force_Arrow_Scene.instantiate()
			mesh_wheels[i].add_child(Lateral_Arrow)
			Lateral_Arrow.color = VehicleValues.Lateral_Arrow_Color
			Lateral_Arrow.Position_Offset = offset
			Lateral_Arrow.Display_Force_Magntiude = VehicleValues.Display_Force_Magntiude
			Debug_Lateral.append(Lateral_Arrow)

	
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
	
	if VehicleValues.DEBUG == true:
			
		for i in range(Debug_Suspension.size()):
			Debug_Suspension[i].force = suspension.wheel_spring_force[i]
		for i in range(Debug_Longtiude.size()):
			Debug_Longtiude[i].force = wheeldata.longitude_force_vector[i]
		for i in range(Debug_Lateral.size()):
			Debug_Lateral[i].force = wheeldata.lateral_force_vector[i]


	
	
