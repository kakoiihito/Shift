# Note: All comments are to explain the code below the comment like this one.
# Note: why the hell did i make a comment for basic logic
# Note: idk man i just work here
# Note: realized i shouldnt spam comments to learn.

## Main vehicle controller that ties all the car simulation scripts together.
##
## Attach this script to the car's [RigidBody3D]. It creates the shared runtime
## data objects and one instance of each simulation script (steering,
## transmission, suspension, tire model, motor, brake, assists), and runs them in
## the right order every physics frame. It also sets up and updates the optional
## debug force arrows. The simulation itself lives in those scripts; this script
## only creates them, connects them and decides the order.
##
## [b]Minimum working scene structure:[/b]
## [codeblock]
## Car (RigidBody3D)              <- this script is attached here
## ├── CollisionShape3D
## ├── FrontLeftRay (RayCast3D)
## │   └── Node3D                 <- wheel mesh holder
## ├── FrontRightRay (RayCast3D)
## │   └── Node3D
## ├── RearRightRay (RayCast3D)
## │   └── Node3D
## └── RearLeftRay (RayCast3D)
##     └── Node3D
## [/codeblock]
## The root must be a [RigidBody3D] with this script, and it needs a
## [CollisionShape3D] child for the body. Each of the four wheels is a
## [RayCast3D] child of the body, with a [Node3D] as its first (and only
## required) child. The [Node3D] is the wheel mesh node: the suspension script
## moves it to the ground contact point, so put a [MeshInstance3D] under it if you
## want a visible wheel. The debug arrows are also attached to it.
##
## [b]Setup in the editor:[/b][br]
## - Assign the car tuning resource ([CarValues]) to [member VehicleValues].[br]
## - Fill [member wheels] with the four wheel [RayCast3D] nodes [b]in this
## order: front left, front right, rear right, rear left[/b]. Each ray must have
## its wheel mesh as its first child.[br]
## - Point each ray down. The length of its [code]target_position[/code] is the
## suspension's fully extended length, and the tire code treats the ray's local
## -Z direction as the wheel's forward direction.
##
## [b]Wheel indices:[/b] after [method _ready], the rays carry a
## [code]"wheel_index"[/code] meta value that every other script uses: 0 = front
## left, 1 = front right, 2 = rear left, 3 = rear right. This is not the same
## order as the [member wheels] array (see the note on [member wheels]).
##
## [b]Order of one physics frame (see [method _physics_process]):[/b][br]
## transmission, suspension, steering, brakes, tire forces, motor, wheel spin,
## assists (ABS and traction control), debug arrows.
extends RigidBody3D

## The car tuning resource that holds all settings: suspension, steering,
## brakes, engine, transmission, wheels, tires and debug options. It must be
## assigned in the editor.
@export var VehicleValues: Resource
## A reference to this node, passed to the simulation scripts as the car body.
var car = self

	################
	# Runtime Data #
	################

## Runtime state of the engine and clutch.
var engine := RuntimeData.engine.new()
## Runtime state of the gearbox.
var transmission := RuntimeData.transmission.new()
## Runtime state of the wheels and tires.
var wheeldata := RuntimeData.wheels.new()
## Runtime state of the suspension.
var suspension := RuntimeData.suspension.new()
## Runtime state of the brakes.
var brake := RuntimeData.brake.new()
## Runtime state of the steering.
var steering := RuntimeData.steering.new()
## Runtime state of assists.
var assists := RuntimeData.assists.new()

	###########
	# SCRIPTS #
	###########

## Script resource of the steering logic.
var SteeringScript = load("res://Car/Scripts/steering.gd")
## Script resource of the transmission logic.
var TransmissionScript = load("res://Car/Scripts/transmission.gd")
## Script resource of the suspension logic.
var SuspensionScript = load("res://Car/Scripts/suspension.gd")
## Script resource of the brush tire model.
var BrushModel_Script = load("res://Car/Scripts/TireModels/brushtire.gd")
## Script resource of the engine and drivetrain logic.
var MotorScript = load("res://Car/Scripts/motor.gd")
## Script resource of the brake logic.
var BrakeScript = load("res://Car/Scripts/brake.gd")
## Script resource of the input feedback logic. It is loaded and instantiated
## here but not used in this script yet.
var InputFeedbackScript = load("res://Car/Scripts/input_feedback.gd")
## Script resource of the driver assists (ABS and traction control).
var AssistsScript = load("res://Car/Scripts/assists.gd")

## Steering logic instance. Not added to the scene tree, it is only used for
## calling its functions.
var Steering = SteeringScript.new()
## Transmission logic instance.
var Transmission = TransmissionScript.new()
## Suspension logic instance.
var Suspension = SuspensionScript.new()
## Brush tire model instance. Calculates the tire forces and the wheel spin.
var BrushModel_Process = BrushModel_Script.new()
## Engine and drivetrain logic instance.
var Motor = MotorScript.new()
## Brake logic instance.
var Brake = BrakeScript.new()
## Input feedback logic instance. Not called in this script yet.
var InputFeedback = InputFeedbackScript.new()
## Driver assist (ABS and traction control) logic instance.
var Assists = AssistsScript.new()

	##########
	# WHEELS #
	##########
	
## The four wheel raycasts, which must be assigned in the editor in the order
## front left, front right, rear right, rear left. Each ray must have its wheel
## mesh as its first child.
##
## [b]Note:[/b] the order of this array has the rear wheels in the opposite order
## to the wheel index used by the other scripts (2 = rear left, 3 = rear right).
## The wheel indices are assigned by name in [method _ready], so the simulation
## is not affected, but the debug arrows are (see [method _ready]).
@export var wheels: Array[RayCast3D]

## Front left wheel ray ([code]wheels[0][/code]).
@onready var fl_wheel = wheels[0]
## Front right wheel ray ([code]wheels[1][/code]).
@onready var fr_wheel = wheels[1]
## Rear right wheel ray ([code]wheels[2][/code]).
@onready var rr_wheel = wheels[2]
## Rear left wheel ray ([code]wheels[3][/code]).
@onready var rl_wheel = wheels[3]

## Front left wheel mesh, the first child of its ray. The suspension script moves
## it to the ground contact point.
@onready var fl_wheel_mesh = fl_wheel.get_child(0)
## Front right wheel mesh, the first child of its ray.
@onready var fr_wheel_mesh = fr_wheel.get_child(0)
## Rear right wheel mesh, the first child of its ray.
@onready var rr_wheel_mesh = rr_wheel.get_child(0)
## Rear left wheel mesh, the first child of its ray.
@onready var rl_wheel_mesh = rl_wheel.get_child(0)

	#########
	# DEBUG #
	#########
	
## Scene of one debug force arrow. One arrow of each kind is created per wheel.
var Force_Arrow_Scene = load("res://UI/force_debug.tscn")

## Debug arrows that show the suspension force of each wheel. Filled in
## [method _ready] when debugging is enabled.
@onready var Debug_Suspension = []
## Debug arrows that show the longitudinal tire force of each wheel.
@onready var Debug_Longtiude = []
## Debug arrows that show the lateral tire force of each wheel.
@onready var Debug_Lateral = []

## Prepares the car when it enters the scene tree.
##
## It does three things:[br]
## 1. Gives each wheel ray its [code]"wheel_index"[/code] meta value (front left
## 0, front right 1, rear left 2, rear right 3), which the other scripts use to
## find their data.[br]
## 2. If [code]VehicleValues.DEBUG[/code] is true, creates three force arrows
## (suspension, longitudinal, lateral) for each wheel and attaches them to the
## wheel meshes. The arrows get their color, offset and "show force magnitude"
## setting from the tuning resource. Wheels on the left side are given the
## negative of [code]Position_Offset[/code] and wheels on the right side the
## positive one.[br]
## 3. Sets the starting gear ratio from the starting gear (neutral, so 0.0).
##
## [b]Note:[/b] the debug arrows are created and later updated by their position
## in the [member wheels] array, while the data arrays use the wheel index. These
## orders differ for the rear wheels (the array has rear right before rear left),
## so the two rear arrow sets show each other's forces. The fix is to build the
## debug arrays in wheel index order, or to swap the last two entries of
## [code]mesh_wheels[/code] and the debug update.
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
			
	transmission.current_gear_ratio = VehicleValues.gear_ratio[transmission.current_gear]
	
## Runs one step of the whole car simulation. Called by Godot every physics frame.
##
## The order matters because each step uses results from earlier steps:[br]
## 1. [b]Transmission:[/b] handles gear changes and sets the active gear ratio.[br]
## 2. [b]Suspension[/b] (each wheel): computes spring compression and applies the
## suspension forces to the body, which gives the tire load.[br]
## 3. [b]Steering:[/b] turns the front wheels.[br]
## 4. [b]Brakes:[/b] calculates the brake torque of each wheel.[br]
## 5. [b]Tire forces[/b] (each wheel): computes slip and applies the tire forces to
## the body. Uses the wheel speed from the previous frame, and the suspension
## load from step 2.[br]
## 6. [b]Motor:[/b] calculates engine, clutch and differential torque. Uses the
## wheel speeds and the transmission state.[br]
## 7. [b]Wheel spin[/b] (each wheel): integrates the wheel angular velocity from
## the engine, brake, tire and rolling resistance torques.[br]
## 8. [b]Assists:[/b] runs the ABS and traction control.[br]
## 9. [b]Debug arrows:[/b] if debugging is on, hands the current forces to the
## arrows.
##
## Steps that call the tire model only run if
## [code]VehicleValues.TireModel[/code] is the brush model, which is currently the
## only model.
##
## [param delta] is the time step in seconds.
##
## [b]Note:[/b] the assists run last, after the brake and engine torque of this
## frame were already calculated, so their effect (for example
## [code]brake.abs_active[/code]) is applied one frame later.
func _physics_process(delta: float) -> void:
	Transmission.transmission_process(delta, transmission, VehicleValues) # independent
	
	for wheel in wheels:
		Suspension.suspension_proccess(wheel, suspension, car, VehicleValues) # independent
	
	Steering.steering_proccess(delta, steering, car, VehicleValues) # independent
	Brake.brake_process(delta, brake, assists, VehicleValues) # independent
	

	for wheel in wheels:
		if VehicleValues.TireModel == VehicleValues.TireModelType.Brush_Model:
			BrushModel_Process._get_wheel_forces(wheel, wheeldata, suspension, car, VehicleValues) # requires wheel angular velocity, motor, suspension
		
	Motor.motor_process(delta, engine, transmission, wheeldata, VehicleValues) # requires wheel angular velocity, wheel forces, transmission
	
	for wheel in wheels:
		if VehicleValues.TireModel == VehicleValues.TireModelType.Brush_Model:
			BrushModel_Process._get_wheel_angular_velocity(wheel, delta, wheeldata, engine, brake, suspension, VehicleValues) # requires wheel forces, suspension, motor, brakes

	Assists.abs_proccess(delta, assists, brake, wheeldata, VehicleValues)
	Assists.tc_proccess(delta, engine, wheeldata, VehicleValues)
	
	if VehicleValues.DEBUG == true:
			
		for i in range(Debug_Suspension.size()):
			Debug_Suspension[i].force = suspension.wheel_spring_force[i]
		for i in range(Debug_Longtiude.size()):
			Debug_Longtiude[i].force = wheeldata.longitude_force_vector[i]
		for i in range(Debug_Lateral.size()):
			Debug_Lateral[i].force = wheeldata.lateral_force_vector[i]
