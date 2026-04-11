# Note: All comments are to explain the code below the comment like this one.
# Note: why the hell did i make a comment for basic logic
# Note: idk man i just work here
# Note: realized i shouldnt spam comments to learn.
extends RigidBody3D

@export var VehicleValues: Resource

	###########
	# SCRIPTS #
	###########

var SteeringScript = load("res://Car/Scripts/steering.gd")
var TransmissionScript = load("res://Car/Scripts/transmission.gd")
var SuspensionScript = load("res://Car/Scripts/suspension.gd")
var WheelProcessScript = load("res://Car/Scripts/wheel.gd")
var MotorScript = load("res://Car/Scripts/motor.gd")
var BrakeScript = load("res://Car/Scripts/brake.gd")
var InputFeedbackScript = load("res://Car/Scripts/input_feedback.gd")

var Steering = SteeringScript.new()
var Transmission = TransmissionScript.new()
var Suspension = SuspensionScript.new()
var WheelProcess = WheelProcessScript.new()
var Motor = MotorScript.new()
var Brake = BrakeScript.new()
var InputFeedback = InputFeedbackScript.new()

	##########
	# WHEELS #
	##########
@export var wheels: Array[RayCast3D]
@onready var fl_wheel = $WheelFrontLeft
@onready var fr_wheel = $WheelFrontRight
@onready var rr_wheel = $WheelRearRight
@onready var rl_wheel = $WheelRearLeft
@onready var fl_wheel_mesh = $WheelFrontLeft/FrontLeftWheel/wheel_frontLeft
@onready var fr_wheel_mesh = $WheelFrontRight/FrontRightWheel/wheel_frontRight
@onready var rr_wheel_mesh = $WheelRearRight/RearRightWheel/wheel_backRight
@onready var rl_wheel_mesh = $WheelRearLeft/RearLeftWheel/wheel_backLeft
	####################
	# ENGINE VARIABLES #
	####################
@export var torque_curve: Curve

	###################
	# BRAKE VARIABLES #
	###################

var FR_torque_brake = false
var FL_torque_brake = false
var RR_torque_brake = true
var RL_torque_brake = true



func _ready() -> void:
	
	Suspension.car = self
	Transmission.car = self
	Steering.car = self
	WheelProcess.car = self
	Motor.car = self
	Brake.car = self
	Suspension.Values = VehicleValues
	Transmission.Values = VehicleValues
	Steering.Values = VehicleValues
	WheelProcess.Values = VehicleValues
	Motor.Values = VehicleValues
	Brake.Values = VehicleValues
	
	fl_wheel.set_meta("wheel_index", 0)
	fr_wheel.set_meta("wheel_index", 1)
	rl_wheel.set_meta("wheel_index", 2)
	rr_wheel.set_meta("wheel_index", 3)
			

func _physics_process(delta: float) -> void:
	
	Steering.steering_proccess(delta) # independent function
	Transmission.transmission_process(delta) # independent function
	
	for wheel in wheels:
		Suspension.suspension_proccess(wheel) # independent function
		WheelProcess._get_wheel_angular_velocity(wheel, delta) # relies on wheel force and suspension functions
		WheelProcess._get_wheel_forces(wheel) # relies on wheel ang and suspension functions
	
	InputFeedback._input_feedback()
	Motor.motor_process(delta) # relies on wheel ang and forces functions
	Brake.brake_proccess() #relies on wheel angular velocity function
