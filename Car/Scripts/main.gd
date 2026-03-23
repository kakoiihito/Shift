# Note: All comments are to explain the code below the comment like this one.
# Note: why the hell did i make a comment for basic logic
# Note: idk man i just work here
# Note: realized i shouldnt spam comments to learn.
extends RigidBody3D

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
	
	fl_wheel.set_meta("wheel_index", 0)
	fr_wheel.set_meta("wheel_index", 1)
	rl_wheel.set_meta("wheel_index", 2)
	rr_wheel.set_meta("wheel_index", 3)

	var brake_wheels = [FL_torque_brake, FL_torque_brake, RR_torque_brake, RL_torque_brake]
	for i in range(4):
		if brake_wheels[i] == true:
			Data.active_wheels_brake += 1
			

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
