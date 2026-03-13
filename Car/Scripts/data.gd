extends Node

var wheel_spring_force = [Vector3(), Vector3(), Vector3(), Vector3()]
var wheel_engine_torque = [0.0, 0.0, 0.0, 0.0]
var wheel_brake_torque = [0.0, 0.0, 0.0, 0.0]
var wheel_angular_velocity = [0.0, 0.0, 0.0, 0.0]
var F_max = [0.0, 0.0, 0.0, 0.0]
var longitude_force = [0.0, 0.0, 0.0, 0.0]
var lateral_force = [0.0, 0.0, 0.0, 0.0]
var engine_rpm: float
var is_shifting: bool
var current_gear_ratio: float
var current_gear = 1
