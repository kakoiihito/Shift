extends Node

var max_brake_torque = 200.0 # How much the car can brake
var wheel_brake_torque = [0.0, 0.0, 0.0, 0.0] # stored brake torque values
var brake_torque: float # storing brake torque for calculation (single wheel)
