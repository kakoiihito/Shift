class_name RuntimeData
extends Resource


class suspension:
	var wheel_spring_force = [Vector3(), Vector3(), Vector3(), Vector3()]
	var compression = [0.0, 0.0, 0.0, 0.0]
	
class brake:
	var wheel_brake_torque = [0.0, 0.0, 0.0, 0.0]
	var abs_active = [false, false, false, false]
	var active_wheels_brake: int
	var brake_torque: float
	var input_brake := 0.0

class engine:
	var wheel_engine_torque = [0.0, 0.0, 0.0, 0.0]
	var engine_torque: float
	var engine_rpm: float
	var engine_angular_velocity: float
	var clutch_torque_on_engine: float
	var engine_stalled: bool
	var engine_driven_count = 0

class transmission:
	var is_shifting = false
	var current_gear_ratio: float
	var shift_timer = 0.0
	var current_gear = 1
	

class wheels:
	var wheel_angular_velocity = [0.0, 0.0, 0.0, 0.0]
	var longitude_force_vector = [Vector3(), Vector3(), Vector3(), Vector3()]
	var lateral_force_vector = [Vector3(), Vector3(), Vector3(), Vector3()]
	var longitude_force = [0.0, 0.0, 0.0, 0.0]
	var lateral_force = [0.0, 0.0, 0.0, 0.0]
	var slip_ratio = [0.0, 0.0, 0.0, 0.0]
	var aligning_torque = [0.0, 0.0, 0.0, 0.0]
	var camber = [0.0, 0.0, 0.0, 0.0]
	var slip_angle = [0.0, 0.0, 0.0, 0.0]
	var forward_speed = 0.0
	var side_velocity = 0.0
	var wheel_surface_speed = 0.0
	var normal_load = [0.0, 0.0, 0.0, 0.0]      
	var friction_limit = [0.0, 0.0, 0.0, 0.0]   
	var slip_force_x = [0.0, 0.0, 0.0, 0.0]     
	var slip_force_y = [0.0, 0.0, 0.0, 0.0]
	var s_combined = [0.0, 0.0, 0.0, 0.0]
	var theta = [0.0, 0.0, 0.0, 0.0]
	var total_force = [0.0, 0.0, 0.0, 0.0]      
	var combined_force_vector = [Vector3(), Vector3(), Vector3(), Vector3()]
	

class steering:
	var target_fl :float
	var target_fr :float
	var inner: float
	var outer: float
	var input_turn: float = 0.0
	var steering_amount: float = 0.0
	var max_angle: float = 0.0
