extends RayCast3D

@export var car: RigidBody3D


var wheel_spring_force = Data.wheel_spring_force

var wheel_engine_torque = Data.wheel_engine_torque
var wheel_brake_torque = Data.wheel_brake_torque

var wheel_angular_velocity = Data.wheel_angular_velocity
var F_max = Data.F_max
var longitude_force = Data.longitude_force
var lateral_force = Data.lateral_force
var wheel_radius = Values.wheel_radius
var wheel_mass = Values.wheel_mass
var rolling_resistance_coeff = Values.rolling_resistance_coeff
var friction_coefficient = Values.friction_coefficient
var cornering_stiffness = Values.cornering_stiffness
var wheel_inertia = Values.wheel_inertia

func _get_point_velocity(point: Vector3) -> Vector3:
	return car.linear_velocity + car.angular_velocity.cross(point - car.global_position)
	
func _get_wheel_forces(ray: RayCast3D):
	
	var wheel_index = ray.get_meta("wheel_index")
	
	var velocity_at_wheel = _get_point_velocity(ray.global_position)
	var side_dir = ray.global_transform.basis.x #
	var side_velocity = velocity_at_wheel.dot(side_dir)
	var car_speed = velocity_at_wheel.dot(-ray.global_transform.basis.z) 
	
	var slip_angle = 0.0
	if abs(car_speed) > 0.1: 
		slip_angle = atan2(side_velocity, abs(car_speed)) * sign(car_speed)
	
	lateral_force[wheel_index] = -slip_angle * cornering_stiffness * (wheel_spring_force[wheel_index].length() / 2500.0)
	
	# Longitude Force
	var slip_ratio: float
	var wheel_surface_speed = wheel_angular_velocity[wheel_index] * wheel_radius

	if abs(car_speed) < 0.5:
		if abs(wheel_surface_speed) > 0.5:
			slip_ratio = sign(wheel_surface_speed) * 1.0
		else:
			slip_ratio = 0.0
	else:
		slip_ratio = (wheel_surface_speed - car_speed) / abs(car_speed)
		slip_ratio = clamp(slip_ratio, -1.0, 1.0)
	
	var optimal_slip = 0.08
	var slip_sign = sign(slip_ratio)
	var normalized_slip = clamp(abs(slip_ratio) / optimal_slip, 0.0, 1.0)
	var traction_multiplier = slip_sign * normalized_slip
	
	if abs(car_speed) < 0.1 and abs(wheel_surface_speed) < 0.1:
		traction_multiplier = 0.0
		
	longitude_force[wheel_index] = (wheel_spring_force[wheel_index].length() * traction_multiplier * friction_coefficient)
	
	# Traction Circle
	F_max[wheel_index] = friction_coefficient * wheel_spring_force[wheel_index].length()

	var current_long_force = longitude_force[wheel_index]
	var current_lat_force = lateral_force[wheel_index]

	var force_2d = Vector2(current_long_force, current_lat_force)
	if force_2d.length() > F_max[wheel_index]:
		force_2d = force_2d.normalized() * F_max[wheel_index]
		longitude_force[wheel_index] = force_2d.x  
		lateral_force[wheel_index] = force_2d.y    
	# Final calc
	var combined_force = (longitude_force[wheel_index] * -ray.global_transform.basis.z) + (lateral_force[wheel_index] * side_dir) # both vectors combined
	var force_pos = ray.global_position - car.global_position
	car.apply_force(combined_force , force_pos)

func _get_wheel_angular_velocity(ray: RayCast3D,delta: float):
	
	var wheel_index = ray.get_meta("wheel_index") 
	
	if not car.wheels[wheel_index].is_colliding():
		var air_drag_torque = 0.001 * wheel_angular_velocity[wheel_index] * abs(wheel_angular_velocity[wheel_index])
		var angular_decel = air_drag_torque / wheel_inertia
		wheel_angular_velocity[wheel_index] -= angular_decel * delta
	else:
		var normal_force = wheel_spring_force[wheel_index].length()
		var rolling_resistance = rolling_resistance_coeff * normal_force * wheel_radius * sign(wheel_angular_velocity[wheel_index])
		
		if wheel_angular_velocity[wheel_index] == 0.0:
			rolling_resistance = 0.0
			
		var ground_reaction_torque = -longitude_force[wheel_index] * wheel_radius
		var net_torque = wheel_engine_torque[wheel_index] - wheel_brake_torque[wheel_index] + ground_reaction_torque - rolling_resistance
		
		var angular_acceleration = net_torque / wheel_inertia if wheel_inertia > 0 else 0
		
		var prev_sign = sign(wheel_angular_velocity[wheel_index])
		wheel_angular_velocity[wheel_index] += angular_acceleration * delta
		
		
		if wheel_brake_torque[wheel_index] > 0 and sign(wheel_angular_velocity[wheel_index]) != prev_sign:
			wheel_angular_velocity[wheel_index] = 0.0
			
	if abs(wheel_angular_velocity[wheel_index]) < 0.1:
		wheel_angular_velocity[wheel_index] = 0.0
		
		
			
