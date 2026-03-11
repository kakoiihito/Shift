extends Node

var rest_length = [0.18, 0.18, 0.18, 0.18]
var spring_stiffness = [28700.0, 28700.0, 17000.0, 17000.0]
var max_compression = [0.12, 0.12, 0.12, 0.12]
var wheel_spring_force = [Vector3(), Vector3(), Vector3(), Vector3()]
var weight_distribution = [0.25, 0.25, 0.25, 0.25]
var velocity_exponent = 1.1

@export var car: RigidBody3D


func suspension_proccess(ray: RayCast3D):
	
	var wheel_index = ray.get_meta("wheel_index")
	
	
	if ray.is_colliding():
		
		
		var hit = ray.get_collision_point()
		var up_dir_spring = ray.global_transform.basis.y
		var hit_distance = ray.global_position.distance_to(hit)
		var compression = rest_length[wheel_index] - hit_distance
		compression = clamp(compression, 0.0, max_compression[wheel_index])  

		var damper_ratio = 0.5
		
		var world_vel = _get_point_velocity(hit)
		var relative_vel = up_dir_spring.dot(world_vel)
		
		var sprung_mass = car.mass * weight_distribution[wheel_index]
		var c_crit = 2.0 * sqrt(spring_stiffness[wheel_index] * sprung_mass)
		var c = damper_ratio * c_crit
		var spring_dampning = c * pow(abs(relative_vel), velocity_exponent) * sign(relative_vel)
		
		
		var spring_force = spring_stiffness[wheel_index] * compression
		var wheel_force_area = hit - car.global_position
		wheel_spring_force[wheel_index] = (spring_force - spring_dampning) * up_dir_spring
		
		car.apply_force(wheel_spring_force[wheel_index], wheel_force_area)
		
func _get_point_velocity(point: Vector3) -> Vector3:
	return car.linear_velocity + car.angular_velocity.cross(point - car.global_position)
