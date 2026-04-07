extends Node

@export var car: RigidBody3D
@onready var Values: Resource

var wheel_spring_force = Data.wheel_spring_force
var compression = [0.0, 0.0, 0.0, 0.0]





func suspension_proccess(ray: RayCast3D):
	var arb_force = [0.0, 0.0, 0.0, 0.0]
	var wheel_index = ray.get_meta("wheel_index")
	
	var wheels = [car.fl_wheel_mesh, car.fr_wheel_mesh, car.rl_wheel_mesh, car.rr_wheel_mesh]
	
	if ray.is_colliding():
		
		var hit = ray.get_collision_point()
		var up_dir_spring = ray.global_transform.basis.y
		var hit_distance = ray.global_position.distance_to(hit)
		compression[wheel_index] = clamp(Values.rest_length[wheel_index] - hit_distance, 0.0, Values.max_compression[wheel_index]) 

		if Values.front_antiroll_bar == true:
			var arb = Values.front_antiroll_bar_stiffness * (compression[0] - compression[1])
			
			arb_force[0]  += -arb
			arb_force[1] +=  arb
			
		if Values.rear_antiroll_bar == true:
			var arb = Values.rear_antiroll_bar_stiffness * (compression[2] - compression[3])

			arb_force[2]  += -arb
			arb_force[3] +=  arb

		var damper_ratio = 0.5
		
		var world_vel = _get_point_velocity(hit)
		var relative_vel = up_dir_spring.dot(world_vel)
		
		var sprung_mass = car.mass * Values.weight_distribution[wheel_index]
		var c_crit = 2.0 * sqrt(Values.spring_stiffness[wheel_index] * sprung_mass)
		var c = damper_ratio * c_crit
		var spring_dampning = c * pow(abs(relative_vel), Values.velocity_exponent) * sign(relative_vel)
		
		var spring_force = Values.spring_stiffness[wheel_index] * compression[wheel_index]
		var wheel_force_area = hit - car.global_position
		wheel_spring_force[wheel_index] = (spring_force - spring_dampning + arb_force[wheel_index]) * up_dir_spring

		
		wheels[wheel_index].position.y = -compression[wheel_index]
		car.apply_force(wheel_spring_force[wheel_index], wheel_force_area)
		
func _get_point_velocity(point: Vector3) -> Vector3:
	return car.linear_velocity + car.angular_velocity.cross(point - car.global_position)
