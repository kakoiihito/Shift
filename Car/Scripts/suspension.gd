extends Node

func suspension_proccess(ray: RayCast3D, Data: RuntimeData.suspension, car: RigidBody3D, Values: Resource):
	
	var wheel_index = ray.get_meta("wheel_index")
	var wheels = [car.fl_wheel_mesh, car.fr_wheel_mesh, car.rl_wheel_mesh, car.rr_wheel_mesh]
	
	if ray.is_colliding():
		
		# compression calc
		
		var hit = ray.get_collision_point()
		var up_dir_spring = ray.global_transform.basis.y
		var hit_distance = ray.global_position.distance_to(hit)
		var rest_length = ray.target_position.length()
		Data.compression[wheel_index] = clamp(rest_length - hit_distance, 0, Values.max_compression[wheel_index]) 
		
		# anti roll bar calc
		
		if Values.front_antiroll_bar:
			var arb = Values.front_antiroll_bar_stiffness * (Data.compression[0] - Data.compression[1])
			Data.arb_force[0] = -arb
			Data.arb_force[1] = arb

		if Values.rear_antiroll_bar:
			var arb = Values.rear_antiroll_bar_stiffness * (Data.compression[2] - Data.compression[3])
			Data.arb_force[2] = -arb
			Data.arb_force[3] = arb
		
		# frequency based spring rate calc
		
		var sprung_mass = car.mass * Values.weight_distribution[wheel_index]
		var omega_n = TAU * Values.ride_frequency[wheel_index]
		var k = sprung_mass * omega_n * omega_n             
		
		# spring dampning calc
		
		var world_vel = _get_point_velocity(hit, car)
		var relative_vel = up_dir_spring.dot(world_vel)
		var c_crit = 2.0 * sqrt(k * sprung_mass)
		var c = (Values.damper_ratio[wheel_index] if relative_vel > 0.0 else Values.damper_ratio[wheel_index]) * c_crit
		var spring_dampning = c * pow(abs(relative_vel), Values.velocity_exponent) * sign(relative_vel)
		
		# spring force calc
		
		var spring_force = k * Data.compression[wheel_index]
		var wheel_force_area = ray.global_position - car.global_position
		Data.wheel_spring_force[wheel_index] = (spring_force - spring_dampning + Data.arb_force[wheel_index]) * up_dir_spring

		wheels[wheel_index].global_position = ray.global_position - up_dir_spring * hit_distance
		car.apply_force(Data.wheel_spring_force[wheel_index], wheel_force_area) # application
	
	else:
		Data.compression[wheel_index] = 0.0
		var rest_length = ray.target_position.length()
		wheels[wheel_index].position.y = -rest_length
		

		

func _get_point_velocity(point: Vector3, car: RigidBody3D) -> Vector3:
	return car.linear_velocity + car.angular_velocity.cross(point - car.global_position)
