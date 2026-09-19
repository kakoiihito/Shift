extends Node

func suspension_proccess(ray: RayCast3D, Data: RuntimeData.suspension, car: RigidBody3D, Values: Resource):
	
	if ray.is_colliding():
		for i in range(4):
			var sprung_mass = sprung_mass_calc(car, Values, i)
			var k = spring_rate_calc(car, Values, i)
			var spring_dampning = spring_dampning_calc(ray, car, Values, k, sprung_mass, i)
			var arb_force = anti_roll_bar_calc(Data, Values)
			compression_calc(ray, Data, Values, i)
			
			spring_force_calc(ray, Data, car, k, spring_dampning, arb_force, i)
			print(Data.wheel_spring_force)
	else:
		for i in range(4):
			Data.compression[i] = 0.0
			#var rest_length = ray.target_position.length()
			#wheels[i].position.y = -rest_length

func anti_roll_bar_calc(Data: RuntimeData.suspension, Values: Resource):# anti roll bar calc
		
	var arb_force: Array[float] = [0.0, 0.0, 0.0, 0.0]
		
	if Values.front_antiroll_bar:
		var arb = Values.front_antiroll_bar_stiffness * (Data.compression[0] - Data.compression[1])
		arb_force[0] = -arb
		arb_force[1] = arb

	if Values.rear_antiroll_bar:
		var arb = Values.rear_antiroll_bar_stiffness * (Data.compression[2] - Data.compression[3])
		arb_force[2] = -arb
		arb_force[3] = arb
	
	return arb_force
		
func sprung_mass_calc(car: RigidBody3D, Values: Resource, i: int):
	
	var sprung_mass = 0.0

	sprung_mass = car.mass * Values.weight_distribution[i]
	return sprung_mass
		
func spring_rate_calc(car: RigidBody3D, Values: Resource, i: int):
	# frequency based spring rate calc
	var sprung_mass = 0.0
	var omega_n = 0.0

	sprung_mass = car.mass * Values.weight_distribution[i]
	omega_n = TAU * Values.ride_frequency[i]
	
	var k = sprung_mass * omega_n * omega_n
	return k 

func spring_dampning_calc(ray: RayCast3D,car: RigidBody3D, Values: Resource, k: float, sprung_mass: float, i: int):
	
	var hit = ray.get_collision_point()
	var up_dir_spring = ray.global_transform.basis.y
	
	var world_vel = _get_point_velocity(hit, car)
	var relative_vel = up_dir_spring.dot(world_vel)
	var c_crit = 2.0 * sqrt(k * sprung_mass)
	var c: float

	c = (Values.damper_ratio[i] if relative_vel > 0.0 else Values.damper_ratio[i]) * c_crit
	var spring_dampning = c * pow(abs(relative_vel), Values.velocity_exponent) * sign(relative_vel)
	return spring_dampning
	
func spring_force_calc(ray: RayCast3D, Data: RuntimeData.suspension, car: RigidBody3D, k: float, spring_dampning: float, arb_force: Array[float], i: int):
		
		var wheels = [car.fl_wheel_mesh, car.fr_wheel_mesh, car.rl_wheel_mesh, car.rr_wheel_mesh]
		var hit = ray.get_collision_point()
		var up_dir_spring = ray.global_transform.basis.y
		var hit_distance = ray.global_position.distance_to(hit)

		var spring_force = k * Data.compression[i]
		Data.wheel_spring_force[i] = (spring_force - spring_dampning + arb_force[i]) * up_dir_spring
		
		wheels[i].global_position = ray.global_position - up_dir_spring * hit_distance
		var wheel_force_area = ray.global_position - car.global_position
		car.apply_force(Data.wheel_spring_force[i], wheel_force_area)

		# compression calc
func compression_calc(ray: RayCast3D, Data: RuntimeData.suspension, Values: Resource, i: int):
	var hit = ray.get_collision_point()
	var hit_distance = ray.global_position.distance_to(hit)
	var rest_length = ray.target_position.length()
	Data.compression[i] = clamp(rest_length - hit_distance, 0, Values.max_compression[i]) 
		

func _get_point_velocity(point: Vector3, car: RigidBody3D) -> Vector3:
	return car.linear_velocity + car.angular_velocity.cross(point - car.global_position)
