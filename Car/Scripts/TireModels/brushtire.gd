extends RayCast3D




func _get_point_velocity(point: Vector3, car: RigidBody3D) -> Vector3:
	return car.linear_velocity + car.angular_velocity.cross(point - car.global_position)

func _get_wheel_forces(ray: RayCast3D, WheelData: RuntimeData.wheels, SuspensionData: RuntimeData.suspension, car: RigidBody3D, Values: Resource):
	var wheel_index = ray.get_meta("wheel_index")

	var velocity_at_wheel = _get_point_velocity(ray.global_position, car)
	var side_velocity = velocity_at_wheel.dot(ray.global_transform.basis.x)
	var forward_speed = velocity_at_wheel.dot(-ray.global_transform.basis.z)

	var wheel_surface_speed = WheelData.wheel_angular_velocity[wheel_index] * Values.wheel_radius

	var Fz = SuspensionData.wheel_spring_force[wheel_index].length()

	if ray.is_colliding():

		if abs(forward_speed) < 0.001:
			WheelData.slip_angle[wheel_index] = 0.0
			WheelData.slip_ratio[wheel_index] = 0.0
		else:
			WheelData.slip_angle[wheel_index] = -(atan(side_velocity / forward_speed))
			WheelData.slip_ratio[wheel_index] = (wheel_surface_speed - forward_speed) / abs(forward_speed)

		var mu_Fz = Values.brush_mu * Fz


		var sx = Values.brush_Csx * WheelData.slip_ratio[wheel_index]
		var sy = Values.brush_Csy * tan(WheelData.slip_angle[wheel_index])

		var s_combined = sqrt(sx * sx + sy * sy)

		var theta = s_combined / (3.0 * mu_Fz) if mu_Fz > 0.001 else 0.0


		var F_total: float
		if theta <= 1.0:
			F_total = mu_Fz * (3.0 * theta - 3.0 * theta * theta + theta * theta * theta)
		else:
			F_total = mu_Fz

		if s_combined > 0.0001:
			WheelData.longitude_force[wheel_index] = F_total * (sx / s_combined)
			WheelData.lateral_force[wheel_index]   = F_total * (sy / s_combined)
		else:
			WheelData.longitude_force[wheel_index] = 0.0
			WheelData.lateral_force[wheel_index]   = 0.0
		
		WheelData.longitude_force_vector[wheel_index] = (WheelData.longitude_force[wheel_index] * -ray.global_transform.basis.z)
		WheelData.lateral_force_vector[wheel_index] = (WheelData.lateral_force[wheel_index]  *  ray.global_transform.basis.x)
		
		var combined_force = WheelData.longitude_force_vector[wheel_index] + WheelData.lateral_force_vector[wheel_index]
		var force_pos = ray.get_collision_point() - car.global_position
		car.apply_force(combined_force, force_pos)

func _get_wheel_angular_velocity(ray: RayCast3D, delta: float, WheelData: RuntimeData.wheels, EngineData: RuntimeData.engine, BrakeData: RuntimeData.brake, SuspensionData: RuntimeData.suspension, car: RigidBody3D, Values: Resource):
	var wheel_inertia =  0.7 * Values.wheel_mass * (Values.wheel_radius * Values.wheel_radius)
	var wheel_index = ray.get_meta("wheel_index") 
	
	# in-air behavior
	
	if not car.wheels[wheel_index].is_colliding():
		var air_drag_torque = 0.001 * WheelData.wheel_angular_velocity[wheel_index] * abs(WheelData.wheel_angular_velocity[wheel_index])
		var brake_torque = BrakeData.wheel_brake_torque[wheel_index] * sign(WheelData.wheel_angular_velocity[wheel_index])
		
		var net_torque = EngineData.wheel_engine_torque[wheel_index] - brake_torque - air_drag_torque
		
		var angular_acceleration = net_torque / wheel_inertia
		WheelData.wheel_angular_velocity[wheel_index] += angular_acceleration * delta
		
	# on ground behavior
		
	else:
		var normal_force = SuspensionData.wheel_spring_force[wheel_index].length()
		
		var rolling_resistance = Values.rolling_resistance_coeff * normal_force * Values.wheel_radius * sign(WheelData.wheel_angular_velocity[wheel_index])
			
		var ground_reaction_torque = -WheelData.longitude_force[wheel_index] * Values.wheel_radius
		
		var net_torque = EngineData.wheel_engine_torque[wheel_index] - BrakeData.wheel_brake_torque[wheel_index] + ground_reaction_torque - rolling_resistance
		
		var angular_acceleration = net_torque / wheel_inertia
		
		WheelData.wheel_angular_velocity[wheel_index] += angular_acceleration * delta
		
		# safe guard for braking.
		
		if BrakeData.wheel_brake_torque[wheel_index] > 0.0 and WheelData.wheel_angular_velocity[wheel_index] < 0.0:
			WheelData.wheel_angular_velocity[wheel_index] = 0.0



		
		
		
			
