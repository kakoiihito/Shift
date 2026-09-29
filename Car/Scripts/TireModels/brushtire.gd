extends RayCast3D

func _get_point_velocity(point: Vector3, car: RigidBody3D) -> Vector3:
	return car.linear_velocity + car.angular_velocity.cross(point - car.global_position)


func _get_wheel_forces(wheel: RayCast3D, WheelData: RuntimeData.wheels, SuspensionData: RuntimeData.suspension, car: RigidBody3D, Values: Resource):
	
	var wheel_index = wheel.get_meta("wheel_index")
	
	core_values_calc(wheel, WheelData, car, wheel_index, Values, SuspensionData)

	if not wheel.is_colliding():
		return

	slip_calc(WheelData, wheel_index)
	slip_force_calc(WheelData, wheel_index, Values)
	total_force_calc(WheelData, wheel_index, Values)
	force_split_calc(WheelData, wheel_index)
	force_vector_calc(wheel, WheelData, wheel_index, car)


func core_values_calc(wheel: RayCast3D, WheelData: RuntimeData.wheels, car: RigidBody3D, wheel_index: int, Values: Resource, SuspensionData: RuntimeData.suspension):
	
	var velocity_at_wheel = _get_point_velocity(wheel.global_position, car)
	WheelData.side_velocity = velocity_at_wheel.dot(wheel.global_transform.basis.x)
	WheelData.forward_speed = velocity_at_wheel.dot(-wheel.global_transform.basis.z)
	WheelData.wheel_surface_speed = WheelData.wheel_angular_velocity[wheel_index] * Values.wheel_radius
	WheelData.normal_load[wheel_index] = SuspensionData.wheel_spring_force[wheel_index].length()

func slip_calc(WheelData: RuntimeData.wheels, wheel_index: int):
	
	if abs(WheelData.forward_speed) < 0.001:
		WheelData.slip_angle[wheel_index] = 0.0
		WheelData.slip_ratio[wheel_index] = 0.0
	else:
		WheelData.slip_angle[wheel_index] = -(atan(WheelData.side_velocity / abs(WheelData.forward_speed)))
		WheelData.slip_ratio[wheel_index] = (WheelData.wheel_surface_speed - WheelData.forward_speed) / abs(WheelData.forward_speed)

func slip_force_calc(WheelData: RuntimeData.wheels, wheel_index: int, Values: Resource):
	
	var sx = Values.brush_Csx * WheelData.slip_ratio[wheel_index]
	var sy = Values.brush_Csy * tan(WheelData.slip_angle[wheel_index])

	WheelData.slip_force_x[wheel_index] = sx
	WheelData.slip_force_y[wheel_index] = sy
	WheelData.s_combined[wheel_index] = sqrt(sx * sx + sy * sy)

func total_force_calc(WheelData: RuntimeData.wheels, wheel_index: int, Values: Resource):
	
	WheelData.friction_limit[wheel_index] = Values.brush_mu * WheelData.normal_load[wheel_index]
	var mu_Fz = WheelData.friction_limit[wheel_index]
	if mu_Fz > 0.001:
		WheelData.theta[wheel_index] = WheelData.s_combined[wheel_index] / (3.0 * mu_Fz)
	else:
		WheelData.theta[wheel_index] = 0.0

	var theta = WheelData.theta[wheel_index]

	if theta <= 1.0:
		WheelData.total_force[wheel_index] = mu_Fz * (3.0 * theta - 3.0 * theta * theta + theta * theta * theta)
	else:
		WheelData.total_force[wheel_index] = mu_Fz

func force_split_calc(WheelData: RuntimeData.wheels, wheel_index: int):
	var s_combined = WheelData.s_combined[wheel_index]

	if s_combined > 0.0001:
		var F_total = WheelData.total_force[wheel_index]
		WheelData.longitude_force[wheel_index] = F_total * (WheelData.slip_force_x[wheel_index] / s_combined)
		WheelData.lateral_force[wheel_index] = F_total * (WheelData.slip_force_y[wheel_index] / s_combined)
	else:
		WheelData.longitude_force[wheel_index] = 0.0
		WheelData.lateral_force[wheel_index] = 0.0


func force_vector_calc(wheel: RayCast3D, WheelData: RuntimeData.wheels, wheel_index: int, car: RigidBody3D):
	WheelData.longitude_force_vector[wheel_index] = WheelData.longitude_force[wheel_index] * -wheel.global_transform.basis.z
	WheelData.lateral_force_vector[wheel_index] = WheelData.lateral_force[wheel_index] * wheel.global_transform.basis.x
	WheelData.combined_force_vector[wheel_index] = WheelData.longitude_force_vector[wheel_index] + WheelData.lateral_force_vector[wheel_index]

	var force_pos = wheel.get_collision_point() - car.global_position
	car.apply_force(WheelData.combined_force_vector[wheel_index], force_pos)
	
func _get_wheel_angular_velocity(wheel: RayCast3D, delta: float, WheelData: RuntimeData.wheels, EngineData: RuntimeData.engine, BrakeData: RuntimeData.brake, SuspensionData: RuntimeData.suspension, Values: Resource):
	var wheel_index: int = wheel.get_meta("wheel_index")
	var net_torque: float

	# in air behavior

	if not wheel.is_colliding():

		var brake_torque: float = BrakeData.wheel_brake_torque[wheel_index] * signf(WheelData.wheel_angular_velocity[wheel_index])
		var air_drag_coeff: float = 0.001 * absf(WheelData.wheel_angular_velocity[wheel_index])

		net_torque = EngineData.wheel_engine_torque[wheel_index] - brake_torque
		WheelData.wheel_angular_velocity[wheel_index] = (WheelData.wheel_angular_velocity[wheel_index] + (net_torque / Values.wheel_inertia) * delta) / (1.0 + (air_drag_coeff / Values.wheel_inertia) * delta)

	# on ground behavior

	else:
		var normal_force: float = SuspensionData.wheel_spring_force[wheel_index].length()
		var rolling_resistance: float = Values.rolling_resistance_coeff * normal_force * Values.wheel_radius * signf(WheelData.wheel_angular_velocity[wheel_index])
		var ground_reaction_torque: float = WheelData.longitude_force[wheel_index] * Values.wheel_radius

		net_torque = EngineData.wheel_engine_torque[wheel_index] - BrakeData.wheel_brake_torque[wheel_index] - ground_reaction_torque - rolling_resistance
		WheelData.wheel_angular_velocity[wheel_index] += (net_torque / Values.wheel_inertia) * delta

		if BrakeData.wheel_brake_torque[wheel_index] > 0.0 and WheelData.wheel_angular_velocity[wheel_index] < 0.0:
			WheelData.wheel_angular_velocity[wheel_index] = 0.0






		
		
		
			
