extends RayCast3D

func _get_point_velocity(point: Vector3, car: RigidBody3D) -> Vector3:
	return car.linear_velocity + car.angular_velocity.cross(point - car.global_position)
	
func _get_wheel_forces(ray: RayCast3D, WheelData: RuntimeData.wheels, SuspensionData: RuntimeData.suspension, car: RigidBody3D, Values: Resource):

	var wheel_index = ray.get_meta("wheel_index")
	var velocity_at_wheel = _get_point_velocity(ray.get_collision_point(), car)
	var side_dir = ray.global_transform.basis.x #
	var side_velocity = velocity_at_wheel.dot(side_dir)
	var forward_speed = velocity_at_wheel.dot(-ray.global_transform.basis.z)
	var wheel_surface_speed = WheelData.wheel_angular_velocity[wheel_index] * Values.wheel_radius
	var Fz = SuspensionData.wheel_spring_force[wheel_index].length() / 1000
	var slip_ratio_percentage: float
	
		
	if ray.is_colliding():
	
		# camber calc
	
		WheelData.camber[wheel_index]  =(Values.camber_angles[wheel_index]) + (Values.camber_gain[wheel_index] * SuspensionData.compression[wheel_index])
		
		# slip angle calc
		if abs(forward_speed) > 0.0:
			WheelData.slip_angle[wheel_index] = -(atan(side_velocity / forward_speed))
		
		if abs(forward_speed) > 0.0:
			WheelData.slip_ratio[wheel_index] = (wheel_surface_speed - forward_speed) / abs(forward_speed)
			slip_ratio_percentage = clamp(WheelData.slip_ratio[wheel_index] * 100.0, -100.0, 100.0)

		var Fz_nominal_kN = (car.mass * 9.81) * Values.weight_distribution[wheel_index] / 1000.0  
		var dfz = (Fz - Fz_nominal_kN) / Fz_nominal_kN
		
		# pure longitudinal force calc

		var C   = Values.b01
		var D   = Fz * (Values.b111 * Fz + Values.b21)
		var BCD = Values.b41 * Fz
		var B   = BCD / (C * D)
		var E   = Values.b61 * pow(Fz, 2) + Values.b71 * Fz + Values.b81
		var Bx1 = B * slip_ratio_percentage
		var Fxo = D * sin(C * atan(Bx1 - E * (Bx1 - atan(Bx1))))
		
		# combined slip longitudinal force calc
		
		var alpha_s = WheelData.slip_angle[wheel_index]
		var Bxa = Values.rBx11 * cos(atan(Values.rBx2 * slip_ratio_percentage)) * Values.lambda_xalpha
		var Cxa = Values.rCx11
		var Exa = Values.rEx11

		var Gxa = cos(Cxa * atan(Bxa * alpha_s - Exa * (Bxa * alpha_s - atan(Bxa * alpha_s))))

		WheelData.longitude_force[wheel_index] = Gxa * Fxo
		
		# pure lateral force calc

		var D1   = Values.a21 * Fz
		var BCD1 = Values.a31 * sin(2.0 * atan(Fz / Values.a41))
		var B1   = BCD1 / (Values.a01 * D1)
		var E1   = Values.a61 * Fz + Values.a71
		var Bx2  = B1 * WheelData.slip_angle[wheel_index]
		var Fyo  = D1 * sin(Values.a01 * atan(Bx2 - E1 * (Bx2 - atan(Bx2))))

		# combined slip lateral force calc

		var kappa_s = WheelData.slip_ratio[wheel_index]
		var Byk = Values.rBy11 * cos(atan(WheelData.slip_angle[wheel_index] - Values.rBy21))
		var Gyk = cos(Values.rCy11 * atan(Byk * kappa_s))

		WheelData.lateral_force[wheel_index] = Fyo * Gyk
		
		# aligning torque calc
		
		var Kxkappa = BCD
		
		var stiffness_ratio_sq = pow(Kxkappa / BCD1, 2)
		var kappa_sq = pow(slip_ratio_percentage, 2)

		var alpha_t_eq = sqrt(pow(WheelData.slip_angle[wheel_index], 2) + stiffness_ratio_sq * kappa_sq) * sign(WheelData.slip_angle[wheel_index])
		var alpha_r_eq = sqrt(pow(WheelData.slip_angle[wheel_index], 2) + stiffness_ratio_sq * kappa_sq) * sign(WheelData.slip_angle[wheel_index])

		var s = Values.Ro * (Values.ssz1 + Values.ssz2 * (WheelData.lateral_force[wheel_index] / Fz_nominal_kN) + (Values.ssz3 + Values.ssz4 * dfz) * WheelData.camber[wheel_index]) * Values.lambda_s

		var Mzr = Values.Dr * cos(Values.Cr * atan(Values.Br * alpha_r_eq))

		var trail = Values.Dt * cos(Values.Ct * atan(Values.Bt * alpha_t_eq - Values.Et * (Values.Bt * alpha_t_eq - atan(Values.Bt * alpha_t_eq)))) * cos(WheelData.slip_angle[wheel_index])
		var Mz_ = -trail * (WheelData.lateral_force[wheel_index])

		WheelData.aligning_torque[wheel_index] = Mz_ + Mzr + s * WheelData.longitude_force[wheel_index]

				
		# final force calc (aligning torque is applied in steering.gd)
		
		var combined_force = (WheelData.longitude_force[wheel_index] * -ray.global_transform.basis.z) + (WheelData.lateral_force[wheel_index] * side_dir) # both vectors combined
		var force_pos = ray.get_collision_point() - car.global_position
		car.apply_force(combined_force , force_pos)

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



		
		
		
			
