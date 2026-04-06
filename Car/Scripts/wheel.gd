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
var wheel_inertia = Values.wheel_inertia
var camber = 0.0

var b0 = Values.b0
var b1 = Values.b1
var b2 = Values.b2
var b3 = Values.b3
var b4 = Values.b4
var b5 = Values.b5
var b6 = Values.b6
var b7 = Values.b7
var b8 = Values.b8
var b9 = Values.b9
var b10 = Values.b10
var b11 = Values.b11
var b12 = Values.b12
var b13 = Values.b13

var a0 = Values.a0
var a1 = Values.a1
var a2 = Values.a2
var a3 = Values.a3
var a4 = Values.a4
var a5 = Values.a5
var a6 = Values.a6
var a7 = Values.a7
var a8 = Values.a8
var a9 = Values.a9
var a10 = Values.a10
var a11 = Values.a11
var a12 = Values.a12
var a13 = Values.a13
var a14 = Values.a14
var a15 = Values.a15
var a16 = Values.a16
var a17 = Values.a17

var rBx1 = Values.rBx1
var rBx2 = Values.rBx2
var rBx3 = Values.rBx3
var rCx1 = Values.rCx1
var rEx1 = Values.rEx1
var rEx2 = Values.rEx2
var rHx1 = Values.rHx1
var lambda_xalpha = Values.lambda_xalpha

var rBy1 = Values.rBy1
var rBy2 = Values.rBy2
var rBy3 = Values.rBy3
var rBy4 = Values.rBy4
var rCy1 = Values.rCy1
var rEy1 = Values.rEy1
var rEy2 = Values.rEy2
var rHy1 = Values.rHy1
var rHy2 = Values.rHy2
var rVy1 = Values.rVy1
var rVy2 = Values.rVy2
var rVy3 = Values.rVy3
var rVy4 = Values.rVy4
var rVy5 = Values.rVy5
var rVy6 = Values.rVy6
var lambda_ykappa = Values.lambda_ykappa
var lambda_Vyk = Values.lambda_Vyk

func _get_point_velocity(point: Vector3) -> Vector3:
	return car.linear_velocity + car.angular_velocity.cross(point - car.global_position)
	
func _get_wheel_forces(ray: RayCast3D):
	
	var wheel_index = ray.get_meta("wheel_index")
	
	var velocity_at_wheel = _get_point_velocity(ray.get_collision_point())
	var side_dir = ray.global_transform.basis.x #
	var side_velocity = velocity_at_wheel.dot(side_dir)
	var forward_speed = velocity_at_wheel.dot(-ray.global_transform.basis.z)
	var wheel_surface_speed = wheel_angular_velocity[wheel_index] * wheel_radius
	var Fz = wheel_spring_force[wheel_index].length() / 1000
		
	if ray.is_colliding():
		
		var slip_angle = 0.0
		if abs(forward_speed) > 1.0:
			slip_angle = rad_to_deg(atan2(side_velocity, abs(forward_speed)))
		
		var slip_ratio: float
		if abs(forward_speed) > 0.5:
			slip_ratio = (wheel_surface_speed - forward_speed) / abs(forward_speed)
		else:
			slip_ratio = (wheel_surface_speed - forward_speed) / 0.5
		slip_ratio = clamp(slip_ratio * 100.0, -100.0, 100.0)
		
		#var B = 13.6527
		
		var D = Fz * (b1 * Fz + b2)
		var C = b0
		var BCD = (b3 * pow(Fz, 2) + b4 * Fz) * exp(-b5 * Fz)
		var B = BCD / (C * D)
		var H = b9 * Fz + b10
		var E = (b6* pow(Fz, 2) + b7 * Fz + b8) *  (1 - b13 * sign(slip_ratio+H))
		var V = b11 * Fz + b12
		var Bx1 = B * (slip_ratio + H)
		var Fxo = D * sin(C * atan(Bx1 - E * (Bx1 - atan(Bx1)))) + V

		var SHxa = rHx1
		var alpha_s = slip_angle + SHxa
		var Bxa = (rBx1 + rBx3 * pow(camber, 2)) * cos(atan(rBx2 * slip_ratio)) * lambda_xalpha
		var Cxa = rCx1
		var Exa = rEx1 + rEx2 * 0
		var Gxa0 = cos(Cxa * atan(Bxa * SHxa - Exa * (Bxa * SHxa - atan(Bxa * SHxa))))
		var Gxa = cos(Cxa * atan(Bxa * alpha_s - Exa * (Bxa * alpha_s - atan(Bxa * alpha_s)))) / Gxa0

		longitude_force[wheel_index] = Gxa * Fxo
		
		var C1 = a0
		var D1 = Fz * (a1 * Fz + a2) * (1 - a15 * pow(camber, 2))
		var BCD1 = a3 * sin(atan(Fz / a4) * 2) * (1 - a5* abs(camber))
		var B1 = BCD1 / (C1 * D1)
		var H1 = a8 * Fz + a9 + a10 * camber
		var E1 = (a6 * Fz + a7) * (1 - (a16 * camber + a17) * sign(-slip_angle + H1))
		var V1 = a11 * Fz + a12 + (a13 * Fz + a14) * camber * Fz
		var Bx2 = B1 * (-slip_angle + H1)
		var Fyo = D1 * sin(C1 * atan(Bx2 - E1 * (Bx2 - atan(Bx2)))) + V1

		var SHyk = rHy1 + rHy2 * 0
		var kappa_s = slip_ratio + SHyk
		var Byk = (rBy1 + rBy4 * pow(camber, 2)) * cos(atan(rBy2 * (slip_angle - rBy3))) * lambda_ykappa
		var Cyk = rCy1
		var Eyk = clamp(rEy1 + rEy2 * 0, -INF, 1.0)
		var Gyk0 = cos(Cyk * atan(Byk * SHyk - Eyk * (Byk * SHyk - atan(Byk * SHyk))))
		var Gyk = cos(Cyk * atan(Byk * kappa_s - Eyk * (Byk * kappa_s - atan(Byk * kappa_s)))) / Gyk0

		var mu_y = a2 / 1000.0
		var DVyk = mu_y * Fz * (rVy1 + rVy2 * 0 + rVy3 * camber) * cos(atan(rVy4 * slip_angle))
		var SVyk = DVyk * sin(rVy5 * atan(rVy6 * slip_ratio)) * lambda_Vyk

		lateral_force[wheel_index] = Gyk * Fyo + SVyk

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
		
		
		
			
