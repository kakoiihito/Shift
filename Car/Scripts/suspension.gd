## Raycast suspension for a four wheel car.
##
## Each wheel is represented by a [RayCast3D] that points down from the car body.
## While the ray hits the ground, the distance it travelled tells how far the
## spring is compressed. From that this script computes a spring force, a damper
## force and an anti-roll bar force, and applies their sum to the car's
## [RigidBody3D] at the wheel's mounting point. The wheel mesh is moved to the
## ground contact point. It keeps no state of its own: compression and forces are
## stored in the suspension runtime data, and the settings come from the car
## tuning resource.
##
## [b]Pipeline for one wheel (see [method suspension_proccess]):[/b][br]
## 1. Work out the mass this wheel carries (sprung mass).[br]
## 2. Derive the spring rate from the sprung mass and the ride frequency.[br]
## 3. Compute the damper force from the vertical velocity at the contact point.[br]
## 4. Compute the anti-roll bar force from the left/right compression
## difference.[br]
## 5. Compute the spring compression from the ray hit.[br]
## 6. Sum the forces, apply them to the car and position the wheel mesh.
##
## [b]Requirements:[/b][br]
## - Each [RayCast3D] must have a [code]"wheel_index"[/code] meta value (0 =
## front left, 1 = front right, 2 = rear left, 3 = rear right).[br]
## - The ray's [code]target_position[/code] length is the suspension's fully
## extended (rest) length. Its local Y axis is the spring's up direction.[br]
## - The [code]car[/code] body must provide the wheel mesh nodes
## [code]fl_wheel_mesh[/code], [code]fr_wheel_mesh[/code],
## [code]rl_wheel_mesh[/code] and [code]rr_wheel_mesh[/code].[br]
## - The suspension runtime data must provide [code]compression[/code] (one float
## per wheel) and [code]wheel_spring_force[/code] (one [Vector3] per wheel).
##
## [b]Units:[/b] meters, kilograms, seconds and newtons.
extends Node


## Runs one step of the suspension simulation for a single wheel.
##
## Call this once per physics frame for each wheel ray.
##
## If the ray is colliding, the full calculation runs: the spring, damper and
## anti-roll forces are computed and applied to [param car], and
## [code]Data.compression[/code] and [code]Data.wheel_spring_force[/code] for
## this wheel are updated.
##
## If the ray is not colliding (the wheel is in the air), the compression is
## reset to 0.0 and the wheel meshes are placed at the fully extended position
## (local [code]y = -rest_length[/code]). No force is applied.
##
## [param ray] is the wheel's raycast.[br]
## [param Data] is the runtime suspension state (read and written).[br]
## [param car] is the car body that the forces act on.[br]
## [param Values] is the car tuning resource.
func suspension_proccess(ray: RayCast3D, Data: RuntimeData.suspension, car: RigidBody3D, Values: Resource):
	var wheel_index: int = ray.get_meta("wheel_index")
	if ray.is_colliding():
		var sprung_mass = sprung_mass_calc(car, Values, wheel_index)
		var k = spring_rate_calc(car, Values, wheel_index)
		var spring_dampning = spring_dampning_calc(ray, car, Values, k, sprung_mass, wheel_index)
		var arb_force = anti_roll_bar_calc(Data, Values)
		compression_calc(ray, Data, Values, wheel_index)
		
		spring_force_calc(ray, Data, car, k, spring_dampning, arb_force, wheel_index)
	else:
		var wheels = [car.fl_wheel_mesh, car.fr_wheel_mesh, car.rl_wheel_mesh, car.rr_wheel_mesh]
		Data.compression[wheel_index] = 0.0
		var rest_length = ray.target_position.length()
		wheels[wheel_index].position.y = -rest_length


## Calculates the anti-roll bar force for each wheel.
##
## An anti-roll bar couples the two wheels of an axle. Here its force is
## proportional to the difference in compression between the left and the right
## wheel, multiplied by the axle's bar stiffness
## ([code]front_antiroll_bar_stiffness[/code] or
## [code]rear_antiroll_bar_stiffness[/code]). The left wheel gets the negative of
## that value and the right wheel gets the positive value. An axle whose bar is
## disabled ([code]front_antiroll_bar[/code] or [code]rear_antiroll_bar[/code] is
## false) gets zero.
##
## Returns an array of four forces in newtons, in the order front left, front
## right, rear left, rear right. The caller adds the value for each wheel to its
## spring force.
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
		

## Calculates the part of the car's mass that one wheel's spring carries.
##
## It is the total car mass multiplied by that wheel's share in
## [code]Values.weight_distribution[/code].
##
## [param i] is the wheel index (0 = front left, 1 = front right, 2 = rear left,
## 3 = rear right).
##
## Returns the sprung mass in kg.
func sprung_mass_calc(car: RigidBody3D, Values: Resource, i: int):
	
	var sprung_mass = 0.0

	sprung_mass = car.mass * Values.weight_distribution[i]
	return sprung_mass
		

## Calculates the spring rate (stiffness) of one wheel from its ride frequency.
##
## Using the spring-mass relation [code]k = m * ω²[/code], where [code]m[/code]
## is the sprung mass of the wheel and [code]ω = TAU * ride_frequency[/code] is
## the natural frequency in rad/s. Choosing a ride frequency in
## [code]Values.ride_frequency[/code] therefore sets the spring stiffness for you,
## and it scales automatically with the car's mass.
##
## [param i] is the wheel index.
##
## Returns the spring rate in N/m.
func spring_rate_calc(car: RigidBody3D, Values: Resource, i: int):
	# frequency based spring rate calc
	var sprung_mass = 0.0
	var omega_n = 0.0

	sprung_mass = car.mass * Values.weight_distribution[i]
	omega_n = TAU * Values.ride_frequency[i]
	
	var k = sprung_mass * omega_n * omega_n
	return k 


## Calculates the damper force of one wheel.
##
## The speed used is the velocity of the car body at the ground contact point,
## projected onto the spring's up direction (the ray's local Y axis). The
## damping coefficient is the critical damping value
## [code]2 * sqrt(k * m)[/code] multiplied by the wheel's damper ratio from
## [code]Values.damper_ratio[/code]. The force is then
## [code]c * |v|^velocity_exponent[/code], with the sign of the velocity, so it
## always opposes the motion. A [code]velocity_exponent[/code] of 1.0 gives
## ordinary linear damping.
##
## [param k] is the spring rate from [method spring_rate_calc].[br]
## [param sprung_mass] is the mass from [method sprung_mass_calc].[br]
## [param i] is the wheel index.
##
## Returns the damper force in newtons. The caller subtracts it from the spring
## force.
##
## [b]Note:[/b] both sides of the ternary use [code]damper_ratio[/code], so
## compression (bump) and extension (rebound) are damped equally. To tune them
## separately, give one side its own value.
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
	

## Combines the spring, damper and anti-roll forces, applies them to the car and
## moves the wheel mesh.
##
## The spring force is [code]k * compression[/code]. The total force along the
## spring's up direction is
## [code]spring_force - damping + anti_roll_force[/code]. It is stored in
## [code]Data.wheel_spring_force[i][/code] as a vector (so other scripts, such as
## the tire code, can use it as the wheel's load) and applied to [param car] with
## [method RigidBody3D.apply_force], at the offset from the car's origin to the
## ray's origin (the suspension mounting point).
##
## The wheel mesh is placed at the ground contact point, along the spring
## direction from the ray's origin.
##
## [param k] is the spring rate from [method spring_rate_calc].[br]
## [param spring_dampning] is the damper force from
## [method spring_dampning_calc].[br]
## [param arb_force] is the array from [method anti_roll_bar_calc].[br]
## [param i] is the wheel index.
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

## Calculates how far the spring of one wheel is compressed and stores it in
## [code]Data.compression[i][/code].
##
## Compression is the ray's rest length (the length of its
## [code]target_position[/code]) minus the distance to the hit point. The result
## is clamped between 0 and [code]Values.max_compression[i][/code], so the
## suspension can never extend below its rest length or compress past its travel
## limit.
##
## [param i] is the wheel index.
func compression_calc(ray: RayCast3D, Data: RuntimeData.suspension, Values: Resource, i: int):
	
	var hit = ray.get_collision_point()
	var hit_distance = ray.global_position.distance_to(hit)
	var rest_length = ray.target_position.length()
	Data.compression[i] = clamp(rest_length - hit_distance, 0, Values.max_compression[i]) 


## Returns the velocity of the car body at a point in world space.
##
## This is the linear velocity of the body plus the velocity caused by its
## rotation at that point ([code]angular_velocity × offset[/code]), where the
## offset is measured from the body's origin.
##
## [param point] is the world-space position to evaluate.[br]
## [param car] is the rigid body.
##
## Returns the world-space velocity in m/s.
func _get_point_velocity(point: Vector3, car: RigidBody3D) -> Vector3:
	return car.linear_velocity + car.angular_velocity.cross(point - car.global_position)
