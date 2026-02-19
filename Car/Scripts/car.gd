# Note: All comments are to explain the code below the comment like this one.
# Note: why the hell did i make a comment for basic logic
# Note: idk man i just work here
extends RigidBody3D



# work on adding rev matching for gears. finally you wont be stuck at max rpm.

	##########
	# WHEELS #
	##########

# To give access to the wheels/raycasts. These will be used to calculate suspension physics, tire physics,
# and much more.

@export var wheels: Array[RayCast3D]
@onready var fl_wheel = $WheelFrontLeft
@onready var fr_wheel = $WheelFrontRight
@onready var rr_wheel = $WheelRearRight
@onready var rl_wheel = $WheelRearLeft
var wheel_base = 4.0
var track = 2.0

	########################
	# SUSPENSION VARIABLES #
	########################

# These are used in calculation for the formula of the force per wheel to lift up or down for the suspension.

var rest_length: float = 0.25
var spring_stiffness: float = 36800.0
var max_compression: float = 0.364
var wheel_spring_force = [Vector3(), Vector3(), Vector3(), Vector3()]

	######################
	# STEERING VARIABLES #
	######################

# The value changes how aggresive it turns. Less is slower but more precise.
# More is faster but harder to control.

var max_tire_turn_angle = 40.0
var tire_turn_speed = 3.0

# Variables used to calculate torque, used to power the car's wheels to go forward.

	###################
	# BRAKE VARIABLES #
	###################
	
var max_brake_torque = 2000.0 # How much the car can brake
var wheel_brake_torque = [0.0, 0.0, 0.0, 0.0]
var brake_torque: float

	####################
	# ENGINE VARIABLES #
	####################

@export var torque_curve: Curve  # Used to calculate how the car accelerates and how fast it goes.
var max_torque = 151.0 # used to convert the torque value on the curve to a proper force amount.
var max_rpm = 7500.0 # Max amount of engine rotations
var idle_rpm = 850.0 # Lowest amount of engine rotations
var engine_rpm: float
var wheel_engine_torque = [0.0, 0.0, 0.0, 0.0] # How much power the engine produces
var engine_angular_velocity: float
var engine_inertia := 0.25  # kg * m^2
# Torque can be applied at any of the wheels. So, these vars allow the torque to be applied at any wheels neccessary.

var FR_torque_engine = false
var FL_torque_engine = false
var RR_torque_engine = true
var RL_torque_engine = true

var FR_torque_brake = true
var FL_torque_brake = true
var RR_torque_brake = true
var RL_torque_brake = true

	##########################
	# TRANSMISSION VARIABLES #
	##########################
	
var is_shifting = false
var shift_timer = 0.0
var drive_train_efficeny = 0.85
var final_drive = 3.63 # Final gear to multiple torque.
var gear_ratio = [-3.1, 0.0, 3.1, 1.8, 1.3, 1.0, 0.8] # power multiplyer for engine
var current_gear_ratio: float
var current_gear = 1
var clutch_dampning = 1000.0
var max_clutch_torque = 220.0 # max amount of engine torque that can be transfered to the wheels
var lock_threshold = 0.2

	###################
	# WHEEL VARIABLES #
	###################
	
var road_resistance_torque: float # reistance against the road (rolling friction)
var active_wheels_engine: int # How many wheels are using engine
var active_wheels_brake: int # How many wheels are using brakes
var wheel_radius = 0.3 # How big the wheel is.
var wheel_mass = 20.0 # How much the wheel takes up
var wheel_angular_velocity = [0.0, 0.0, 0.0, 0.0] # wheel speed in a direction using rads
var tire_stiffness = 800.0 # Changes how much grip the tire has, makes turning more or less.
var F_max = [0.0, 0.0, 0.0, 0.0] # max amount of traction
var wheel_force = [0.0, 0.0, 0.0, 0.0] # How much the wheel gives force
var slip_ratio: float # how much the wheel is slipping from the ground
var longitude_force = [0.0, 0.0, 0.0, 0.0]
var lateral_force = [0.0, 0.0, 0.0, 0.0]
var rolling_resistance_coeff = 0.015
var friction_coefficient = 1.0

func _ready() -> void:
	
	# If you want torque to be applied to a wheel, it will add it to an active wheel counter to divide
	# Wheel torque per wheel in gas_process
	
	var driven_wheels = [FR_torque_engine, FL_torque_engine, RR_torque_engine, RL_torque_engine]
	var brake_wheels = [FL_torque_brake, FL_torque_brake, RR_torque_brake, RL_torque_brake]
	
	for i in range(4):
		if driven_wheels[i] == true:
			active_wheels_engine += 1
	for i in range(4):
		if brake_wheels[i] == true:
			active_wheels_brake += 1
	
	# Set metadeta so other function can easily spot out the wheel when needed.
	
	fl_wheel.set_meta("wheel_index", 0)
	fr_wheel.set_meta("wheel_index", 1)
	rl_wheel.set_meta("wheel_index", 2)
	rr_wheel.set_meta("wheel_index", 3)

func _physics_process(delta: float) -> void:

	motor_process(delta)
	transmission_process(delta)
	

	for wheel in wheels:
		suspension_proccess(wheel)
		_get_wheel_angular_velocity(wheel, delta)
		_get_wheel_traction(wheel)
	
	steering_proccess(delta)
	brake_proccess()
	
	# Apply forces
	if FR_torque_engine == true:
		gas_proccess(fr_wheel)
	if FL_torque_engine == true:
		gas_proccess(fl_wheel)
	if RR_torque_engine == true:
		gas_proccess(rr_wheel)
	if RL_torque_engine == true:
		gas_proccess(rl_wheel)
		
		
func motor_process(delta: float) -> void:
	# To calculate power neccessary, we must first calculate how fast the wheel must rotate (wheel_rpm),
	# then calculate how fast the engine is moving (engine_rpm), next calculate the engine torque via using a torque curve and converting to proper newton force.
	# Again, we calculate how much the wheel produces power (wheel_torque). Finally, we calculate the force neccesary to push the wheels.
	
	var clutch_torque_on_engine := 0.0
	var angular_velocity_sum: float = 0.0
	var driven_count: int = 0
	var throttle_input := Input.get_action_strength("Gas")
	var clutch_input := Input.get_action_strength("Clutch")
	var clutch_engagement = (1.0 - clutch_input)
	var target_engine_ang_vel: float
	var drivetrain_ratio = current_gear_ratio * final_drive
	
	# get total angular velocity from each wheel and add a counter to how many wheels are being used
	
	if Input.is_action_pressed("Ignition"):
		engine_rpm = 5000.0
	
	var driven_wheels = [FR_torque_engine, FL_torque_engine, RR_torque_engine, RL_torque_engine]
	for i in range(4):
		if driven_wheels[i]:
			angular_velocity_sum += wheel_angular_velocity[i]
			driven_count += 1
		
	# engine rpm calc and refresh
		
	engine_rpm = engine_angular_velocity * 60.0 / TAU
	
	# use the curve and scale it using max_torque. Then based on input apply throttle
	var normalized_rpm = engine_rpm / max_rpm
	var engine_torque: float
	if is_shifting == false:
		engine_torque = torque_curve.sample(normalized_rpm) * max_torque * throttle_input
	else: # if shifting you dont have torque due to engine not being in use
		clutch_engagement = 0.0
		engine_torque = 0.0
		
	# engine experiences losses with each rotation
	var base_friction = 0.005 * max_torque
	var linear_friction = 0.015 * max_torque * normalized_rpm
	var quadratic_friction = 0.008 * max_torque * normalized_rpm * normalized_rpm
	var engine_friction = base_friction + linear_friction + quadratic_friction
	

	# clutch_torque calc (basically how much torque will be used from engine), dry model
	
	var max_transferable_torque = max_clutch_torque * clutch_engagement #max amount of torque that can be used
	
	if driven_count > 0:
		target_engine_ang_vel = (angular_velocity_sum / driven_count) * drivetrain_ratio # needed speed to transfer
		var speed_difference = engine_angular_velocity - target_engine_ang_vel # is it faster or slower than the target
		if abs(speed_difference) < lock_threshold and clutch_engagement > 0.9: # if clutch is fully connected
			var torque_to_sync = speed_difference * clutch_dampning
			clutch_torque_on_engine = clamp(torque_to_sync, -max_transferable_torque, max_transferable_torque)
		else:
			clutch_torque_on_engine = sign(speed_difference) * max_transferable_torque # when slipping
		
	var net_engine_torque = engine_torque - engine_friction - clutch_torque_on_engine
	var engine_angular_accel = net_engine_torque / engine_inertia
	engine_angular_velocity += engine_angular_accel * delta # how fast the pistons in the engine are moving. used to calcualte rpm
	 
	# recompute rpm due to key variable change
	engine_angular_velocity = clamp(engine_angular_velocity, idle_rpm * TAU / 60.0, max_rpm * TAU / 60.0)
	engine_rpm = engine_angular_velocity * 60.0 / TAU
	
	# final calc
	
	var clutch_torque_to_wheels = clutch_torque_on_engine
	var torque_at_wheels = clutch_torque_to_wheels * drivetrain_ratio * drive_train_efficeny
	var per_wheel_torque = torque_at_wheels / driven_count if driven_count > 0 else 0.0

		#spread the torque across wheels.
	for i in range(4):
		if driven_wheels[i]:
			wheel_engine_torque[i] = per_wheel_torque
		else:
			wheel_engine_torque[i] = 0.0
	
	#limit wheel forces
	for i in range(4):
		wheel_force[i] = clamp(
			wheel_engine_torque[i] / wheel_radius,
			-F_max[i],
			F_max[i]
		)
					
func gas_proccess(ray: RayCast3D) -> void:
	if not ray.is_colliding():
		return  # Don't apply force if wheel is off ground
	
	var wheel_index = ray.get_meta("wheel_index")
	var forward = -ray.global_transform.basis.z

	var requested_force = wheel_force[wheel_index]
	
	 # find the place to apply and direction and do it
	var wheel_force_vector = requested_force * forward
	var wheel_force_pos = ray.global_position - global_position
	apply_force(wheel_force_vector, wheel_force_pos)
		
		
func transmission_process(delta: float):
	var target_clutch = Input.get_action_strength("Clutch")


# upshift
	if not is_shifting and target_clutch > 0.3:
		if Input.is_action_just_pressed("ShiftUp"):
			if current_gear < gear_ratio.size() - 1:
				current_gear += 1
				is_shifting = true
				shift_timer = 0.3  # realistic shift time in seconds

# down shift
		if Input.is_action_just_pressed("ShiftDown"):
			if current_gear > 0:
				current_gear -= 1
				is_shifting = true
				shift_timer = 0.3  # realistic shift time

# countdown system before another shift can be made
	if is_shifting:
		shift_timer -= delta
		if shift_timer <= 0.0:
			current_gear_ratio = gear_ratio[current_gear]
			is_shifting = false

				
	
func brake_proccess() -> void:
	
	# actual braking is done in wheel angular velocity calculation
	
	var brake_wheels = [FR_torque_brake, FL_torque_brake, RR_torque_brake, RL_torque_brake]
	var input_brake = Input.get_action_strength("Brake") # strength of input
	
	if input_brake > 0.0:
		
		brake_torque = (input_brake * max_brake_torque) / active_wheels_brake # formula of torque divided by wheels using it
		
		for i in range(4):
			if brake_wheels[i] == true:
				var brake_direction = 1.0
				if wheel_angular_velocity[i] != 0.0:
					brake_direction = sign(wheel_angular_velocity[i])
				else:
					brake_direction = 0.0
				wheel_brake_torque[i] = brake_torque * brake_direction
	else:
		for i in range(4):
			wheel_brake_torque[i] = 0.0 # to reset any power when not in use
		
func steering_proccess(delta: float) -> void:
	
	# When you check the strength it returns a number value and either Right or Left are assigned a value.
	# So, we can base the direction the car steers on that.
	
	var input_turn = Input.get_action_strength("SteerLeft") - Input.get_action_strength("SteerRight")
	var steering_amount = input_turn * max_tire_turn_angle
	# Since the turn strength is multiplied by the turn it will turn based on the input, and since its multipled
	# delta, it will turn gradually.

	if abs(input_turn) < 0.01:
		fl_wheel.rotation.y = move_toward(fl_wheel.rotation.y, 0.0, tire_turn_speed * delta)
		fr_wheel.rotation.y = move_toward(fr_wheel.rotation.y, 0.0, tire_turn_speed * delta)
		return
	var L = wheel_base
	var W = track
	var R = L / tan(abs(steering_amount))

	var inner = atan(L / (R - W * 0.5))
	var outer = atan(L / (R + W * 0.5))
	
	var target_fl := 0.0
	var target_fr := 0.0

	if steering_amount > 0.0:
		# Turning left
		target_fr = -inner
		target_fl = -outer
	else:
		# Turning right
		target_fl = inner
		target_fr = outer
		
	fl_wheel.rotation.y = move_toward(
		fl_wheel.rotation.y,
		target_fl,
		tire_turn_speed * delta
	)
	fr_wheel.rotation.y = move_toward(
		fr_wheel.rotation.y,
		target_fr,
		tire_turn_speed * delta
	)

		
func suspension_proccess(ray: RayCast3D):
	
	# If the ray is not touching anything its pointless to attempt to calculate anything
	var wheel_index = ray.get_meta("wheel_index")
	
	if ray.is_colliding():
		
		# Here we use the ray to calculate the distance from the ground to the wheel and and subtract that
		# from the rest length since that will tell us if the wheel should extend or contract.
		
		# To explain further, the rest length means 0. If we add to that it will contract and subtracting will
		# extend it. the subtracting and adding are the hit distance and the rest length factors where and how the spring will react.
		
		var hit = ray.get_collision_point()
		var up_dir_spring = ray.global_transform.basis.y
		var hit_distance = ray.global_position.distance_to(hit)
		var compression = sign(rest_length - hit_distance)
		compression = clamp(compression, 0, max_compression)  

		# Spring dampning is calculated by the spring's speed and dampning coefficent (amount).
		# To get the coefficent, a damper ratio and critical dampning variable are needed.
		# once we get the coefficent, we multiply by spring speed and the dampning is found.
		var damper_ratio = 0.3
		
		var world_vel = _get_point_velocity(hit)
		var relative_vel = up_dir_spring.dot(world_vel)
		
		var c_crit = 2.0 * sqrt(spring_stiffness * mass / wheels.size())
		var c = damper_ratio * c_crit
		var spring_dampning = c * relative_vel
		
		# This section is used to calculate how much force and where the force should be applied to mimic a car suspension.
		# The spring force is calculated via how fast the spring will move (spring_stiffness) and how much it is compressed or extended (compression).
		# Then, based on spring_force, we subtract how much movement based on dampning and multiply the force by the direction it should move in.
		# Finally we calculate the area the force should be in and apply both wheel_force and the force area (wheel_force_area) to have a result of suspension.
		
		var spring_force = spring_stiffness * compression
		wheel_spring_force[wheel_index] = (spring_force - spring_dampning) * up_dir_spring
		var wheel_force_area = hit - ray.global_position
		apply_force(wheel_spring_force[wheel_index], wheel_force_area)




func _get_point_velocity(point: Vector3) -> Vector3:
	# A physics forumla used to calculate dampning.
	return linear_velocity + angular_velocity.cross(point - global_position)
	
	
##########
# WHEELS #
##########

func _get_wheel_traction(ray: RayCast3D):
	
	  # Typical tire friction
	
	var wheel_index = ray.get_meta("wheel_index") # wheel meta data
	
	var velocity_at_wheel = _get_point_velocity(ray.global_position)
	

	
	# Steering is applied through here. When you steer the wheel is not doing the steering all for you.
	# A force pushes the car in the direction of the side of the wheel. This results in the car turning

	var side_dir = ray.global_transform.basis.x # The speed to the direction to the side of the car
	var side_velocity = velocity_at_wheel.dot(side_dir) # How fast the car is going from the side.
	var car_speed = velocity_at_wheel.dot(-ray.transform.basis.z) # car speed in forward direction
	
	var slip_angle = 0.0
	if abs(car_speed) > 0.1: 
		slip_angle = atan2(side_velocity, abs(car_speed))
	
	var cornering_stiffness = 400.0 
	lateral_force[wheel_index] = -slip_angle * cornering_stiffness * (wheel_spring_force[wheel_index].length() / 5000.0)  # Normalize to typical load

	
	# Longitude Force

	var wheel_surface_speed = wheel_angular_velocity[wheel_index] * wheel_radius # how fast the wheel is moving based on the ground
	if abs(car_speed) > 0.5:
		slip_ratio = (wheel_surface_speed - car_speed) / abs(car_speed) # slip ratio decides whether wheel is spinning same, less, or more than the speed of car
	else: slip_ratio = 0.0
	
	var optimal_slip = 0.08
	var slip_sign = sign(slip_ratio)
	var normalized_slip = abs(slip_ratio) / optimal_slip
	var traction_multiplier = (2.0 * normalized_slip) / (1.0 + normalized_slip * normalized_slip)
	traction_multiplier = clamp(traction_multiplier * slip_sign, -1.0, 1.0)
	
	if abs(car_speed) < 0.1 and abs(wheel_surface_speed) < 0.1:
		traction_multiplier = 0.0
	
	longitude_force[wheel_index] = (wheel_spring_force[wheel_index].length() * traction_multiplier * friction_coefficient)

	F_max[wheel_index] = friction_coefficient * wheel_spring_force[wheel_index].length()
	

	var current_long_force = longitude_force[wheel_index] 

	var current_lat_force = lateral_force[wheel_index]

# Now create Vector2 with floats
	var force_2d = Vector2(current_long_force, current_lat_force)
	if force_2d.length() > F_max[wheel_index]:
		force_2d = force_2d.normalized() * F_max[wheel_index]
		longitude_force[wheel_index] = force_2d.x  
		lateral_force[wheel_index] = force_2d.y    
	# Final calc
	var combined_force = (longitude_force[wheel_index] * -ray.global_transform.basis.z) + (lateral_force[wheel_index] * side_dir) # both vectors combined
	var force_pos = ray.global_position - global_position
	apply_force(combined_force, force_pos)

func _get_wheel_angular_velocity(ray: RayCast3D,delta: float):
	var wheel_index = ray.get_meta("wheel_index") # wheel meta data
	var wheel_inertia = 0.8 * wheel_mass * wheel_radius * wheel_radius
	
	if not wheels[wheel_index].is_colliding():
		var air_drag_torque = 0.001 * wheel_angular_velocity[wheel_index] * abs(wheel_angular_velocity[wheel_index])
		var angular_decel = air_drag_torque / wheel_inertia
		wheel_angular_velocity[wheel_index] -= angular_decel * delta
	else:
		var normal_force = wheel_spring_force[wheel_index].length()
		var rolling_resistance = rolling_resistance_coeff * normal_force * wheel_radius * sign(wheel_angular_velocity[wheel_index])
		
		if wheel_angular_velocity[wheel_index] == 0.0:
			rolling_resistance = 0.0
			
		var ground_reaction_torque = longitude_force[wheel_index] * wheel_radius
		var signed_brake_torque = wheel_brake_torque[wheel_index] * sign(wheel_angular_velocity[wheel_index])
		var net_torque = wheel_engine_torque[wheel_index] - signed_brake_torque - ground_reaction_torque - rolling_resistance
		
		var angular_acceleration = net_torque / wheel_inertia if wheel_inertia > 0 else 0
		
		wheel_angular_velocity[wheel_index] += angular_acceleration * delta
		
	if abs(wheel_angular_velocity[wheel_index]) < 0.1:
		wheel_angular_velocity[wheel_index] = 0.0


		
		# wheel angular velocity is the root problem. brake causes it to go into the negatives which then moves the car backwards.
		# motor process then uses it which causes it's torque to go backwards. whole problem is the brake torque.

	
