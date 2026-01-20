# Note: All comments are to explain the code below the comment like this one.
# Note: why the hell did i make a comment for basic logic
extends RigidBody3D



# work on enginebraking and manual system 

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

	########################
	# SUSPENSION VARIABLES #
	########################

# These are used in calculation for the formula of the force per wheel to lift up or down for the suspension.

var rest_length: float = 0.5
var spring_stiffness: float = 20000.0
var damper_stiffness: float = 10995.0
var max_compression: float = 0.5
var wheel_spring_force: Vector3

	######################
	# STEERING VARIABLES #
	######################

# The value changes how aggresive it turns. Less is slower but more precise.
# More is faster but harder to control.

var turnStrength: float = 10.0
var max_tire_turn_radius = 90.0
var tire_turn_speed = 10.0

# Variables used to calculate torque, used to power the car's wheels to go forward.

	###################
	# BRAKE VARIABLES #
	###################
	
var max_brake_torque = 200.0 # How much the car can brake
var wheel_brake_torque = [0.0, 0.0, 0.0, 0.0]
var brake_torque: float

	####################
	# ENGINE VARIABLES #
	####################

@export var torque_curve: Curve  # Used to calculate how the car accelerates and how fast it goes.
var max_torque = 2000.0 # used to convert the torque value on the curve to a proper force amount.
var max_rpm = 7000.0 # Max amount of engine rotations
var idle_rpm = 1000.0 # Lowest amount of engine rotations
var gear_ratio = 4.1 # Current power multiplyer of gear
var final_drive = 3.63 # Final gear to multiple torque.
var drive_train_efficeny = 0.85

var wheel_engine_torque = [0.0, 0.0, 0.0, 0.0] # How much power the engine produces



# Torque can be applied at any of the wheels. So, these vars allow the torque to be applied at any wheels neccessary.

var FR_torque_engine = true
var FL_torque_engine = true
var RR_torque_engine = false
var RL_torque_engine = false

var FR_torque_brake = true
var FL_torque_brake = true
var RR_torque_brake = true
var RL_torque_brake = true

	###################
	# WHEEL VARIABLES #
	###################
	
var road_resistance_torque: float # reistance against the road (rolling friction)
var active_wheels_engine: int # How many wheels are using engine
var active_wheels_brake: int # How many wheels are using brakes
var wheel_radius = 0.5 # How big the wheel is.
var wheel_mass = 10.0 # How much the wheel takes up
var wheel_angular_velocity = [0.0, 0.0, 0.0, 0.0] # wheel speed in a direction using rads
var tire_stiffness = 800.0 # Changes how much grip the tire has, makes turning more or less.
var F_max = [0.0, 0.0, 0.0, 0.0] # max amount of traction
var wheel_force = [0.0, 0.0, 0.0, 0.0] # How much the wheel gives force
var slip_ratio: float # how much the wheel is slipping from the ground
var longitude_force = [0.0, 0.0, 0.0, 0.0]
var lateral_force = [0.0, 0.0, 0.0, 0.0]

func _ready() -> void:
	
	# If you want torque to be applied to a wheel, it will add it to an active wheel counter to divide
	# Wheel torque per wheel in gas_process
	
	if FR_torque_engine == true:
		active_wheels_engine += 1
	if FL_torque_engine == true:
		active_wheels_engine += 1
	if RR_torque_engine == true:
		active_wheels_engine += 1
	if RL_torque_engine == true:
		active_wheels_engine+= 1
		
	if FR_torque_brake == true:
		active_wheels_brake += 1
	if FL_torque_brake == true:
		active_wheels_brake += 1
	if RR_torque_brake == true:
		active_wheels_brake += 1
	if RL_torque_brake == true:
		active_wheels_brake+= 1
		
	# Set metadeta so other function can easily spot out the wheel when needed.
	
	fl_wheel.set_meta("wheel_index", 0)
	fr_wheel.set_meta("wheel_index", 1)
	rl_wheel.set_meta("wheel_index", 2)
	rr_wheel.set_meta("wheel_index", 3)

func _physics_process(delta: float) -> void:
	
	for wheel in wheels:
		suspension_proccess(wheel)
	for wheel in wheels:
		_get_wheel_traction(wheel)
	
	_get_wheel_angular_velocity(delta)
	
	steering_proccess(delta)
	brake_proccess(delta)

	
	motor_process()
	
	for wheel in wheels:
		if wheel.is_colliding():
			gas_proccess(wheel)
		
		
func motor_process() -> void:
	# To calculate power neccessary, we must first calculate how fast the wheel must rotate (wheel_rpm),
	# then calculate how fast the engine is moving (engine_rpm), next calculate the engine torque via using a torque curve and converting to proper newton force.
	# Again, we calculate how much the wheel produces power (wheel_torque). Finally, we calculate the force neccesary to push the wheels.


	var angular_velocity_sum := 0.0
	var driven_count := 0

	var throttle_input := Input.get_action_strength("Gas")

# adds the speed of the wheel in rads together into a sum.

	var driven_wheels = [FR_torque_engine, FL_torque_engine, RR_torque_engine, RL_torque_engine]
	for i in range(4):
		if driven_wheels[i]:
			angular_velocity_sum += wheel_angular_velocity[i]
			driven_count += 1

	# wheel rpm calc (wheel rotations per minute). Here the total rad is averaged out, converted to revs, then converted to rpm.
	if driven_count == 0:
		return
		
	var wheel_rpm = (angular_velocity_sum / driven_count) * 60.0 / (2.0 * PI)
	var engine_rpm = wheel_rpm * gear_ratio * final_drive
	
	engine_rpm = clamp(engine_rpm, idle_rpm, max_rpm)

	# curve used to scale the rpm levels and now torque.
	
	var normalized_rpm = engine_rpm / max_rpm
	
	# use the curve and scale it using max_torque. Then based on input apply throttle
	
	var engine_torque = (torque_curve.sample(normalized_rpm) * max_torque) * throttle_input # in N·m
	
	var wheel_torque = (engine_torque * gear_ratio * final_drive * drive_train_efficeny) / active_wheels_engine
	
	# Apply engine braking when throttle is zero
	var engine_braking_torque := 0.0
	if throttle_input == 0.0 and engine_rpm > idle_rpm * 1.1:  # Only apply when significantly above idle
		# Engine braking torque increases with RPM
		engine_braking_torque = engine_rpm / max_rpm * max_torque * 0.15
	
	if FR_torque_engine == true:
		wheel_engine_torque[0] = wheel_torque - engine_braking_torque
	if FL_torque_engine == true:
		wheel_engine_torque[1] = wheel_torque - engine_braking_torque
	if RR_torque_engine == true:
		wheel_engine_torque[2] = wheel_torque - engine_braking_torque
	if RL_torque_engine == true:
		wheel_engine_torque[3] = wheel_torque - engine_braking_torque
	

	# calculate and limit wheel forces
	for i in range(4):
		wheel_force[i] = clamp(
			wheel_engine_torque[i] / wheel_radius,
			-F_max[i],
			F_max[i]
		)

					

func gas_proccess(ray: RayCast3D) -> void:
	if not ray.is_colliding():
		return  # Don't apply force if wheel is off ground
	
	if Input.is_action_pressed("Gas"):
		var wheel_index = ray.get_meta("wheel_index")
		var forward = -ray.global_transform.basis.z
		
		# Calculate actual force considering traction limits

		var requested_force = wheel_force[wheel_index]
		
		 # find the place to apply and direction and do it
		var wheel_force_vector = requested_force * forward
		var wheel_force_pos = ray.global_position - global_position
		apply_force(wheel_force_vector, wheel_force_pos)
		
func brake_proccess(delta: float) -> void:
	
	var wheel_inertia = 0.5 * wheel_mass * pow(wheel_radius, 2) # How much the wheel resists torque/motion
	
	if Input.is_action_pressed("Brake"):
		
		var input_brake = Input.get_action_strength("Brake") # strength of input
		
		brake_torque = (input_brake * max_brake_torque) / active_wheels_brake # formula of torque divided by wheels using it
		
		# If brake torque is enabled on that wheel, it will send a value to that wheel value which will be used in calculations.
		
		for i in range(4):
			if (i == 0 and FR_torque_brake) or (i == 1 and FL_torque_brake) or (i == 2 and RR_torque_brake) or (i == 3 and RL_torque_brake):
				wheel_brake_torque[i] = brake_torque
				wheel_angular_velocity[i] = max(wheel_angular_velocity[i] - wheel_brake_torque[i] / wheel_inertia * delta, 0)

	else:
		for i in range(4):
			wheel_brake_torque[i] = 0.0 # to reset any power when not in use
		
func steering_proccess(delta: float) -> void:
	
	# When you check the strength it returns a number value and either Right or Left are assigned a value.
	# So, we can base the direction the car steers on that.
	
	var input_turn = Input.get_action_strength("SteerLeft") - Input.get_action_strength('SteerRight')
	
	# Since the turn strength is multiplied by the turn it will turn based on the input, and since its multipled
	# delta, it will turn gradually.
	
	if input_turn != 0:
		fl_wheel.rotation.y += deg_to_rad(turnStrength * input_turn) * delta
		fr_wheel.rotation.y += deg_to_rad(turnStrength * input_turn) * delta

		fl_wheel.rotation.y = clamp(fl_wheel.rotation.y, deg_to_rad(-max_tire_turn_radius), deg_to_rad(max_tire_turn_radius))
		fr_wheel.rotation.y = clamp(fr_wheel.rotation.y, deg_to_rad(-max_tire_turn_radius), deg_to_rad(max_tire_turn_radius))
	else:
		
		## to reset wheel to starting position smoothly
		
		fl_wheel.rotation.y = move_toward(fl_wheel.rotation.y, 0.0, tire_turn_speed * delta)
		fr_wheel.rotation.y = move_toward(fr_wheel.rotation.y, 0.0, tire_turn_speed * delta)
		
		
func suspension_proccess(ray: RayCast3D):
	
	# If the ray is not touching anything its pointless to attempt to calculate anything
	
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

		# This section is used to calculate dampning. Based on the spring's velocity calculated in the function below,
		# it is multiplied with how much the spring should be dampned. The spring dampning calculates how much the spring should not move.
		# So more dampning is softer and plush, but less dampning is harsh and rough.
		var damper_ratio = 0.5
		
		var world_vel = _get_point_velocity(hit)
		var relative_vel = up_dir_spring.dot(world_vel)
		var c_crit = 2.0 * sqrt(spring_stiffness * mass / wheels.size())
		var c = damper_ratio * c_crit
		var spring_dampning = c * relative_vel
		# This section is used to calculate how much force and where the force should be applied to mimic a car suspension.
		# The spring force is calculated via how fast the spring moves (spring_stiffness) and how much it should move (compression).
		# Then, based on spring_force, we subtract how much movement based on dampning and multiply the force by the direction it should move in.
		# Finally we calculate the area the force should be in and apply both wheel_force and the force area (wheel_force_area) to have a result of suspension.
		
		var spring_force = spring_stiffness * compression
		wheel_spring_force = (spring_force - spring_dampning) * up_dir_spring
		var wheel_force_area = hit - ray.global_position
		apply_force(wheel_spring_force, wheel_force_area)

func _get_point_velocity(point: Vector3) -> Vector3:
	# A physics forumla used to calculate dampning.
	return linear_velocity + angular_velocity.cross(point - global_position)
	
	
##########
# WHEELS #
##########

func _get_wheel_traction(ray: RayCast3D):
	
	var wheel_index = ray.get_meta("wheel_index") # wheel meta data
	
	var velocity_at_wheel = linear_velocity + angular_velocity.cross(
		ray.global_position - global_position
	)
	

	
	# Steering is applied through here. When you steer the wheel is not doing the steering all for you.
	# A force pushes the car in the direction of the side of the wheel. This results in the car turning

	var side_dir = ray.global_transform.basis.x # The speed to the direction to the side of the car
	var side_velocity = velocity_at_wheel.dot(side_dir) # How fast the car is going from the side.
	var car_speed = velocity_at_wheel.dot(-ray.transform.basis.z) # car speed in forward direction
	
	
	var slip_angle = 0.0
	if abs(car_speed) > 0.1:  # Check car_speed, not slip_angle!
		slip_angle = atan2(side_velocity, abs(car_speed))
	
	lateral_force[wheel_index] = (-slip_angle * tire_stiffness)
	
	# Longitude Force

	var wheel_surface_speed = wheel_angular_velocity[wheel_index] * wheel_radius # how fast the wheel is moving based on the ground
	if abs(car_speed) > 0.5:
		slip_ratio = (wheel_surface_speed - car_speed) / abs(car_speed) # slip ratio decides whether wheel is spinning same, less, or more than the speed of car
	
  # Better traction curve
	var optimal_slip = 10.0
	var normalized_slip = slip_ratio / optimal_slip
	var traction_multiplier = (2.0 * normalized_slip) / (1.0 + normalized_slip * normalized_slip)
	traction_multiplier = clamp(traction_multiplier, -1.0, 1.0)
	
	if abs(car_speed) < 0.1 and abs(wheel_surface_speed) < 0.1:
		traction_multiplier = 0.0
	
	longitude_force[wheel_index] = (wheel_spring_force.length() * traction_multiplier)

	
	# Traction Calc

	# F_max = μ * F_normal (friction coefficient × normal force)
	var friction_coefficient = 1.0  # Typical tire friction

	F_max[wheel_index] = friction_coefficient * wheel_spring_force.length()
	
# Get the specific force values for THIS wheel
	var current_long_force = longitude_force[wheel_index]  # Get float from array
		   # If lateral_force is not an array

# OR if lateral_force is also an array:
	var current_lat_force = lateral_force[wheel_index]     # Get float from array

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


func _get_wheel_angular_velocity(delta: float):
	for i in range(4):
		if not wheels[i].is_colliding():
			# Wheel in air - free spin with damping
			wheel_angular_velocity[i] *= 0.99
			continue
		
		# Real physics: τ = I * α
		var wheel_inertia = 0.5 * wheel_mass * wheel_radius * wheel_radius
		
		# Net torque = engine torque - brake torque
		var net_torque = wheel_engine_torque[i] - wheel_brake_torque[i]
		
		# Angular acceleration = τ / I
		var angular_acceleration = net_torque / wheel_inertia if wheel_inertia > 0 else 0
		
		# Update angular velocity: ω = ω₀ + α * Δt
		wheel_angular_velocity[i] += angular_acceleration * delta
		
		# Rolling resistance (simplified)
		var rolling_resistance = 0.01 * abs(wheel_spring_force.length()) * wheel_radius
		wheel_angular_velocity[i] -= sign(wheel_angular_velocity[i]) * rolling_resistance * delta
		
		

	
