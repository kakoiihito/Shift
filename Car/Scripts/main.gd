# Note: All comments are to explain the code below the comment like this one.
# Note: why the hell did i make a comment for basic logic
# Note: idk man i just work here
extends RigidBody3D

	###########
	# SCRIPTS #
	###########
	
var SuspensionScript = load("res://Car/Scripts/suspension.gd")
var MotorScript = load("res://Car/Scripts/motor.gd")
var SteeringScript = load("res://Car/Scripts/steering.gd")
var TransmissionScript = load("res://Car/Scripts/transmission.gd")
var WheelProcessScript = load("res://Car/Scripts/wheel.gd")
var BrakeScript = load("res://Car/Scripts/brake.gd")

var Suspension = SuspensionScript.new()
var Motor = MotorScript.new()
var Steering = SteeringScript.new()
var Transmission = TransmissionScript.new()
var WheelProcess = WheelProcessScript.new()
var Brake = BrakeScript.new()


	##########
	# WHEELS #
	##########
@export var wheels: Array[RayCast3D]
@onready var fl_wheel = $WheelFrontLeft
@onready var fr_wheel = $WheelFrontRight
@onready var rr_wheel = $WheelRearRight
@onready var rl_wheel = $WheelRearLeft
var wheel_base = Values.wheel_base
var track = Values.track
	########################
	# SUSPENSION VARIABLES #
	########################
var rest_length = Values.rest_length
var spring_stiffness = Values.spring_stiffness
var max_compression = Values.max_compression
var wheel_spring_force = Values.wheel_spring_force
var weight_distribution = Values.weight_distribution
var velocity_exponent = Values.velocity_exponent
	######################
	# STEERING VARIABLES #
	######################
var max_tire_turn_angle = Values.max_tire_turn_angle
var tire_turn_speed = Values.tire_turn_speed
	###################
	# BRAKE VARIABLES #
	###################
var max_brake_torque = Values.max_brake_torque
var wheel_brake_torque = Values.wheel_brake_torque
var brake_torque: float
	####################
	# ENGINE VARIABLES #
	####################
@export var torque_curve: Curve
var max_torque = Values.max_torque
var max_rpm = Values.max_rpm
var idle_rpm = Values.idle_rpm
var engine_rpm: float
var wheel_engine_torque = Values.wheel_engine_torque
var engine_angular_velocity: float
var engine_inertia = Values.engine_inertia
var FR_torque_engine = Values.FR_torque_engine
var FL_torque_engine = Values.FL_torque_engine
var RR_torque_engine = Values.RR_torque_engine
var RL_torque_engine = Values.RL_torque_engine
var FR_torque_brake = Values.FR_torque_brake
var FL_torque_brake = Values.FL_torque_brake
var RR_torque_brake = Values.RR_torque_brake
var RL_torque_brake = Values.RL_torque_brake
	##########################
	# TRANSMISSION VARIABLES #
	##########################
	
var is_shifting = Values.is_shifting
var shift_timer = Values.shift_timer
var drive_train_efficeny = Values.drive_train_efficeny
var final_drive = Values.final_drive
var gear_ratio = Values.gear_ratio
var current_gear_ratio: float
var current_gear = Values.current_gear
var max_clutch_torque = Values.max_clutch_torque
var lock_threshold = Values.lock_threshold
var unlock_threshold = Values.unlock_threshold
var clutch_stiffness = Values.clutch_stiffness
var is_clutch_locked: bool
	###################
	# WHEEL VARIABLES #
	###################
var active_wheels_engine: int
var active_wheels_brake: int
var longitude_f: float
var wheel_angular_velocity = Values.wheel_angular_velocity
var F_max = Values.F_max
var longitude_force = Values.longitude_force
var lateral_force = Values.lateral_force
var wheel_radius = Values.wheel_radius
var wheel_mass = Values.wheel_mass
var rolling_resistance_coeff = Values.rolling_resistance_coeff
var friction_coefficient = Values.friction_coefficient
var cornering_stiffness = Values.cornering_stiffness
var wheel_inertia = Values.wheel_inertia

func _ready() -> void:
	
	# If you want torque to be applied to a wheel, it will add it to an active wheel counter to divide
	# Wheel torque per wheel in gas_process
	Suspension.car = self
	
	
	var driven_wheels = [FR_torque_engine, FL_torque_engine, RR_torque_engine, RL_torque_engine]
	var brake_wheels = [FL_torque_brake, FL_torque_brake, RR_torque_brake, RL_torque_brake]
	
	for i in range(4):
		if driven_wheels[i] == true:
			active_wheels_engine += 1
	for i in range(4):
		if brake_wheels[i] == true:
			active_wheels_brake += 1
	
	current_gear_ratio = gear_ratio[current_gear]
	
	
	fl_wheel.set_meta("wheel_index", 0)
	fr_wheel.set_meta("wheel_index", 1)
	rl_wheel.set_meta("wheel_index", 2)
	rr_wheel.set_meta("wheel_index", 3)

func _physics_process(delta: float) -> void:

	transmission_process(delta) #ind
	steering_proccess(delta) #ind
	
	for wheel in wheels:
		suspension_proccess(wheel) # ind
		_get_wheel_angular_velocity(wheel, delta) # relies wheel force and suspension
		_get_wheel_forces(wheel) # relies wheel ang and suspension

	motor_process(delta) # relies wheel ang and forces
	brake_proccess() #relies wheel ang


		
func motor_process(delta: float) -> void:
	
	var clutch_torque_on_engine := 0.0
	var angular_velocity_sum: float = 0.0
	var driven_count: int = 0
	var throttle_input := Input.get_action_strength("Gas")
	var clutch_input := Input.get_action_strength("Clutch")
	var clutch_engagement = (1.0 - clutch_input) * (1.0 - clutch_input)
	var target_engine_ang_vel: float
	var drivetrain_ratio = current_gear_ratio * final_drive
	
	
	var driven_wheels = [FR_torque_engine, FL_torque_engine, RR_torque_engine, RL_torque_engine]
	for i in range(4):
		if driven_wheels[i] == true:
			angular_velocity_sum += wheel_angular_velocity[i]
			driven_count += 1
	
 # torque
	var normalized_rpm = engine_rpm / max_rpm
	var engine_torque: float
	if is_shifting == false:
		engine_torque = torque_curve.sample(normalized_rpm) * max_torque * throttle_input
	else: # if shifting you dont have torque due to engine not being in use
		clutch_engagement = 0.0
		engine_torque = 0.0
		
	var base_friction = 0.01 * max_torque
	var linear_friction = 0.03 * max_torque * normalized_rpm
	var quadratic_friction = 0.02 * max_torque * normalized_rpm * normalized_rpm
	var engine_friction = base_friction + linear_friction + quadratic_friction
	
	# clutch_torque calc, dry model
			
	if driven_count > 0:
		target_engine_ang_vel = (angular_velocity_sum / driven_count) * drivetrain_ratio
		var speed_difference = engine_angular_velocity - target_engine_ang_vel
		var max_transferable_torque = max_clutch_torque * clutch_engagement
					
		if abs(speed_difference) > unlock_threshold:
			clutch_torque_on_engine = clamp(
				-speed_difference * clutch_stiffness,
				-max_transferable_torque,
				max_transferable_torque
			)
		else:
			clutch_torque_on_engine = -sign(speed_difference) * max_transferable_torque
			
	var net_engine_torque = engine_torque - engine_friction + clutch_torque_on_engine
	var engine_angular_accel = net_engine_torque / engine_inertia
	engine_angular_velocity += engine_angular_accel * delta
	engine_angular_velocity = clamp(engine_angular_velocity, idle_rpm * TAU / 60.0, max_rpm * TAU / 60.0)
	engine_rpm = engine_angular_velocity * 60.0 / TAU
	# final calc
	
	var clutch_torque_to_wheels = -clutch_torque_on_engine
	var torque_at_wheels = clutch_torque_to_wheels * (drivetrain_ratio) * drive_train_efficeny
	var per_wheel_torque = torque_at_wheels / driven_count if driven_count > 0 else 0.0
	
	
	for i in range(4):
		if driven_wheels[i]:
			wheel_engine_torque[i] = per_wheel_torque
		else:
			wheel_engine_torque[i] = 0.0
					

		
func transmission_process(delta: float):
	var target_clutch = Input.get_action_strength("Clutch")


# upshift
	if not is_shifting and target_clutch > 0.3:
		if Input.is_action_just_pressed("ShiftUp"):
			if current_gear < gear_ratio.size() - 1:
				current_gear += 1
				is_shifting = true
				shift_timer = 0.2  # realistic shift time in seconds

# down shift
		if Input.is_action_just_pressed("ShiftDown"):
			if current_gear > 0:
				current_gear -= 1
				is_shifting = true
				shift_timer = 0.2  # realistic shift time

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
	
	var input_turn = Input.get_action_strength("SteerLeft") - Input.get_action_strength("SteerRight")
	var steering_amount = input_turn * max_tire_turn_angle
	
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
	
	var wheel_index = ray.get_meta("wheel_index")
	var driven_wheels = [$WheelFrontRight/FrontRightWheel, $WheelFrontLeft/FrontLeftWheel, $WheelRearLeft/RearLeftWheel, $WheelRearRight/RearRightWheel]
	
	if ray.is_colliding():
		
		
		var hit = ray.get_collision_point()
		var up_dir_spring = ray.global_transform.basis.y
		var hit_distance = ray.global_position.distance_to(hit)
		var compression = rest_length[wheel_index] - hit_distance
		compression = clamp(compression, 0.0, max_compression[wheel_index])  

		var damper_ratio = 0.5
		
		var world_vel = _get_point_velocity(hit)
		var relative_vel = up_dir_spring.dot(world_vel)
		
		var sprung_mass = mass * weight_distribution[wheel_index]
		var c_crit = 2.0 * sqrt(spring_stiffness[wheel_index] * sprung_mass)
		var c = damper_ratio * c_crit
		var spring_dampning = c * pow(abs(relative_vel), velocity_exponent) * sign(relative_vel)
		
		
		var spring_force = spring_stiffness[wheel_index] * compression
		var wheel_force_area = hit - global_position
		wheel_spring_force[wheel_index] = (spring_force - spring_dampning) * up_dir_spring
		driven_wheels[wheel_index].position = hit - ray.global_position
		apply_force(wheel_spring_force[wheel_index], wheel_force_area)
		
func _get_point_velocity(point: Vector3) -> Vector3:
	return linear_velocity + angular_velocity.cross(point - global_position)
	
	
##########
# WHEELS #
##########

func _get_wheel_forces(ray: RayCast3D):
	
	var wheel_index = ray.get_meta("wheel_index")
	
	var velocity_at_wheel = _get_point_velocity(ray.global_position)
	var side_dir = ray.global_transform.basis.x #
	var side_velocity = velocity_at_wheel.dot(side_dir)
	var car_speed = velocity_at_wheel.dot(-ray.global_transform.basis.z) 
	
	var slip_angle = 0.0
	if abs(car_speed) > 0.1: 
		slip_angle = atan2(side_velocity, abs(car_speed)) * sign(car_speed)
	
	lateral_force[wheel_index] = -slip_angle * cornering_stiffness * (wheel_spring_force[wheel_index].length() / 2500.0)
	
	# Longitude Force
	var slip_ratio: float
	var wheel_surface_speed = wheel_angular_velocity[wheel_index] * wheel_radius

	if abs(car_speed) < 0.5:
		if abs(wheel_surface_speed) > 0.5:
			slip_ratio = sign(wheel_surface_speed) * 1.0
		else:
			slip_ratio = 0.0
	else:
		slip_ratio = (wheel_surface_speed - car_speed) / abs(car_speed)
		slip_ratio = clamp(slip_ratio, -1.0, 1.0)
	
	var optimal_slip = 0.08
	var slip_sign = sign(slip_ratio)
	var normalized_slip = clamp(abs(slip_ratio) / optimal_slip, 0.0, 1.0)
	var traction_multiplier = slip_sign * normalized_slip
	
	if abs(car_speed) < 0.1 and abs(wheel_surface_speed) < 0.1:
		traction_multiplier = 0.0
		
	longitude_force[wheel_index] = (wheel_spring_force[wheel_index].length() * traction_multiplier * friction_coefficient)
	
	# Traction Circle
	F_max[wheel_index] = friction_coefficient * wheel_spring_force[wheel_index].length()

	var current_long_force = longitude_force[wheel_index]
	var current_lat_force = lateral_force[wheel_index]

	var force_2d = Vector2(current_long_force, current_lat_force)
	if force_2d.length() > F_max[wheel_index]:
		force_2d = force_2d.normalized() * F_max[wheel_index]
		longitude_force[wheel_index] = force_2d.x  
		lateral_force[wheel_index] = force_2d.y    
	# Final calc
	var combined_force = (longitude_force[wheel_index] * -ray.global_transform.basis.z) + (lateral_force[wheel_index] * side_dir) # both vectors combined
	var force_pos = ray.global_position - global_position
	apply_force(combined_force , force_pos)

func _get_wheel_angular_velocity(ray: RayCast3D,delta: float):
	
	var wheel_index = ray.get_meta("wheel_index") 
	
	if not wheels[wheel_index].is_colliding():
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
			

	
