# Portions are Copyright (c) 2021 Dechode
# https://github.com/Dechode/Godot-Advanced-Vehicle

# Portions are Copyright (c) 2024 Baron Wittman
# https://lupine-vidya.itch.io/gdsim/devlog/677572/series-driving-simulator-workshop-mirror

class_name Wheel
extends RayCast3D

## The [Node3D] correlating to this Wheel, which will have its
## rotation manipulated to make it spin and rotate.
## [br][br]
## [b]Tip:[/b] Make sure that your wheel is facing in the [b]+Z[/b] axis
## as this is considered the forward direction by both Godot and this script.
## [br][br]
## [b]Tip:[/b] If you're having issues with positioning your wheel,
## try parenting it to a [Node3D] and using that as the wheel node instead.
@export var wheel_node : Node3D

var current_steer_angle: float = 0.0  # keeps state across frames
var wheel_mass := 15.0
var tire_radius := 0.3
var tire_width := 205.0
var ackermann := 0.15
var contact_patch := 0.2
var braking_grip_multiplier := 1.4
var surface_type := ""
var tire_stiffnesses := { "Road" : 5.0, "Dirt" : 0.5, "Grass" : 0.5 }
var coefficient_of_friction := { "Road" : 2.0, "Dirt" : 1.4, "Grass" : 1.0 }
var rolling_resistance := { "Road" : 1.0, "Dirt" : 2.0, "Grass" : 4.0 }
var lateral_grip_assist := { "Road" : 0.05, "Dirt" : 0.0, "Grass" : 0.0}
var longitudinal_grip_ratio := { "Road" : 0.5, "Dirt": 0.5, "Grass" : 0.5}

var spring_length := 0.15
var spring_rate := 0.0
var slow_bump := 0.0
var fast_bump := 0.0
var slow_rebound := 0.0
var fast_rebound := 0.0
var fast_damp_threshold := 127.0
var antiroll := 0.0
var toe := 0.0
var bump_stop_multiplier := 1.0
var wheel_to_body_torque_multiplier := 0.0
var mass_over_wheel := 0.0

var wheel_moment := 0.0
var spin := 0.0
var spin_velocity_diff := 0.0
var spring_force := 0.0
var applied_torque := 0.0
var local_velocity := Vector3.ZERO
var previous_velocity := Vector3.ZERO
var previous_global_position := Vector3.ZERO
var force_vector := Vector2.ZERO
var slip_vector := Vector2.ZERO
var previous_compression := 0.0
var spring_current_length := 0.0
var max_spring_length := 0.0
var antiroll_force := 0.0
var damping_force := 0.0
var steering_ratio := 0.0
var last_collider
var last_collision_point := Vector3.ZERO
var last_collision_normal := Vector3.ZERO
var current_cof := 0.0
var current_rolling_resistance := 0.0
var current_lateral_grip_assist := 0.0
var current_longitudinal_grip_ratio := 0.0
var current_tire_stiffness := 0.0
var abs_enable_time := 0.0
var abs_pulse_time := 0.3
var abs_spin_difference_threshold := -12.0
var limit_spin := false
var is_driven := false
var opposite_wheel : Wheel
var beam_axle := 0.0

var vehicle : Vehicle

func _process(delta : float) -> void:
	if wheel_node:
		wheel_node.position.y = minf(0.0, -spring_current_length)
		if not is_zero_approx(beam_axle):
			var wheel_lookat_vector := (opposite_wheel.transform * opposite_wheel.wheel_node.position) - (transform * wheel_node.position)
			wheel_node.rotation.z = wheel_lookat_vector.angle_to(Vector3.RIGHT * beam_axle) * signf(wheel_lookat_vector.y * beam_axle)
		wheel_node.rotation.x -= (wrapf(spin * delta, 0, TAU))
		
func initialize() -> void:
	wheel_node.rotation_order = EULER_ORDER_ZXY
	wheel_moment = 0.5 * wheel_mass * pow(tire_radius, 2)
	set_target_position(Vector3.DOWN * (spring_length + tire_radius))
	vehicle = get_parent()
	max_spring_length = spring_length
	current_cof = coefficient_of_friction[surface_type]
	current_rolling_resistance = rolling_resistance[surface_type]
	current_lateral_grip_assist = lateral_grip_assist[surface_type]
	current_longitudinal_grip_ratio = longitudinal_grip_ratio[surface_type]
	current_tire_stiffness = 1000000.0 + 8000000.0 * tire_stiffnesses[surface_type]

# Steering function for raycast wheels
# wheelbase = distance between front and rear axles
# track_width = distance between left and right wheels
# caster_strength = how much steering self-centers
# steer_speed = how quickly wheels respond to input



func steer(input: float, max_steering_angle: float, delta: float, wheelbase: float = 2.6, track_width: float = 1.55, caster_strength: float = 3.0, steer_speed: float = 6.0):
	# Clamp input [-1,1]
	input = clamp(input, -1.0, 1.0)

	# Target angle with ackermann geometry
	var inside_angle = atan(wheelbase / ( (wheelbase / tan(max_steering_angle)) - (track_width * 0.5) ))
	var outside_angle = atan(wheelbase / ( (wheelbase / tan(max_steering_angle)) + (track_width * 0.5) ))
	
	# If input is left, use left wheel ackermann, else right
	var target_angle = 0.0
	if input > 0.0:
		target_angle = lerp(0.0, inside_angle, input)
	elif input < 0.0:
		target_angle = lerp(0.0, -outside_angle, -input)
	
	# Steering ratio scaling
	target_angle *= steering_ratio
	
	# Toe adjustment
	target_angle += toe
	
	# Smooth steering (driver hand speed)
	current_steer_angle = lerp(current_steer_angle, target_angle, steer_speed * delta)
	
	# Self-aligning torque (caster effect)
	current_steer_angle = lerp(current_steer_angle, 0.0, caster_strength * delta * (1.0 - abs(input)))
	
	# Apply to wheel node
	rotation.y = current_steer_angle

func process_torque(
	drive: float,
	drive_inertia: float,
	brake_torque: float,
	allow_abs: bool,
	delta: float
) -> float:
	var net_torque := force_vector.y * tire_radius
	var previous_spin := spin
	
	# ---------------------------
	# 1. Apply drivetrain efficiency losses
	# ---------------------------
	var drivetrain_efficiency := 0.92  # ~8% loss
	drive *= drivetrain_efficiency
	
	net_torque += drive
	
	# ---------------------------
	# 2. ABS logic with slip ratio
	# ---------------------------
	var wheel_linear_speed = spin * tire_radius
	var slip_ratio = 0.0
	if absf(local_velocity.z) > 0.5:
		slip_ratio = (wheel_linear_speed - local_velocity.z) / absf(local_velocity.z)
	
	# Slip threshold ~0.2 (20%) before ABS engages
	if allow_abs and brake_torque > 0.0:
		if absf(slip_ratio) > 0.2:
			brake_torque *= 0.2   # pulse brake force
			abs_enable_time = vehicle.delta_time + abs_pulse_time
	
	# ---------------------------
	# 3. Brake fade under heat (basic model)
	# ---------------------------
	var brake_temp := clamp(brake_torque * 0.0005, 0.0, 1.0) # 0-1 heat factor
	var fade_factor : float = 1.0 - (brake_temp * 0.5) # lose up to 50% force
	brake_torque *= fade_factor
	
	# ---------------------------
	# 4. Engine braking when off throttle
	# ---------------------------
	if is_zero_approx(drive) and absf(local_velocity.z) > 2.0:
		var engine_brake := clamp(absf(local_velocity.z) * 5.0, 0.0, 150.0)
		net_torque -= engine_brake * signf(spin)
	
	# ---------------------------
	# 5. Apply braking torque
	# ---------------------------
	if absf(spin) < 5.0 and brake_torque > absf(net_torque):
		# Full stop lock
		spin = 0.0
	else:
		net_torque -= brake_torque * signf(spin)
		var new_spin = spin + ((net_torque / (wheel_moment + drive_inertia)) * delta)
		
		# Prevent oscillation crossing zero
		if signf(spin) != signf(new_spin) and brake_torque > absf(drive):
			new_spin = 0.0
		
		spin = new_spin
	
	# ---------------------------
	# 6. Applied torque tracking
	# ---------------------------
	if is_zero_approx(spin):
		applied_torque = absf(drive - brake_torque)
	else:
		applied_torque = absf(drive - (brake_torque * signf(spin)))
	
	# ---------------------------
	# 7. Return wheel acceleration factor
	# ---------------------------
	if is_zero_approx(drive * delta):
		return 0.5
	else:
		return (spin - previous_spin) * (wheel_moment + drive_inertia) / (drive * delta)

func process_forces(opposite_compression : float, braking : bool, delta : float) -> float:
	var compression := process_suspension(opposite_compression, delta)
	force_raycast_update()
	previous_velocity = local_velocity
	local_velocity = (global_position - previous_global_position) / delta * global_transform.basis
	previous_global_position = global_position
	
	## Surface detection (friction, grip, stiffness change)
	if is_colliding():
		last_collider = get_collider()
		last_collision_point = get_collision_point()
		last_collision_normal = get_collision_normal()
		var surface_groups : Array[StringName] = last_collider.get_groups()
		if surface_groups.size() > 0:
			if surface_type != surface_groups[0]:
				surface_type = surface_groups[0]
				current_cof = coefficient_of_friction.get(surface_type, 1.0)
				current_rolling_resistance = rolling_resistance.get(surface_type, 0.01)
				current_lateral_grip_assist = lateral_grip_assist.get(surface_type, 1.0)
				current_longitudinal_grip_ratio = longitudinal_grip_ratio.get(surface_type, 1.0)
				## Dynamic tire stiffness scaling
				var stiffness_mult := tire_stiffnesses.get(surface_type, 1.0)
				current_tire_stiffness = (500000.0 + 6000000.0 * stiffness_mult) * clamp(1.0 - compression * 0.2, 0.5, 1.0)
	else:
		last_collider = null
	
	## Process suspension with bump/rebound separation

	
	if is_colliding() and last_collider:
		## Tires generate forces with a response delay (relaxation length)
		var relax_rate := clamp(5.0 * delta, 0.0, 1.0)  # higher = more immediate grip
		process_tires(braking, delta)
		force_vector = force_vector.lerp(force_vector, relax_rate)
		
		var contact := last_collision_point - vehicle.global_position
		
		## Apply spring force (vertical load transfer)
		if spring_force > 0.0:
			vehicle.apply_force(last_collision_normal * spring_force, contact)
		else:
			vehicle.apply_force(-global_transform.basis.y * vehicle.mass * 9.81 * 0.1, global_position - vehicle.global_position)
		
		## Longitudinal + lateral forces
		vehicle.apply_force(global_transform.basis.x * force_vector.x, contact)
		vehicle.apply_force(global_transform.basis.z * force_vector.y, contact)
		
		## Scrub torque (resistance to lateral slip angle)
		var scrub_torque := -slip_vector.x * tire_radius * 0.05
		vehicle.apply_torque(global_transform.basis.y * scrub_torque)
		
		## Weight transfer torque effect during braking
		if braking:
			wheel_to_body_torque_multiplier = 1.0 / (braking_grip_multiplier + 1.0)
		
		vehicle.apply_force(-global_transform.basis.y * force_vector.y * 0.5 * wheel_to_body_torque_multiplier, to_global(Vector3.FORWARD * tire_radius))
		vehicle.apply_force(global_transform.basis.y * force_vector.y * 0.5 * wheel_to_body_torque_multiplier, to_global(Vector3.BACK * tire_radius))
		
		return compression
	
	else:
		## Wheel in air = no grip, but spin down with air drag
		force_vector = Vector2.ZERO
		slip_vector = Vector2.ZERO
		spin -= signf(spin) * delta * (2.0 / wheel_moment + 0.05)  # air drag term added
		return 0.0

func process_suspension(opposite_compression : float, delta : float) -> float:
	if is_colliding() and last_collider:
		spring_current_length = last_collision_point.distance_to(global_position) - tire_radius
	else:
		spring_current_length = spring_length
	
	var no_contact := false
	if spring_current_length > max_spring_length:
		spring_current_length = max_spring_length
		no_contact = true
	
	var bottom_out := false
	if spring_current_length < 0.0:
		spring_current_length = 0.0
		bottom_out = true
	
	## Progressive spring compression (stiffer the more compressed)
	var compression := spring_length - spring_current_length
	var progressive_rate := 1.0 + pow(compression / spring_length, 1.5) * 2.0
	compression *= 100.0 * progressive_rate
	
	## Spring speed (mm/s)
	var spring_speed := (compression - previous_compression) / delta
	previous_compression = compression
	
	## Spring force (linear + progressive)
	spring_force = compression * spring_rate * progressive_rate
	
	## Anti-roll force (scaled by suspension travel)
	var antiroll_effect : float = antiroll * (compression - opposite_compression) * clamp(1.0 + abs(compression - opposite_compression) / spring_length, 0.0, 2.0)
	spring_force += antiroll_effect
	
	## Bottom-out handling
	var bottom_out_force := 0.0
	if bottom_out:
		var gravity_factor := clamp(global_transform.basis.y.dot(-vehicle.current_gravity.normalized()), 0.0, 1.0)
		bottom_out_force = (mass_over_wheel * vehicle.current_gravity.length() * gravity_factor + mass_over_wheel * spring_speed * 0.01) * bump_stop_multiplier
	
	## Damping: separate bump and rebound curves
	var damping_force := 0.0
	if spring_speed >= 0: # Compression
		if spring_speed > fast_damp_threshold:
			damping_force = (spring_speed - fast_damp_threshold) * fast_bump + fast_damp_threshold * slow_bump
		else:
			damping_force = spring_speed * slow_bump
	else: # Rebound
		if spring_speed < -fast_damp_threshold:
			damping_force = (spring_speed + fast_damp_threshold) * fast_rebound + (-fast_damp_threshold * slow_rebound)
		else:
			damping_force = spring_speed * slow_rebound
	
	## Total spring force
	spring_force += damping_force + bottom_out_force
	
	spring_force = max(0, spring_force)
	
	## Adjust max spring length dynamically based on force (prevents instant snap)
	max_spring_length = clamp(((spring_force / wheel_mass) - spring_speed) * delta * 0.001 + spring_current_length, 0.0, spring_length)
	
	if no_contact:
		spring_force = 0.0
	
	return compression

func process_tires(braking: bool, delta: float):
	## Planar velocity (XZ plane)
	var local_planar := Vector2(local_velocity.x, local_velocity.z).normalized() * clampf(local_velocity.length(), 0.0, 1.0)
	
	## Slip angles (simplified brush model basis)
	slip_vector.x = asin(clampf(-local_planar.x, -1.0, 1.0)) # Lateral slip
	slip_vector.y = 0.0
	
	## Wheel velocity vs car velocity
	var wheel_velocity := spin * tire_radius
	spin_velocity_diff = wheel_velocity + local_velocity.z
	var needed_rolling_force := ((spin_velocity_diff * wheel_moment) / tire_radius) / delta
	
	## Force caps (so forces don’t go infinite)
	var max_y_force := max(absf(applied_torque / tire_radius), absf(needed_rolling_force / tire_radius))
	var max_x_force :float = absf(mass_over_wheel * local_velocity.x) / max(delta, 0.0001)
	
	## Slip ratio (longitudinal)
	var z_sign := signf(-local_velocity.z)
	if local_velocity.z == 0.0:
		z_sign = 1.0
	slip_vector.y = (absf(local_velocity.z) - (wheel_velocity * z_sign)) / (1.0 + absf(local_velocity.z))
	
	if slip_vector.is_zero_approx():
		slip_vector = Vector2(0.0001, 0.0001)
	
	## Tire stiffness & friction (simplified brush tire model)
	var cornering_stiffness := 0.5 * current_tire_stiffness * pow(contact_patch, 2.0)
	var friction := current_cof * spring_force - (spring_force / (tire_width * contact_patch * 0.2))
	var deflect := 1.0 / sqrt(pow(cornering_stiffness * slip_vector.y, 2.0) + pow(cornering_stiffness * slip_vector.x, 2.0))
	
	## Braking longitudinal grip boost (ABS-like effect)
	var braking_help := 1.0
	if slip_vector.y > 0.3 and braking:
		braking_help = 1.0 + (braking_grip_multiplier * clampf(absf(slip_vector.y), 0.0, 1.0))
	
	## Pacejka-like combined grip scaling
	var slip_magnitude := sqrt(pow(slip_vector.x, 2.0) + pow(slip_vector.y, 2.0))
	var friction_limit := friction * (1.0 - slip_magnitude) * contact_patch * (0.5 * deflect)
	
	if friction_limit >= contact_patch:
		## High slip (force saturates)
		force_vector.y = cornering_stiffness * slip_vector.y / (1.0 - slip_vector.y)
		force_vector.x = cornering_stiffness * slip_vector.x / (1.0 - slip_vector.y)
	else:
		## Within grip limit
		var brushx := (1.0 - friction * (1.0 - slip_vector.y) * (0.25 * deflect)) * deflect
		force_vector.y = friction * current_longitudinal_grip_ratio * cornering_stiffness * slip_vector.y * brushx * braking_help * z_sign
		force_vector.x = friction * cornering_stiffness * slip_vector.x * brushx * (absf(slip_vector.x * current_lateral_grip_assist) + 1.0)
	
	## Clamp forces to max physical force
	if absf(force_vector.y) > absf(max_y_force):
		force_vector.y = max_y_force * signf(force_vector.y)
		limit_spin = true
	else:
		limit_spin = false
	
	if absf(force_vector.x) > max_x_force:
		force_vector.x = max_x_force * signf(force_vector.x)
	
	## Rolling resistance (small but important)
	force_vector.y -= process_rolling_resistance() * signf(local_velocity.z)

func process_rolling_resistance() -> float:
	# Base rolling resistance coefficient (road + tire deformation)
	var c_rr0 := 0.015  # typical value for road cars (0.010–0.018)
	
	# Small speed-dependent factor (tires heat & deform more with speed)
	var c_rr_speed :float= c_rr0 * (1.0 + 0.015 * abs(local_velocity.z * 3.6) / 100.0)
	
	# Rolling resistance force = Crr * Normal Load
	# local_velocity.z * 0.036 = m/s → km/h, so scale to realistic growth
	var rolling_force :float = c_rr_speed * spring_force * current_rolling_resistance
	
	return rolling_force

func get_reaction_torque(delta: float) -> float:
	# Longitudinal force contribution
	var torque_from_force := force_vector.y * tire_radius
	
	# Wheel surface linear velocity from spin
	var wheel_velocity := spin * tire_radius
	
	# Wheel rotational inertia effect
	var angular_accel :float = (wheel_velocity - local_velocity.z) / max(delta, 0.0001)
	var inertia_torque := -wheel_moment * angular_accel
	
	# Rolling resistance torque
	var rolling_torque := -process_rolling_resistance() * tire_radius * signf(spin)
	
	# Slip ratio adjustment
	var slip_ratio := 0.0
	if absf(local_velocity.z) > 0.5:
		slip_ratio = (wheel_velocity - local_velocity.z) / absf(local_velocity.z)
	var slip_factor := clampf(1.0 - absf(slip_ratio), 0.0, 1.0)
	
	# Total reaction torque
	var reaction_torque := (torque_from_force * slip_factor) + inertia_torque + rolling_torque
	
	return reaction_torque

func get_friction(normal_force : float, surface : String) -> float:
	# Base coefficient of friction (μ) depending on surface
	var surface_cof := 1.0
	if coefficient_of_friction.has(surface):
		surface_cof = coefficient_of_friction[surface]
	
	# --- Tire load sensitivity ---
	# Tires don't scale grip linearly with load.
	# At higher loads, grip increases but at a diminishing rate.
	# This models "friction curve" flattening at high downforce.
	var load_sensitivity := 1.0 / (1.0 + 0.0008 * normal_force)
	
	# Effective μ with load sensitivity
	var effective_cof := surface_cof * (0.9 + 0.1 * load_sensitivity)
	
	# Contact patch effect
	# Wider tires spread load better → more consistent μ
	var patch_factor := clamp(1.0 + (tire_width * contact_patch * 0.0001), 0.9, 1.2)
	
	# Final maximum friction force
	var max_friction :float= effective_cof * normal_force * patch_factor
	
	return max_friction
