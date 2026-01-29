extends Control

@onready var vbox = $MarginContainer/ScrollContainer/VBoxContainer
var vehicle_data = Vehicledata

# Define variables to edit with their metadata (name, type, tooltip)
var editable_vars = {
	# Steering
	"steering_speed": { "type": "float", "desc": "The rate that the steering input changes in order to smooth out direction changes to the wheel. Steering input is between -1 and 1. Speed is in units per second." },
	"countersteer_speed": { "type": "float", "desc": "The rate that the steering input changes when steering back to center. Speed is in units per second." },
	"steering_speed_decay": { "type": "float", "desc": "Reduces steering input based on the vehicle's speed. Steering speed is divided by the velocity at this magnitude. The larger the number, the slower the steering at speed." },
	"steering_slip_assist": { "type": "float", "desc": "Further steering input is prevented if the wheels' lateral slip is greater than this number." },
	"countersteer_assist": { "type": "float", "desc": "The magnitude to adjust steering toward the direction of travel based on the vehicle's lateral velocity." },
	"steering_exponent": { "type": "float", "desc": "Steering input is raised to the power of this number. This slows steering input near the limits." },
	"max_steering_angle": { "type": "float", "desc": "The maximum steering angle in radians. Edited in the inspector in degrees. Use deg_to_rad() in scripts if needed." },
	# Front Axle
	"front_steering_ratio": { "type": "float", "desc": "The ratio that the wheels turn based on steering input. The higher this value, the more the wheels will turn due to steering input." },
	"front_ackermann": { "type": "float", "desc": "Ackermann wheel steering angle correction." }, # optional if needed
	# Rear Axle
	"rear_steering_ratio": { "type": "float", "desc": "The ratio the wheels turn based on steering input. The higher this value, the more the wheels will turn due to steering input." },
	# Throttle and Braking
	"braking_scale": { "type": "float", "desc": "The amount of brake force needed." },
	"throttle_speed": { "type": "float", "desc": "The rate the throttle input changes to smooth input. Throttle input is between 0 and 1. Speed is in units per second." },
	"throttle_steering_adjust": { "type": "float", "desc": "Multiply the throttle speed by this based on steering input." },
	"braking_speed": { "type": "float", "desc": "The rate braking input changes to smooth input. Braking input is between 0 and 1. Speed is in units per second." },
	"brake_force_multiplier": { "type": "float", "desc": "Multiplies the automatically calculated brake force." },
	"front_brake_bias": { "type": "float", "desc": "Ratio of total brake force applied as front wheels : back wheels. If this value is below 0.0, it will be automatically calculated instead." },
	"traction_control_max_slip": { "type": "float", "desc": "Prevents engine power from causing the tires to slip beyond this value. Values below 0 disable the effect." },
	# Front Axle
	"front_abs_pulse_time": { "type": "float", "desc": "How long the ABS releases the brake, in seconds, when the spin threshold is crossed." },
	"front_abs_spin_difference_threshold": { "type": "float", "desc": "The difference in speed required between the wheel and the driving surface for ABS to engage." },
	# Rear Axle
	"rear_abs_pulse_time": { "type": "float", "desc": "How long the ABS releases the brake, in seconds, when the spin threshold is crossed." },
	"rear_abs_spin_difference_threshold": { "type": "float", "desc": "The difference in speed required between the wheel and the driving surface for ABS to engage." },
	# Stability
	"enable_stability": { "type": "bool", "desc": "Stability applies torque forces to the vehicle body when the body angle relative to the direction of travel exceeds a threshold." },
	"stability_yaw_engage_angle": { "type": "float", "desc": "The yaw angle the vehicle must reach before stability is applied. Based on the dot product, 0 being straight, 1 being 90 degrees." },
	"stability_yaw_strength": { "type": "float", "desc": "Strength multiplier for the applied yaw correction." },
	"stability_yaw_ground_multiplier": { "type": "float", "desc": "Additional strength multiplier for a grounded vehicle to overcome traction." },
	"stability_upright_spring": { "type": "float", "desc": "A multiplier for the torque used to keep the vehicle upright while airborne." },
	"stability_upright_damping": { "type": "float", "desc": "A multiplier for the torque used to dampen rotation while airborne." },
	# Gearbox
	"gear_ratios": { "type": "array", "desc": "L bozo + ratio" },
	"final_drive": { "type": "float", "desc": "Final drive ratio." },
	"reverse_ratio": { "type": "float", "desc": "Reverse gear ratio." },
	"shift_time": { "type": "float", "desc": "Time it takes to change gears on up shifts in seconds." },
	"automatic_transmission": { "type": "bool", "desc": "Enables automatic gear changes." },
	"automatic_time_between_shifts": { "type": "float", "desc": "Timer to prevent automatic gear shifts changing gears too quickly, in milliseconds." },
	"gear_inertia": { "type": "float", "desc": "Drivetrain inertia." },
	# Motor
	"max_torque": { "type": "float", "desc": "Maximum motor torque in NM." },
	"max_rpm": { "type": "float", "desc": "Maximum motor RPM." },
	"idle_rpm": { "type": "float", "desc": "Idle motor RPM." },
	"motor_drag": { "type": "float", "desc": "Variable motor drag based on RPM." },
	"motor_brake": { "type": "float", "desc": "Constant motor drag." },
	"motor_moment": { "type": "float", "desc": "Moment of inertia for the motor." },
	"clutch_out_rpm": { "type": "float", "desc": "The motor will use this rpm when launching from a stop." },
	"max_clutch_torque_ratio": { "type": "float", "desc": "Max clutch torque as a ratio of max motor torque." },
	"is_turbo": { "type": "float", "desc": "..." },
	# Drivetrain
	"front_torque_split": { "type": "float", "desc": "Torque delivered to the front wheels vs the rear wheels. 1.0 = FWD, 0.0 = RWD, in between = AWD." },
	"variable_torque_split": { "type": "bool", "desc": "When enabled, the torque split will change based on wheel slip." },
	"front_variable_split": { "type": "float", "desc": "Torque split to interpolate toward when there is wheel slip. Requires variable torque split." },
	"variable_split_speed": { "type": "float", "desc": "How quickly to interpolate between torque splits in seconds." },
	# Front Axle
	"front_locking_differential_engage_torque": { "type": "float", "desc": "Torque threshold (post-gear ratio) to lock the front differential. Negative disables." },
	"front_torque_vectoring": { "type": "float", "desc": "Torque vectoring factor for the front axle (based on steering). 1.0 = all torque to outside wheel. Only active if differential is locked." },
	# Rear Axle
	"rear_locking_differential_engage_torque": { "type": "float", "desc": "Torque threshold (post-gear ratio) to lock the rear differential. Negative disables." },
	"rear_torque_vectoring": { "type": "float", "desc": "Torque vectoring factor for the rear axle (based on steering). 1.0 = all torque to outside wheel. Only active if differential is locked." },
	# Suspension
	"vehicle_mass": { "type": "float", "desc": "Vehicle mass in kilograms." },
	"front_weight_distribution": { "type": "float", "desc": "The percentage of the vehicle mass over the front axle." },
	"center_of_gravity_height_offset": { "type": "float", "desc": "Offset for the vertical center of gravity relative to the wheel raycast positions." },
	"inertia_multiplier": { "type": "float", "desc": "Multiplier for calculated inertia. Higher values make the car harder to rotate." },
	# Front Axle
	"front_spring_length": { "type": "float", "desc": "Suspension travel in meters." },
	"front_resting_ratio": { "type": "float", "desc": "Compression ratio at rest. 0 = fully compressed." },
	"front_damping_ratio": { "type": "float", "desc": "Damping ratio. 1 = critically damped. Typical cars ~0.3, race cars up to 0.9." },
	"front_bump_damp_multiplier": { "type": "float", "desc": "Multiplier for bump damping force." },
	"front_rebound_damp_multiplier": { "type": "float", "desc": "Multiplier for rebound damping force." },
	"front_arb_ratio": { "type": "float", "desc": "Anti-roll bar stiffness as a ratio of spring stiffness." },
	"front_camber": { "type": "float", "desc": "Wheel camber angle in radians. Helps simulation stability." },
	"front_toe": { "type": "float", "desc": "Wheel toe angle in radians." },
	"front_bump_stop_multiplier": { "type": "float", "desc": "Multiplier for force when suspension is fully compressed." },
	"front_beam_axle": { "type": "bool", "desc": "If true, wheels align as if attached to a beam axle. Does not affect handling." },
	# Rear Axle
	"rear_spring_length": { "type": "float", "desc": "Suspension travel in meters. Typically longer than the front." },
	"rear_resting_ratio": { "type": "float", "desc": "Compression ratio at rest. 1 = fully compressed, 0.5 = mid travel." },
	"rear_damping_ratio": { "type": "float", "desc": "Damping ratio. 1 = critically damped. Typical cars ~0.3, race cars up to 0.9." },
	"rear_bump_damp_multiplier": { "type": "float", "desc": "Multiplier for bump damping force." },
	"rear_rebound_damp_multiplier": { "type": "float", "desc": "Multiplier for rebound damping force." },
	"rear_arb_ratio": { "type": "float", "desc": "Anti-roll bar stiffness as a ratio of spring stiffness." },
	"rear_camber": { "type": "float", "desc": "Wheel camber angle in radians. Helps simulation stability." },
	"rear_toe": { "type": "float", "desc": "Wheel toe angle in radians." },
	"rear_bump_stop_multiplier": { "type": "float", "desc": "Multiplier for force when suspension is fully compressed." },
	"rear_beam_axle": { "type": "bool", "desc": "If true, wheels align as if attached to a beam axle. Does not affect handling." },
	# Tires
	"contact_patch": { "type": "float", "desc": "Length of the tire contact patch in the brush tire model." },
	"braking_grip_multiplier": { "type": "float", "desc": "Additional longitudinal grip when braking." },
	"wheel_to_body_torque_multiplier": { "type": "float", "desc": "Multiplier for torque applied from tire force to vehicle body." },
	"tire_stiffnesses": { "type": "dict<string,float>", "desc": "Tire stiffness per surface type. Higher values increase responsiveness." },
	"coefficient_of_friction": { "type": "dict<string,float>", "desc": "Grip multiplier based on surface type." },
	"rolling_resistance": { "type": "dict<string,float>", "desc": "Rolling resistance force multiplier per surface." },
	"lateral_grip_assist": { "type": "dict<string,float>", "desc": "Grip multiplier applied with lateral slip to prevent excessive sliding." },
	"longitudinal_grip_ratio": { "type": "dict<string,float>", "desc": "Adjusts longitudinal grip relative to lateral grip for more realistic traction." },
	# Front Axle
	"front_tire_radius": { "type": "float", "desc": "Front tire radius in meters." },
	"front_tire_width": { "type": "float", "desc": "Front tire width in millimeters." },
	"front_wheel_mass": { "type": "float", "desc": "Front wheel mass in kilograms." },
	# Rear Axle
	"rear_tire_radius": { "type": "float", "desc": "Rear tire radius in meters." },
	"rear_tire_width": { "type": "float", "desc": "Rear tire width in millimeters." },
	"rear_wheel_mass": { "type": "float", "desc": "Rear wheel mass in kilograms." },
	# Aerodynamics
	"coefficient_of_drag": { "type": "float", "desc": "Drag coefficient of the vehicle body. Lower values mean less aerodynamic resistance (e.g., ~0.40 for most cars, as low as 0.05 for streamlined shapes)." },
	"air_density": { "type": "float", "desc": "Air density in kg/m³. At sea level this is about 1.225. Lower density reduces both drag and lift." },
	"frontal_area": { "type": "float", "desc": "Frontal surface area of the vehicle in square meters (m²). Rough estimates are fine; it affects drag force scaling." },
	"side_area": { "type": "float", "desc": "..." },
	"side_drag_coeff": { "type": "float", "desc": "..." },
}



var last_valid_code: String = ""
var code_line_edit: LineEdit

func _ready():
	# Build UI fields
	for var_name in editable_vars.keys():
		var hbox = HBoxContainer.new()

		var label = Label.new()
		label.text = var_name
		label.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		label.tooltip_text = editable_vars[var_name]["desc"]

		var line_edit = LineEdit.new()
		line_edit.size_flags_horizontal = Control.SIZE_EXPAND_FILL

		line_edit.text = str(vehicle_data.get(var_name))

		line_edit.set_meta("var_name", var_name)
		line_edit.connect("text_entered", func(new_text):
			_on_value_entered(new_text, line_edit)
		)
		line_edit.connect("focus_exited", func():
			_on_value_entered(line_edit.text, line_edit)
		)

		hbox.add_child(label)
		hbox.add_child(line_edit)
		vbox.add_child(hbox)

	# --- Add top-right copy/paste JSON LineEdit ---
	code_line_edit = LineEdit.new()
	code_line_edit.anchor_left = 1.0
	code_line_edit.anchor_right = 1.0
	code_line_edit.anchor_top = 0.0
	code_line_edit.anchor_bottom = 0.0
	code_line_edit.offset_left = -500
	code_line_edit.offset_right = -20
	code_line_edit.offset_top = 20
	code_line_edit.offset_bottom = 50
	code_line_edit.placeholder_text = "Copy/paste vehicle data JSON..."
	add_child(code_line_edit)

	code_line_edit.text_changed.connect(_on_code_text_changed)

	# Initialize top-right LineEdit
	last_valid_code = _encode_vehicle()
	code_line_edit.text = last_valid_code

	# --- Add Back Button ---
# --- Add Back Button ---
	var back_button = Button.new()
	back_button.text = "Back"
	# Anchor to bottom-left
	back_button.anchor_left = 1.0
	back_button.anchor_right = 0.0
	back_button.anchor_top = 0.0
	back_button.anchor_bottom = 1.0
	back_button.pressed.connect(Callable(self, "_on_back_button_pressed"))
	add_child(back_button)




func _process(delta: float) -> void:
	# Live update top-right LineEdit if vehicle_data changed elsewhere
	if not code_line_edit.has_focus():
		var current_code = _encode_vehicle()
		if current_code != last_valid_code:
			last_valid_code = current_code
			code_line_edit.text = last_valid_code


# -----------------------
# Editable fields helpers
# -----------------------
func _on_value_entered(new_text: String, line_edit: LineEdit) -> void:
	var var_name = line_edit.get_meta("var_name")
	var var_type = editable_vars[var_name]["type"]
	var as_degrees = line_edit.get_meta("as_degrees") or false

	var new_value
	var valid = true

	match var_type:
		"float":
			valid = new_text.is_valid_float()
			if valid:
				new_value = float(new_text)
				if as_degrees:
					new_value = deg_to_rad(new_value)
		"int":
			valid = new_text.is_valid_int()
			if valid:
				new_value = int(new_text)
		"bool":
			new_value = new_text.to_lower() in ["true", "1", "yes"]
		"array", "dict":
			var expr = Expression.new()
			var err = expr.parse(new_text, [])
			if err == OK:
				var result_array = expr.execute([])
				var result = result_array[0]
				var exec_err = result_array[1]
				if exec_err == OK and typeof(result) == (TYPE_ARRAY if var_type == "array" else TYPE_DICTIONARY):
					new_value = result
				else:
					valid = false
			else:
				valid = false
		_:
			new_value = new_text

	if valid:
		vehicle_data.set(var_name, new_value)

	# Refresh displayed value
	var display_value = vehicle_data.get(var_name)
	if var_type == "float" and as_degrees:
		display_value = rad_to_deg(display_value)
	elif var_type == "array" or var_type == "dict":
		display_value = str(display_value)
	line_edit.text = str(display_value)


func _update_ui_from_data():
	for child in vbox.get_children():
		if child is HBoxContainer:
			for subchild in child.get_children():
				if subchild is LineEdit:
					var var_name = subchild.get_meta("var_name")
					subchild.text = str(vehicle_data.get(var_name))


func _on_back_button_pressed() -> void:
	get_tree().change_scene_to_file("res://UI/main_menu.tscn")


# -----------------------
# Top-right LineEdit helpers (JSON)
# -----------------------
func _encode_vehicle() -> String:
	var copy := {}
	for key in editable_vars.keys():
		copy[key] = vehicle_data.get(key)
	return JSON.stringify(copy, "\t") # pretty print


func _decode_vehicle(code: String) -> bool:
	var result = JSON.parse_string(code)
	if typeof(result) != TYPE_DICTIONARY:
		return false

	for key in result.keys():
		if editable_vars.has(key):
			vehicle_data.set(key, result[key])
	return true


func _on_code_text_changed(new_text: String) -> void:
	if _decode_vehicle(new_text):
		last_valid_code = _encode_vehicle()
	else:
		# Invalid JSON → revert
		code_line_edit.text = last_valid_code
