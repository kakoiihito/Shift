class_name PlayerInput
extends Resource

var throttle = 0.0
var brake = 0.0
var steering = 0.0
var clutch = 0.0
var shift_up = false   
var shift_down = false 

## Avoids colliding with the user's own Input Map actions.
const PREFIX := "vehicle_"

@export var keyboard_input: Dictionary = {
	"Gas": "W",
	"Brake": "S",
	"SteerLeft": "A",
	"SteerRight": "D",
	"Clutch": "Shift",
	"ShiftUp": "E",
	"ShiftDown": "Q",
}

@export var joy_buttons: Dictionary = {
	"Clutch": JOY_BUTTON_X,
	"ShiftUp": JOY_BUTTON_RIGHT_SHOULDER,
	"ShiftDown": JOY_BUTTON_LEFT_SHOULDER,
}
@export var joy_axes: Dictionary = {
	"Gas": [JOY_AXIS_TRIGGER_RIGHT, 1.0],
	"Brake": [JOY_AXIS_TRIGGER_LEFT, 1.0],
	"SteerLeft": [JOY_AXIS_LEFT_X, -1.0],
	"SteerRight": [JOY_AXIS_LEFT_X, 1.0],
}

@export_range(0.0, 0.9, 0.01) var deadzone := 0.15

func set_input() -> void:

	for d in [keyboard_input, joy_buttons, joy_axes]:
		for action_name in d:
			_clear_action(action_name)

	for action_name in keyboard_input:
		var key_name: String = keyboard_input[action_name]
		var keycode = OS.find_keycode_from_string(key_name)
		if keycode == KEY_NONE:
			print("PlayerInput: unknown key '%s' for action '%s'" % [key_name, action_name])
		else:
			var ev = InputEventKey.new()
			ev.physical_keycode = keycode
			_add_event(action_name, ev)

	for action_name in joy_buttons:
		var ev = InputEventJoypadButton.new()
		ev.button_index = joy_buttons[action_name]
		_add_event(action_name, ev)

	for action_name in joy_axes:
		var ev = InputEventJoypadMotion.new()
		ev.axis = joy_axes[action_name][0]
		ev.axis_value = joy_axes[action_name][1]
		_add_event(action_name, ev)


func poll() -> void:
	throttle = Input.get_action_strength(PREFIX + "Gas")
	brake = Input.get_action_strength(PREFIX + "Brake")
	steering = Input.get_axis(PREFIX + "SteerRight", PREFIX + "SteerLeft")
	clutch = Input.get_action_strength(PREFIX + "Clutch")
	shift_up = Input.is_action_just_pressed(PREFIX + "ShiftUp")
	shift_down = Input.is_action_just_pressed(PREFIX + "ShiftDown")


func _clear_action(action_name: String) -> void:
	var action = PREFIX + action_name
	if InputMap.has_action(action):
		InputMap.action_erase_events(action)
	else:
		InputMap.add_action(action, deadzone)

func _add_event(action_name: String, ev: InputEvent) -> void:
	var action = PREFIX + action_name
	if not InputMap.has_action(action):
		InputMap.add_action(action, deadzone)
	InputMap.action_add_event(action, ev)
