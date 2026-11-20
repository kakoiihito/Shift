## Manual gearbox with a clutch-gated shift delay.
##
## This script handles gear selection for a manual transmission. The player can
## only change gear while the clutch pedal is (almost) fully pressed. Each shift
## then takes a short, slightly random amount of time, during which the gear
## ratio is zero so no torque passes through the gearbox. It keeps no state of
## its own: the current gear, ratio and shift state live in the transmission
## runtime data, and the gear ratios come from the car tuning resource.
##
## [b]Gear numbering[/b] matches the [code]gear_ratio[/code] array in the car
## tuning resource: index 0 is reverse, index 1 is neutral (ratio 0.0), and
## indices 2 and up are the forward gears (first gear is index 2).
##
## [b]Input actions used:[/b] [code]"Clutch"[/code] (pedal, 0..1, where 1 is
## fully pressed), [code]"ShiftUp"[/code] and [code]"ShiftDown"[/code].
##
## [b]Runtime data fields used:[/b] [code]current_gear[/code],
## [code]current_gear_ratio[/code], [code]is_shifting[/code] and
## [code]shift_timer[/code]. Other scripts, such as the clutch and drivetrain
## calculation, read these to know whether the car is in gear.
extends Node


## Runs one step of the transmission simulation.
##
## Call this once per physics frame. It first checks the shift inputs and starts
## a shift if one is requested, then advances any shift that is in progress.
##
## [param delta] is the time step in seconds.[br]
## [param Data] is the runtime transmission state (read and written).[br]
## [param Values] is the car tuning resource that holds the
## [code]gear_ratio[/code] array.
func transmission_process(delta: float, Data: RuntimeData.transmission, Values: Resource):
	
	gear_change(Data, Values)
	gear_ratio_change(Data, delta, Values)
	

## Handles shift up and shift down requests.
##
## A request is only accepted when both of these are true:[br]
## - no shift is already in progress ([code]is_shifting[/code] is false),[br]
## - the clutch pedal is pressed more than 95 percent.
##
## When [code]"ShiftUp"[/code] is pressed and the current gear is below the
## highest gear (the last index of [code]gear_ratio[/code]), the gear goes up by
## one. When [code]"ShiftDown"[/code] is pressed and the current gear is above 0
## (reverse), the gear goes down by one. Because gears are consecutive indices,
## shifting passes through neutral between reverse and first gear.
##
## An accepted shift sets [code]is_shifting[/code] to true and gives
## [code]shift_timer[/code] a random duration between 0.15 and 0.25 seconds,
## which adds some natural variation to the shift time.
##
## The gear number changes immediately, but the gear ratio is only applied when
## the shift finishes (see [method gear_ratio_change]).
func gear_change(Data: RuntimeData.transmission, Values: Resource):
	
	var target_clutch = Input.get_action_strength("Clutch")
	if not Data.is_shifting and target_clutch > 0.95:
		if Input.is_action_just_pressed("ShiftUp") and Data.current_gear < Values.gear_ratio.size() - 1:
			Data.current_gear += 1
			Data.is_shifting = true
			Data.shift_timer = 0.15 + randf() * 0.1
		elif Input.is_action_just_pressed("ShiftDown") and Data.current_gear > 0:
			Data.current_gear -= 1
			Data.is_shifting = true
			Data.shift_timer = 0.15 + randf() * 0.1


## Updates the active gear ratio and finishes shifts that are in progress.
##
## While a shift is in progress ([code]is_shifting[/code] is true) the active
## ratio [code]current_gear_ratio[/code] is held at 0.0, so the gearbox
## transmits no torque, and [code]shift_timer[/code] counts down by
## [param delta]. When the timer reaches zero, the ratio of the newly selected
## gear is read from [code]Values.gear_ratio[/code] and [code]is_shifting[/code]
## is set back to false.
##
## Nothing happens when no shift is in progress, so [code]current_gear_ratio[/code]
## keeps its last value. This means its starting value (before the first shift)
## must be set elsewhere, for example in the runtime data defaults.
##
## [param delta] is the time step in seconds.
func gear_ratio_change(Data: RuntimeData.transmission, delta: float, Values: Resource):

	if Data.is_shifting:
		Data.current_gear_ratio = 0.0
		Data.shift_timer -= delta
		if Data.shift_timer <= 0.0:
			Data.current_gear_ratio = Values.gear_ratio[Data.current_gear]
			Data.is_shifting = false
