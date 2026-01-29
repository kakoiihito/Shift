# Portions are Copyright (c) 2021 Dechode
# https://github.com/Dechode/Godot-Advanced-Vehicle

#class_name Vehicle
extends RigidBody3D

@export_group("Wheel Nodes")
## Assign this to the Wheel [RayCast3D] that is this vehicle's front left wheel.
@export var front_left_wheel : Wheel
## Assign this to the Wheel [RayCast3D] that is this vehicle's front right wheel.
@export var front_right_wheel : Wheel
## Assign this to the Wheel [RayCast3D] that is this vehicle's rear left wheel.
@export var rear_left_wheel : Wheel
## Assign this to the Wheel [RayCast3D] that is this vehicle's rear right wheel.
@export var rear_right_wheel : Wheel

@export_group("Steering")
## The rate that the steering input changes in order to smooth
## out direction changes to the wheel.
## Steering input is between -1 and 1. Speed is in units per second.
@export var steering_speed := 7.5
## The rate that the steering input changes when steering back to center.
## Speed is in units per second.
@export var countersteer_speed := 20.0
## Reduces steering input based on the vehicle's speed.
## Steering speed is divided by the velocity at this magnitude.
## The larger the number, the slower the steering at speed.
@export var steering_speed_decay := 0.30
## Further steering input is prevented if the wheels' lateral slip is greater than this number.
@export var steering_slip_assist := 0.12
## The magnitude to adjust steering toward the direction of travel based on the vehicle's lateral velocity.
@export var countersteer_assist := 1.1
## Steering input is raised to the power of this number.
## This has the effect of slowing steering input near the limits.
@export var steering_exponent := 1.8
## The maximum steering angle in radians.
## [br][br]
## [b]Note:[/b] This property is edited in the inspector in degrees. If you want to use degrees in a script, use [code]deg_to_rad[/code].
@export_range(0, 360, 0.1, "radians_as_degrees") var max_steering_angle := deg_to_rad(28.0)

@export_subgroup("Front Axle", "front_")
## The ratio that the wheels turn based on steering input.
## [br]The higher this value, the more the wheels will turn due to steering input.
@export var front_steering_ratio := 0.85
## Ackermann wheel steering angle correction
@export var front_ackermann := 0.18

@export_subgroup("Rear Axle", "rear_")
## The ratio the wheels turn based on steering input.
## [br]The higher this value, the more the wheels will turn due to steering input.
@export var rear_steering_ratio := 0.05


@export_group("Throttle and Braking")
@export var braking_scale: float = 0.0001
## The rate the throttle input changes to smooth input.
## Throttle input is between 0 and 1. Speed is in units per second.
@export var throttle_speed := 25.0
## Multiply the throttle speed by this based on steering input.
@export var throttle_steering_adjust := 0.12
## The rate braking input changes to smooth input.
## Braking input is between 0 and 1. Speed is in units per second.
@export var braking_speed := 18.0
## Multiplies the automatically calculated brake force.
@export var brake_force_multiplier := 1.3
## Ratio of total brake force applied as front wheels : back wheels. If this value is
## below 0.0, this value will be automatically calculated instead.
@export var front_brake_bias := 0.60
## Prevents engine power from causing the tires to slip beyond this value.
## Values below 0 disable the effect.
@export var traction_control_max_slip := 4.5

@export_subgroup("Front Axle", "front_")
## How long the ABS releases the brake, in seconds, when the
## spin threshold is crossed.
@export var front_abs_pulse_time := 0.03
## The difference in speed required between the wheel and the
## driving surface for ABS to engage.
@export var front_abs_spin_difference_threshold := 6.5

@export_subgroup("Rear Axle", "rear_")
## How long the ABS releases the brake, in seconds, when the
## spin threshold is crossed.
@export var rear_abs_pulse_time := 0.03
## The difference in speed required between the wheel and the
## driving surface for ABS to engage.
@export var rear_abs_spin_difference_threshold := 6.5

@export_group("Stability")
## Stability applies torque forces to the vehicle body when the body angle
## relative to the direction of travel exceeds a threshold.
@export var enable_stability := true
## The yaw angle the vehicle must reach before stability is applied.
## Based on the dot product, 0 being straight, 1 being 90 degrees
@export var stability_yaw_engage_angle := 0.25
## Strength multiplier for the applied yaw correction.
@export var stability_yaw_strength := 10.0
## Additional strength multiplier for a grounded vehicle to overcome traction.
@export var stability_yaw_ground_multiplier := 2.0
## A multiplier for the torque used to keep the vehicle upright while airborne.
@export var stability_upright_spring := 2.5
## A multiplier for the torque used to dampen rotation while airborne.
@export var stability_upright_damping := 1200.0


@export_group("Motor")
## Maximum motor torque in NM.
@export var max_torque := 520.0
## Maximum motor RPM.
@export var max_rpm := 8500.0
## Idle motor RPM.
@export var idle_rpm := 900.0
## Percentage of torque produced across the RPM range.
@export var torque_curve : Curve
## Variable motor drag based on RPM.
@export var motor_drag := 0.008
## Constant motor drag.
@export var motor_brake := 7.5
## Moment of inertia for the motor.
@export var motor_moment := 0.45
## The motor will use this rpm when launching from a stop.
@export var clutch_out_rpm := 3200.0
## Max clutch torque as a ratio of max motor torque.
@export var max_clutch_torque_ratio := 1.6

@export var is_turbo := false


@export_group("Gearbox")
## Transmission gear ratios, the size of the array determines the number of gears
@export var gear_ratios : Array[float] = [ 3.6, 2.3, 1.7, 1.3, 1.05, 0.9 ] 
## Final drive ratio
@export var final_drive := 3.7
## Reverse gear ratio
@export var reverse_ratio := 3.5
## Time it takes to change gears on up shifts in seconds
@export var shift_time := 0.12
## Enables automatic gear changes
@export var automatic_transmission := true
## Timer to prevent the automatic gear shifts changing gears too quickly 
## in milliseconds
@export var automatic_time_between_shifts := 500.0
## Drivetrain inertia
@export var gear_inertia := 0.012


@export_group("Drivetrain")
## Torque delivered to the front wheels vs the rear wheels.
## Value of 1 is FWD, a value of 0 is RWD, anything in between is AWD.
@export var front_torque_split := 0.25
## When enabled, the torque split will change based on wheel slip.
@export var variable_torque_split := true
## Torque split to interpolate toward when there is wheel slip. Variable Torque
## Split must be enabled.
@export var front_variable_split := 0.35
## How quickly to interpolate between torque splits in seconds.
@export var variable_split_speed := 1.2

@export_subgroup("Front Axle", "front_")
## The wheels of the axle will be forced to spin the same speed if there
## is at least this much torque applied. Keeps vehicle from spinning one wheel.
## Torque is measured after multiplied by the current gear ratio.
## Negative values will disable.
@export var front_locking_differential_engage_torque := 160.0
## The amount of torque vectoring to apply to the axle based on steering input.
## Only functions if the differential is locked.
## A value of 1.0 would apply all torque to the outside wheel.
@export var front_torque_vectoring := 0.25

@export_subgroup("Rear Axle", "rear_")
## The wheels of the axle will be forced to spin the same speed if there
## is at least this much torque applied. Keeps vehicle from spinning one wheel.
## Torque is measured after multiplied by the current gear ratio.
## Negative values will disable.
@export var rear_locking_differential_engage_torque := 170.0
## The amount of torque vectoring to apply to the axle based on steering input.
## Only functions if the differential is locked.
## A value of 1.0 would apply all torque to the outside wheel.
@export var rear_torque_vectoring := 0.35


@export_group("Suspension")
## Vehicle mass in kilograms.
@export var vehicle_mass := 1450.0
## The percentage of the vehicle mass over the front axle.
@export var front_weight_distribution := 0.50
## The center of gravity is calculated from the front weight distribution
## with the height centered on the wheel raycast positions. This will offset
## the height from that calculated position.
@export var center_of_gravity_height_offset := -0.25
## Multiplies the calculated inertia by this value.
## Greater inertia values will cause more force to be
## required to rotate the car.
@export var inertia_multiplier := 1.2

@export_subgroup("Front Axle", "front_")
## The amount of suspension travel in meters.
@export var front_spring_length := 0.12
## How much the spring is compressed when the vehicle is at rest.
## This is used to calculate the approriate spring rate for the wheel.
## A value of 0 would be a fully compressed spring.
@export var front_resting_ratio := 0.38
## Damping ratio is used to calculate the damping forces on the spring.
## A value of 1 would be critically damped. Passenger cars typically have a
## ratio around 0.3, while a race car could be as high as 0.9.
@export var front_damping_ratio := 0.65
## Bump damping multiplier applied to the damping force calculated from the
## damping ratio. A typical ratio for a passenger car is 2/3 bump damping to
## 3/2 rebound damping. Race cars typically run 3/2 bump to 2/3 rebound.
@export var front_bump_damp_multiplier := 0.7
## Rebound damping multiplier applied to the damping force calculated from the
## damping ratio. A typical ratio for a passenger car is 2/3 bump damping to
## 3/2 rebound damping. Race cars typically run 3/2 bump to 2/3 rebound.
@export var front_rebound_damp_multiplier := 1.3
## Antiroll bar stiffness as a ratio to spring stiffness.
@export var front_arb_ratio := 0.40
## Wheel camber isn't simulated, but giving the raycast a slight angle helps
## with simulation stability. Measured in radians.
@export var front_camber := 0.04
## Toe of the tires measured in radians.
@export var front_toe := 0.003
## Multiplier for the force applied when the suspension is fully compressed.
## If the vehicle bounces off large bumps, reducing this will help.
@export var front_bump_stop_multiplier := 1.25
## If true the wheels of this axle will be aligned as if they were attached to
## a beam axle. This setting does not affect vehicle handling.
@export var front_beam_axle := false

@export_subgroup("Rear Axle", "rear_")
## The amount of suspension travel in meters. Rear suspension typically has
## more travel than the front.
@export var rear_spring_length := 0.14
## How much the spring is compressed when the vehicle is at rest.
## This is used to calculate the approriate spring rate for the wheel.
## A value of 1 would be a fully compressed spring. With a value of 0.5 the
## suspension will rest at the center of it's length.
@export var rear_resting_ratio := 0.37
## Damping ratio is used to calculate the damping forces on the spring.
## A value of 1 would be critically damped. Passenger cars typically have a
## ratio around 0.3, while a race car could be as high as 0.9.
@export var rear_damping_ratio := 0.63
## Bump damping multiplier applied to the damping force calulated from the
## damping ratio. A typical ratio for a passenger car is 2/3 bump damping to
## 3/2 rebound damping. Race cars typically run 3/2 bump to 2/3 rebound.
@export var rear_bump_damp_multiplier := 0.68
## Rebound damping multiplier applied to the damping force calulated from the
## damping ratio. A typical ratio for a passenger car is 2/3 bump damping to
## 3/2 rebound damping. Race cars typically run 3/2 bump to 2/3 rebound.
@export var rear_rebound_damp_multiplier := 1.32
## Antiroll bar stiffness as a ratio to spring stiffness.
@export var rear_arb_ratio := 0.38
## Wheel camber isn't simulated, but giving the raycast a slight angle helps
## with simulation stability.
@export var rear_camber := 0.035
## Toe of the tires measured in radians.
@export var rear_toe := 0.003
## Multiplier for the force applied when the suspension is fully compressed.
## If the vehicle bounces off large bumps, reducing this will help.
@export var rear_bump_stop_multiplier := 1.22
## If true the wheels of this axle will be aligned as if they were attached to
## a beam axle. This setting does not affect vehicle handling.
@export var rear_beam_axle := false

@export_group("Tires")
## Represents the length of the tire contact patch in the brush tire model.
@export var contact_patch := 0.20
## Provides additional longitudinal grip when braking.
@export var braking_grip_multiplier := 1.6
## Tire force applied to the ground is also applied to the vehicle body as a
## torque centered on the wheel. 
@export var wheel_to_body_torque_multiplier := 1.25
## Represents tire stiffness in the brush tire model. Higher values increase
## the responsiveness of the tire.
## Surface detection uses node groups to identify the surface, so make sure
## your staticbodies and rigidbodies belong to one of these groups.
@export var tire_stiffnesses := { "Road" : 16.0, "Dirt" : 1.8, "Grass" : 1.3 }
## A multiplier for the amount of force a tire can apply based on the surface.
## Surface detection uses node groups to identify the surface, so make sure
## your staticbodies and rigidbodies belong to one of these groups.
@export var coefficient_of_friction := { "Road" : 3.5, "Dirt" : 2.0, "Grass" : 1.6 }
## A multiplier for the amount of rolling resistance force based on the surface.
## Surface detection uses node groups to identify the surface, so make sure
## your staticbodies and rigidbodies belong to one of these groups.
@export var rolling_resistance := { "Road" : 1.1, "Dirt" : 2.3, "Grass" : 3.2 }
## A multiplier to provide more grip based on the amount of lateral wheel slip.
## This can be used to keep vehicles from sliding a long distance, but may provide
## unrealistically high amounts of grip.
## Surface detection uses node groups to identify the surface, so make sure
## your staticbodies and rigidbodies belong to one of these groups.
@export var lateral_grip_assist := { "Road" : 0.07, "Dirt" : 0.025, "Grass" : 0.015 }
## A multiplier to adjust longitudinal grip to differ from lateral grip.
## Useful for allowing vehicles to have wheel spin and maintain high lateral grip.
## Surface detection uses node groups to identify the surface, so make sure
## your staticbodies and rigidbodies belong to one of these groups.
@export var longitudinal_grip_ratio := { "Road" : 0.63, "Dirt": 0.53, "Grass" : 0.48 }

@export_subgroup("Front Axle", "front_")
## Tire radius in meters
@export var front_tire_radius := 0.34
## Tire width in millimeters. The width doesn't directly affect tire friction,
## but reduces the effects of tire load sensitivity.
@export var front_tire_width := 250.0
## Wheel mass in kilograms.
@export var front_wheel_mass := 12.5

@export_subgroup("Rear Axle", "rear_")
## Tire radius in meters
@export var rear_tire_radius := 0.34
## Tire width in millimeters. The width doesn't directly affect tire friction,
## but reduces the effects of tire load sensitivity.
@export var rear_tire_width := 260.0
## Wheel mass in kilograms.
@export var rear_wheel_mass := 12.8

@export_group("Aerodynamics")
## The drag coefficient quantifies how much [b]drag[/b] (force against thrust)
## the vehicle receives when moving through air. In the drag equation,
## a lower drag coefficient means the vehicle will experience less drag
## force, allowing it to move faster.
## [br]Typically, the drag coefficient is assumed from the shape of the
## body, where more teardrop-shaped bodies experience a lower drag coefficient.
## Un-streamlined cylindrical bodies have a drag coefficient of
## around [code]0.80[/code], while more streamlined teardrop-shaped bodies 
## can have a drag coefficient as low as [code]0.05[/code], or even lower.
## [br]As a more relevant example, most cars have drag coefficients
## around [code]0.40[/code].
@export var coefficient_of_drag := 0.33
## From [url=https://www.grc.nasa.gov/www/k-12/VirtualAero/BottleRocket/airplane/density.html#:~:text=Halving%20the%20density%20halves%20the,above%20which%20it%20cannot%20fly.]NASA[/url]:
## [i]"Halving the density halves the lift, halving the density halves the drag. The [lb]air[rb] density depends on the type of [lb]air[rb] and the depth of the [lb]air[rb]. In the atmosphere, air density decreases as altitude increases. This explains why airplanes have a flight ceiling, an altitude above which it cannot fly."[/i]
@export var air_density := 1.225
## The amount of surface area the front-facing part of the vehicle has,
## in meters squared ([code]m^2[/code]).
## [br][br]
## [b]Note:[/b] You do not have to calculate this value to be exact,
## a rough estimate - or even something completely different, depending
## on the result you want - will do.
@export var frontal_area := 2.0
@export var side_area: float = 3.5              # m² (rough guess)
@export var side_drag_coeff: float = 1.1  
