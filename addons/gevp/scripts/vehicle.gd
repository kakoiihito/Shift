# Portions are Copyright (c) 2021 Dechode
# https://github.com/Dechode/Godot-Advanced-Vehicle

class_name Vehicle
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
@export var steering_speed := Vehicledata.steering_speed
## The rate that the steering input changes when steering back to center.
## Speed is in units per second.
@export var countersteer_speed := Vehicledata.countersteer_speed
## Reduces steering input based on the vehicle's speed.
## Steering speed is divided by the velocity at this magnitude.
## The larger the number, the slower the steering at speed.
@export var steering_speed_decay := Vehicledata.steering_speed_decay
## Further steering input is prevented if the wheels' lateral slip is greater than this number.
@export var steering_slip_assist := Vehicledata.steering_slip_assist
## The magnitude to adjust steering toward the direction of travel based on the vehicle's lateral velocity.
@export var countersteer_assist := Vehicledata.countersteer_assist
## Steering input is raised to the power of this number.
## This has the effect of slowing steering input near the limits.
@export var steering_exponent := Vehicledata.steering_exponent
## The maximum steering angle in radians.
## [br][br]
## [b]Note:[/b] This property is edited in the inspector in degrees. If you want to use degrees in a script, use [code]deg_to_rad[/code].
@export_range(0, 360, 0.1, "radians_as_degrees") var max_steering_angle := deg_to_rad(Vehicledata.max_steering_angle)

@export_subgroup("Front Axle", "front_")
## The ratio that the wheels turn based on steering input.
## [br]The higher this value, the more the wheels will turn due to steering input.
@export var front_steering_ratio := Vehicledata.front_steering_ratio
## Ackermann wheel steering angle correction
#@export var front_ackermann := Vehicledata.front_ackermann
@export_subgroup("Rear Axle", "rear_")
## The ratio the wheels turn based on steering input.
## [br]The higher this value, the more the wheels will turn due to steering input.
@export var rear_steering_ratio := Vehicledata.rear_steering_ratio

@export_group("Throttle and Braking")
@export var braking_scale: float = Vehicledata.braking_scale
## The rate the throttle input changes to smooth input.
## Throttle input is between 0 and 1. Speed is in units per second.
@export var throttle_speed := Vehicledata.throttle_speed
## Multiply the throttle speed by this based on steering input.
@export var throttle_steering_adjust := Vehicledata.throttle_steering_adjust
## The rate braking input changes to smooth input.
## Braking input is between 0 and 1. Speed is in units per second.
@export var braking_speed := Vehicledata.braking_speed
## Multiplies the automatically calculated brake force.
@export var brake_force_multiplier := Vehicledata.brake_force_multiplier
## Ratio of total brake force applied as front wheels : back wheels. If this value is
## below 0.0, this value will be automatically calculated instead.
@export var front_brake_bias := Vehicledata.front_brake_bias
## Prevents engine power from causing the tires to slip beyond this value.
## Values below 0 disable the effect.
@export var traction_control_max_slip := Vehicledata.traction_control_max_slip

@export_subgroup("Front Axle", "front_")
## How long the ABS releases the brake, in seconds, when the
## spin threshold is crossed.
@export var front_abs_pulse_time := Vehicledata.front_abs_pulse_time
## The difference in speed required between the wheel and the
## driving surface for ABS to engage.
@export var front_abs_spin_difference_threshold := Vehicledata.front_abs_spin_difference_threshold

@export_subgroup("Rear Axle", "rear_")
## How long the ABS releases the brake, in seconds, when the
## spin threshold is crossed.
@export var rear_abs_pulse_time := Vehicledata.rear_abs_pulse_time
## The difference in speed required between the wheel and the
## driving surface for ABS to engage.
@export var rear_abs_spin_difference_threshold := Vehicledata.rear_abs_spin_difference_threshold

@export_group("Stability")
## Stability applies torque forces to the vehicle body when the body angle
## relative to the direction of travel exceeds a threshold.
@export var enable_stability := Vehicledata.enable_stability
## The yaw angle the vehicle must reach before stability is applied.
## Based on the dot product, 0 being straight, 1 being 90 degrees
@export var stability_yaw_engage_angle := Vehicledata.stability_yaw_engage_angle
## Strength multiplier for the applied yaw correction.
@export var stability_yaw_strength := Vehicledata.stability_yaw_strength
## Additional strength multiplier for a grounded vehicle to overcome traction.
@export var stability_yaw_ground_multiplier := Vehicledata.stability_yaw_ground_multiplier
## A multiplier for the torque used to keep the vehicle upright while airborne.
@export var stability_upright_spring := Vehicledata.stability_upright_spring
## A multiplier for the torque used to dampen rotation while airborne.
@export var stability_upright_damping := Vehicledata.stability_upright_damping

@export_group("Motor")
## Maximum motor torque in NM.
@export var max_torque := Vehicledata.max_torque
## Maximum motor RPM.
@export var max_rpm := Vehicledata.max_rpm
## Idle motor RPM.
@export var idle_rpm := Vehicledata.idle_rpm
## Percentage of torque produced across the RPM range.
@export var torque_curve : Curve
## Variable motor drag based on RPM.
@export var motor_drag := Vehicledata.motor_drag
## Constant motor drag.
@export var motor_brake := Vehicledata.motor_brake
## Moment of inertia for the motor.
@export var motor_moment := Vehicledata.motor_moment
## The motor will use this rpm when launching from a stop.
@export var clutch_out_rpm := Vehicledata.clutch_out_rpm
## Max clutch torque as a ratio of max motor torque.
@export var max_clutch_torque_ratio := Vehicledata.max_clutch_torque_ratio

@export var is_turbo : bool = Vehicledata.is_turbo

@export_group("Gearbox")
## Transmission gear ratios, the size of the array determines the number of gears
@export var gear_ratios : Array[float] = Vehicledata.gear_ratios
## Final drive ratio
@export var final_drive := Vehicledata.final_drive
## Reverse gear ratio
@export var reverse_ratio := Vehicledata.reverse_ratio
## Time it takes to change gears on up shifts in seconds
@export var shift_time := Vehicledata.shift_time
## Enables automatic gear changes
@export var automatic_transmission := Vehicledata.automatic_transmission
## Timer to prevent the automatic gear shifts changing gears too quickly 
## in milliseconds
@export var automatic_time_between_shifts := Vehicledata.automatic_time_between_shifts
## Drivetrain inertia
@export var gear_inertia := Vehicledata.gear_inertia

@export_group("Drivetrain")
## Torque delivered to the front wheels vs the rear wheels.
## Value of 1 is FWD, a value of 0 is RWD, anything in between is AWD.
@export var front_torque_split := Vehicledata.front_torque_split
## When enabled, the torque split will change based on wheel slip.
@export var variable_torque_split := Vehicledata.variable_torque_split
## Torque split to interpolate toward when there is wheel slip. Variable Torque
## Split must be enabled.
@export var front_variable_split := Vehicledata.front_variable_split
## How quickly to interpolate between torque splits in seconds.
@export var variable_split_speed := Vehicledata.variable_split_speed
@export_subgroup("Front Axle", "front_")
## The wheels of the axle will be forced to spin the same speed if there
## is at least this much torque applied. Keeps vehicle from spinning one wheel.
## Torque is measured after multiplied by the current gear ratio.
## Negative values will disable.
@export var front_locking_differential_engage_torque := Vehicledata.front_locking_differential_engage_torque
## The amount of torque vectoring to apply to the axle based on steering input.
## Only functions if the differential is locked.
## A value of 1.0 would apply all torque to the outside wheel.
@export var front_torque_vectoring := Vehicledata.front_torque_vectoring
@export_subgroup("Rear Axle", "rear_")
## The wheels of the axle will be forced to spin the same speed if there
## is at least this much torque applied. Keeps vehicle from spinning one wheel.
## Torque is measured after multiplied by the current gear ratio.
## Negative values will disable.
@export var rear_locking_differential_engage_torque := Vehicledata.rear_locking_differential_engage_torque
## The amount of torque vectoring to apply to the axle based on steering input.
## Only functions if the differential is locked.
## A value of 1.0 would apply all torque to the outside wheel.
@export var rear_torque_vectoring := Vehicledata.rear_torque_vectoring

@export_group("Suspension")
## Vehicle mass in kilograms.
@export var vehicle_mass := Vehicledata.vehicle_mass
## The percentage of the vehicle mass over the front axle.
@export var front_weight_distribution := Vehicledata.front_weight_distribution
## The center of gravity is calculated from the front weight distribution
## with the height centered on the wheel raycast positions. This will offset
## the height from that calculated position.
@export var center_of_gravity_height_offset := Vehicledata.center_of_gravity_height_offset
## Multiplies the calculated inertia by this value.
## Greater inertia values will cause more force to be
## required to rotate the car.
@export var inertia_multiplier := Vehicledata.inertia_multiplier

@export_subgroup("Front Axle", "front_")
## The amount of suspension travel in meters.
@export var front_spring_length := Vehicledata.front_spring_length
## How much the spring is compressed when the vehicle is at rest.
## This is used to calculate the approriate spring rate for the wheel.
## A value of 0 would be a fully compressed spring.
@export var front_resting_ratio := Vehicledata.front_resting_ratio
## Damping ratio is used to calculate the damping forces on the spring.
## A value of 1 would be critically damped. Passenger cars typically have a
## ratio around 0.3, while a race car could be as high as 0.9.
@export var front_damping_ratio := Vehicledata.front_damping_ratio
## Bump damping multiplier applied to the damping force calculated from the
## damping ratio. A typical ratio for a passenger car is 2/3 bump damping to
## 3/2 rebound damping. Race cars typically run 3/2 bump to 2/3 rebound.
@export var front_bump_damp_multiplier := Vehicledata.front_bump_damp_multiplier
## Rebound damping multiplier applied to the damping force calculated from the
## damping ratio. A typical ratio for a passenger car is 2/3 bump damping to
## 3/2 rebound damping. Race cars typically run 3/2 bump to 2/3 rebound.
@export var front_rebound_damp_multiplier := Vehicledata.front_rebound_damp_multiplier
## Antiroll bar stiffness as a ratio to spring stiffness.
@export var front_arb_ratio := Vehicledata.front_arb_ratio
## Wheel camber isn't simulated, but giving the raycast a slight angle helps
## with simulation stability. Measured in radians.
@export var front_camber := Vehicledata.front_camber
## Toe of the tires measured in radians.
@export var front_toe := Vehicledata.front_toe
## Multiplier for the force applied when the suspension is fully compressed.
## If the vehicle bounces off large bumps, reducing this will help.
@export var front_bump_stop_multiplier := Vehicledata.front_bump_stop_multiplier
## If true the wheels of this axle will be aligned as if they were attached to
## a beam axle. This setting does not affect vehicle handling.
@export var front_beam_axle := Vehicledata.front_beam_axle

@export_subgroup("Rear Axle", "rear_")
## The amount of suspension travel in meters. Rear suspension typically has
## more travel than the front.
@export var rear_spring_length := Vehicledata.rear_spring_length
## How much the spring is compressed when the vehicle is at rest.
## This is used to calculate the approriate spring rate for the wheel.
## A value of 1 would be a fully compressed spring. With a value of 0.5 the
## suspension will rest at the center of it's length.
@export var rear_resting_ratio := Vehicledata.rear_resting_ratio
## Damping ratio is used to calculate the damping forces on the spring.
## A value of 1 would be critically damped. Passenger cars typically have a
## ratio around 0.3, while a race car could be as high as 0.9.
@export var rear_damping_ratio := Vehicledata.rear_damping_ratio
## Bump damping multiplier applied to the damping force calulated from the
## damping ratio. A typical ratio for a passenger car is 2/3 bump damping to
## 3/2 rebound damping. Race cars typically run 3/2 bump to 2/3 rebound.
@export var rear_bump_damp_multiplier := Vehicledata.rear_bump_damp_multiplier
## Rebound damping multiplier applied to the damping force calulated from the
## damping ratio. A typical ratio for a passenger car is 2/3 bump damping to
## 3/2 rebound damping. Race cars typically run 3/2 bump to 2/3 rebound.
@export var rear_rebound_damp_multiplier := Vehicledata.rear_rebound_damp_multiplier
## Antiroll bar stiffness as a ratio to spring stiffness.
@export var rear_arb_ratio := Vehicledata.rear_arb_ratio
## Wheel camber isn't simulated, but giving the raycast a slight angle helps
## with simulation stability.
@export var rear_camber := Vehicledata.rear_camber
## Toe of the tires measured in radians.
@export var rear_toe := Vehicledata.rear_toe
## Multiplier for the force applied when the suspension is fully compressed.
## If the vehicle bounces off large bumps, reducing this will help.
@export var rear_bump_stop_multiplier := Vehicledata.rear_bump_stop_multiplier
## If true the wheels of this axle will be aligned as if they were attached to
## a beam axle. This setting does not affect vehicle handling.
@export var rear_beam_axle := Vehicledata.rear_beam_axle

@export_group("Tires")
## Represents the length of the tire contact patch in the brush tire model.
@export var contact_patch := Vehicledata.contact_patch
## Provides additional longitudinal grip when braking.
@export var braking_grip_multiplier := Vehicledata.braking_grip_multiplier
## Tire force applied to the ground is also applied to the vehicle body as a
## torque centered on the wheel. 
@export var wheel_to_body_torque_multiplier := Vehicledata.wheel_to_body_torque_multiplier
## Represents tire stiffness in the brush tire model. Higher values increase
## the responsiveness of the tire.
## Surface detection uses node groups to identify the surface, so make sure
## your staticbodies and rigidbodies belong to one of these groups.
@export var tire_stiffnesses := Vehicledata.tire_stiffnesses
## A multiplier for the amount of force a tire can apply based on the surface.
## Surface detection uses node groups to identify the surface, so make sure
## your staticbodies and rigidbodies belong to one of these groups.
@export var coefficient_of_friction := Vehicledata.coefficient_of_friction
## A multiplier for the amount of rolling resistance force based on the surface.
## Surface detection uses node groups to identify the surface, so make sure
## your staticbodies and rigidbodies belong to one of these groups.
@export var rolling_resistance := Vehicledata.rolling_resistance
## A multiplier to provide more grip based on the amount of lateral wheel slip.
## This can be used to keep vehicles from sliding a long distance, but may provide
## unrealistically high amounts of grip.
## Surface detection uses node groups to identify the surface, so make sure
## your staticbodies and rigidbodies belong to one of these groups.
@export var lateral_grip_assist := Vehicledata.lateral_grip_assist
## A multiplier to adjust longitudinal grip to differ from lateral grip.
## Useful for allowing vehicles to have wheel spin and maintain high lateral grip.
## Surface detection uses node groups to identify the surface, so make sure
## your staticbodies and rigidbodies belong to one of these groups.
@export var longitudinal_grip_ratio := Vehicledata.longitudinal_grip_ratio
@export_subgroup("Front Axle", "front_")
## Tire radius in meters
@export var front_tire_radius := Vehicledata.front_tire_radius
## Tire width in millimeters. The width doesn't directly affect tire friction,
## but reduces the effects of tire load sensitivity.
@export var front_tire_width := Vehicledata.front_tire_width
## Wheel mass in kilograms.
@export var front_wheel_mass := Vehicledata.front_wheel_mass
@export_subgroup("Rear Axle", "rear_")
## Tire radius in meters
@export var rear_tire_radius := Vehicledata.rear_tire_radius
## Tire width in millimeters. The width doesn't directly affect tire friction,
## but reduces the effects of tire load sensitivity.
@export var rear_tire_width := Vehicledata.rear_tire_width
## Wheel mass in kilograms.
@export var rear_wheel_mass := Vehicledata.rear_wheel_mass

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
@export var coefficient_of_drag := Vehicledata.coefficient_of_drag
## From [url=https://www.grc.nasa.gov/www/k-12/VirtualAero/BottleRocket/airplane/density.html#:~:text=Halving%20the%20density%20halves%20the,above%20which%20it%20cannot%20fly.]NASA[/url]:
## [i]"Halving the density halves the lift, halving the density halves the drag. The [lb]air[rb] density depends on the type of [lb]air[rb] and the depth of the [lb]air[rb]. In the atmosphere, air density decreases as altitude increases. This explains why airplanes have a flight ceiling, an altitude above which it cannot fly."[/i]
@export var air_density := Vehicledata.air_density
## The amount of surface area the front-facing part of the vehicle has,
## in meters squared ([code]m^2[/code]).
## [br][br]
## [b]Note:[/b] You do not have to calculate this value to be exact,
## a rough estimate - or even something completely different, depending
## on the result you want - will do.
@export var frontal_area := Vehicledata.frontal_area
@export var side_area:= Vehicledata.side_area            # m² (rough guess)
@export var side_drag_coeff:= Vehicledata.side_drag_coeff    # sideways drag is higher

# Physics constants - adjust these for your car
const DRAG_COEFFICIENT = 0.3
const FRONTAL_AREA = 2.2  # m²
const DOWNFORCE_COEFFICIENT = 0.8
const ROLL_STIFFNESS = 40000.0
const PITCH_STIFFNESS = 35000.0
const ANTI_ROLL_FRONT = 15000.0
const ANTI_ROLL_REAR = 12000.0

# Vehicle properties

var wheelbase: float = 2.6
var front_weight_bias: float = 0.55
var center_of_mass_height: float = 0.5
var track_width_front: float = 1.5
var track_width_rear: float = 1.5

# Dynamic variables

var wheel_loads: Array = [0.0, 0.0, 0.0, 0.0]  # FL, FR, RL, RR
var lateral_load_transfer: float = 0.0
var longitudinal_load_transfer: float = 0.0

const ANGULAR_VELOCITY_TO_RPM := 60.0 / TAU

var wheel_array : Array[Wheel] = []
var axles : Array[Axle] = []
var front_axle : Axle
var rear_axle : Axle
var drive_wheels : Array[Wheel] = []

## Controller Inputs: An external script should set these values
var throttle_input := 0.0
var steering_input := 0.0
var brake_input := 0.0
var handbrake_input := 0.0
var clutch_input := 0.0
################################################################
var current_gear_ratio: float = get_gear_ratio(current_gear)
var drive_inertia: float = motor_moment + pow(current_gear_ratio, 2.0) * gear_inertia + drive_axles_inertia

# Add these variables to your car class
var brake_temp_front := 0.0
var brake_temp_rear := 0.0
var abs_active := false
var abs_timer := 0.0

var brake_balance := 0.6  # 0.6 = 60% front, 40% rear (realistic default)

# Constants for realism
const BRAKE_TEMP_BUILDUP := 800.0
const BRAKE_TEMP_DISSIPATION := 200.0
const BRAKE_FADE_TEMP := 400.0
const BRAKE_OPTIMAL_TEMP := 200.0
const ABS_THRESHOLD := 0.15  # Wheel slip ratio threshold for ABS
const ABS_CYCLE_TIME := 0.1
const TIRE_GRIP_COEFFICIENT := 1.0

var steering_rack_ratio := 14.0  # Realistic steering rack ratio (10-20 typical)
var caster_angle := deg_to_rad(3.5)  # Caster angle for self-aligning torque
var scrub_radius := 0.02  # Scrub radius in meters (affects steering feel)
var steering_friction := 0.15  # Internal steering friction
var power_steering_assist := 0.6  # Power steering assistance (0-1)
var ackermann_factor := 0.9  # Ackermann steering geometry (0.8-1.0)
var tire_slip_angles := [0.0, 0.0, 0.0, 0.0]  # FL, FR, RL, RR slip angles

var front_load_factor := 0.6  # Front axle load distribution
var engine_to_wheel_torque: float = 0.0
var blip_strength: float = 0.0
var previous_throttle: float = 0.0
var true_throttle: float = 0.0
var load_factor: float = 1.0
var last_gear: int = 0
var blip_timer: float = 0.0
var previous_local_velocity: Vector3 = Vector3.ZERO
var longitudinal_accel: float = 0.0
var lateral_accel: float = 0.0

var is_ready := false
var local_velocity := Vector3.ZERO
var previous_global_position := Vector3.ZERO
var speed := 0.0
var motor_rpm := 0.0

var engine_temp := 85.0  # Celsius (operating temperature)
var intake_air_temp := 25.0  # Ambient air temperature
var fuel_cut_active := false
var fuel_cut_timer := 0.0
var engine_inertia := 0.15  # Engine rotational inertia
var throttle_body_position := 0.0  # Actual throttle body position
var manifold_pressure := 1.0  # Atmospheric pressure = 1.0
var engine_vacuum := 0.0  # Engine vacuum level
var overrun_fuel_cut := false

# Engine characteristics constants
const ENGINE_OPTIMAL_TEMP := 90.0
const FUEL_CUT_DURATION := 0.1
const OVERRUN_RPM_THRESHOLD := 3000.0
const VACUUM_RESPONSE_RATE := 8.0

var engine_compression_ratio := 9.5  # Realistic compression ratio
var engine_displacement := 2.0  # Liters
var turbo_boost := 0.0  # Boost pressure (0 = NA, >0 = turbocharged)
var engine_knock_threshold := 0.85  # RPM ratio where knock risk starts
var fuel_octane := 95.0  # Fuel octane rating
var engine_friction_coefficient := 0.02  # Internal friction
var valve_float_rpm := 0.0  # RPM where valve float begins (0 = disabled)
var engine_inertia_factor := 1.0  # Engine rotational inertia multiplier

# Engine state variables
var manifold_vacuum := 0.0
var combustion_efficiency := 1.0
var mechanical_efficiency := 0.9
var rev_limiter_active := false
var rev_limiter_timer := 0.0

# Constants for realism
const STOICHIOMETRIC_RATIO := 14.7  # Air/fuel ratio for complete combustion
const ENGINE_BRAKE_BASE := 0.15  # Base engine braking coefficient
const REV_LIMITER_DURATION := 0.05  # Duration of rev limiter cut
const KNOCK_POWER_REDUCTION := 0.8  # Power reduction due to knock protection


var steering_amount := 0.0
var steering_exponent_amount := 0.0
var true_steering_amount := 0.0
var throttle_amount := 0.0
var brake_amount := 0.0
var clutch_amount := 0.0
var current_gear := 0
var requested_gear := 0
var torque_output := 0.0
var clutch_torque := 0.0
var max_clutch_torque := 0.0
var drive_axles_inertia := 0.0
var complete_shift_delta_time := 0.0
var last_shift_delta_time := 0.0
var average_drive_wheel_radius := 0.0
var current_torque_split := 0.0
var true_torque_split := 0.0
var brake_force := 0.0
var max_brake_force := 0.0
var handbrake_force := 0.0
var max_handbrake_force := 0.0
var is_braking := false
var motor_is_redline := false
var is_shifting := false
var is_up_shifting := false
var need_clutch := false
var tcs_active := false
var stability_active := false
var stability_yaw_torque := 0.0
var stability_torque_vector := Vector3.ZERO
var front_axle_position := Vector3.ZERO
var rear_axle_position := Vector3.ZERO

var clutch_bite_point := 0.15  # Where clutch starts to engage (0-1)
var clutch_friction_mu := 0.35  # Clutch friction coefficient

var lsd_factor := 1.0  # Limited slip differential factor (1.0 = open diff)

var lateral_grip_threshold := 0.85  # When lateral forces exceed this ratio of max grip
var longitudinal_grip_threshold := 0.9  # For detecting wheel spin/lock
var stability_intervention_delay := 0.1  # Seconds before full intervention
var current_intervention_time := 0.0
var wheel_slip_ratios: Array[float] = []  # Track slip for each wheel
var lateral_acceleration_filter := 0.0  # Smoothed lateral G-force
var delta_time := 0.0

var transmission_state: int = 0  # 0=normal, 1=shifting, 2=emergency
var shift_timer: float = 0.0
var shift_target_gear: int = 0
var clutch_slip_velocity: float = 0.0
var engine_load_factor: float = 1.0
var transmission_temperature: float = 85.0
var last_valid_speed: float = 0.0
var speed_filter_buffer: Array = []
var rpm_filter_buffer: Array = []
var gear_change_lockout: float = 0.0
var stall_prevention_active: bool = false
var torque_converter_slip: float = 0.0
var differential_lock_factor: float = 0.0

var wheel_load_distribution: Array[float] = []  # Track individual wheel loads
var contact_patch_quality: Array[float] = []    # Contact patch effectiveness per wheel
var surface_grip_multipliers: Array[float] = [] # Surface-specific grip per wheel
var minimum_contact_normal_dot := 0.3           # Less restrictive for banking/hills
var suspension_travel_threshold := 0.05         # Min compression for meaningful contact
var load_transfer_sensitivity := 1.2            # How much load affects contact quality
var airborne_threshold := 0.1                   # Contact sum below this = airborne
var last_contact_count := 0.0                   # Track for landing detection

var front_downforce_coefficient: float = 0.15  # Front wing/splitter Cl
var rear_downforce_coefficient: float = 0.25   # Rear wing/diffuser Cl
var center_of_pressure_offset: float = 0.2     # Distance behind center of mass (positive = rearward)
var ground_effect_strength: float = 0.1        # Additional downforce near ground
var ride_height_optimal: float = 0.15          # Optimal ride height for ground effect
var stall_angle: float = 15.0                  # Angle where wings stall (degrees)

# Aerodynamic balance
var front_aero_balance: float = 0.4  # 0.4 = 40% front, 60% rear
var underbody_downforce: float = 0.05  # Flat floor/diffuser contribution

var downforce_coefficient: float = 0.0  # Cl - lift coefficient (negative for downforce)
var tire_rolling_resistance: Array[float] = [0.015, 0.015, 0.015, 0.015]  # Per wheel
var brake_drag_coefficient: float = 0.0  # Additional drag when braking
var cooling_drag_factor: float = 1.0  # Increases with engine load/temperature

# Physics constants
const MIN_SPEED_THRESHOLD: float = 0.05
const TIRE_HEAT_FACTOR: float = 1.2  # Rolling resistance increases with heat
const ALTITUDE_FACTOR: float = 1.0  # Air density factor (1.0 = sea level)

var raw_throttle_input: float = 0.0
var raw_brake_input: float = 0.0
var filtered_throttle: float = 0.0
var filtered_brake: float = 0.0
var throttle_history: Array[float] = []
var brake_history: Array[float] = []

# Input configuration
var throttle_deadzone: float = 0.02  # Dead zone to prevent micro-inputs
var brake_deadzone: float = 0.02
var throttle_sensitivity: float = 2.0  # Input curve exponent (>1 = more sensitive at low inputs)
var brake_sensitivity: float = 1.5
var input_filter_strength: float = 0.15  # Smoothing filter (lower = more smoothing)
var max_input_rate: float = 8.0  # Maximum input change per second
var trail_braking_assist: float = 0.0  # 0.0-1.0, helps with brake modulation

# Pedal simulation
var throttle_travel: float = 0.0  # Simulated pedal position
var brake_travel: float = 0.0
var pedal_response_rate: float = 12.0  # How quickly pedals respond
var brake_pressure_curve: float = 1.8  # Non-linear brake pressure buildup

var longitudinal_slip_threshold := 0.12       # 12% longitudinal slip threshold
var lateral_slip_threshold := 0.08           # 8% lateral slip threshold  
var combined_slip_threshold := 0.15          # Combined slip threshold
var wheel_slip_states: Array[int] = []       # 0=grip, 1=slip, 2=spin, 3=lock

var wheel_lateral_slip: Array[float] = []    # Lateral slip angles per wheel
var peak_grip_slip_ratio := 0.08             # Peak grip occurs around 8% slip
var lockup_threshold := 0.95                 # 95% slip = wheel lockup
var spin_threshold := 1.5                    # 150% slip = wheel spin
var slip_hysteresis := 0.02                  # Prevent oscillation at threshold
var surface_slip_multipliers: Array[float] = [] # Surface-dependent slip behavior
var tire_compound_factor := 1.0              # Different tire compounds

# Safety constants
const ANGULAR_TO_RPM = 9.549296596  # 60/(2*PI)
const MAX_SAFE_SPEED = 120.0  # m/s (~432 km/h)
const MIN_SAFE_RPM = 300.0
const MAX_SAFE_RPM = 9000.0
const GEAR_CHANGE_TIME = 0.18  # 180ms realistic shift time
const EMERGENCY_TIMEOUT = 0.5

var vehicle_inertia : Vector3
var current_gravity : Vector3

var shift_protection_rpm_high := 6800.0  # Rev limiter during shifts
var shift_protection_rpm_low := 1200.0   # Prevent lugging engine
var clutch_engagement_speed := 4.0        # How fast clutch engages/disengages
var rev_match_strength := 0.8            # How aggressively to rev-match
var shift_shock_threshold := 800.0       # RPM difference causing harsh shifts
var engine_brake_factor := 0.15          # Engine braking strength
var flywheel_inertia := 0.12             # Engine rotational inertia
var clutch_slip_threshold := 200.0       # RPM diff where clutch starts slipping
var gear_whine_factor := 0.0             # For audio/feedback (if needed)
var last_engine_load := 0.0              # Track engine load for realistic behavior

var clutch_temperature: float = 0.0  # Heat buildup affects clutch performance
var drivetrain_load_history: Array[float] = []  # For load-based shift timing
var reverse_lockout_timer: float = 0.0  # Prevents rapid reverse/forward switching
var shift_quality_factor: float = 1.0  # Affects how smooth shifts are

class Axle:
	var differential_preload_torque: float = 50.0  # Nm, tweak between 20–150
	var anti_roll_stiffness : float = 5000.0  # Torque applied to resist roll
	var anti_roll_damping : float = 1000.0    # Damping for oscillations
	var wheels : Array[Wheel] = []
	var is_drive_axle := false
	var inertia := 0.0
	var handbrake := false
	var brake_bias := 0.5
	var rotation_split := 0.5
	var applied_split := 0.5
	var torque_vectoring := 0.0
	var suspension_compression_left := 0.0
	var suspension_compression_right := 0.0
	var tire_size_correction := 0.0
	var differential_lock_torque := 0.0
	
	
	# New variables for realistic physics
	var differential_viscous_coupling: float = 0.15  # Viscous LSD effect (0-1)
	var temperature_factor: float = 1.0  # Heat buildup affects grip
	var lateral_load_transfer: float = 0.0  # For realistic weight distribution
	
	# Internal variables for smooth operation
	var _previous_avg_spin: float = 0.0
	var _spin_smoothing: float = 0.95
	var _previous_max_slip: float = 0.0  # For slip smoothing
	var _slip_smoothing_factor: float = 0.85  # Smooth slip transitions
	


	# New realistic variables
	var axle_type: String = "locked"  # "open", "limited_slip", "locked", "torsen"
	var lsd_clutch_plates: int = 6
	var lsd_ramp_angles: Vector2 = Vector2(45.0, 20.0)  # power/coast angles
	var anti_roll_bar_stiffness: float = 0.0
	var track_width: float = 1.5  # meters
	var unsprung_mass: float = 40.0  # kg per wheel
	var tire_compound: String = "soft"  # "soft", "medium", "hard"
	var tire_pressure: float = 2.2  # bar
	var camber_angle: float = 0.0  # degrees
	var toe_angle: float = 0.0  # degrees
	var caster_angle: float = 6.0  # degrees

	# Tire model parameters (Pacejka-like)
	var tire_peak_friction: float = 2.2
	var tire_shape_factor: float = 1.3
	var tire_stiffness_factor: float = 10.0
	var tire_vertical_stiffness: float = 200000.0  # N/m

	# Historical data for realistic behavior
	var slip_history: Array = []
	var temperature_history: Array = []
	var wear_level: float = 0.0

	func get_spin(vehicle_velocity: Vector3, vehicle_basis: Basis) -> float:
		var total_slip: float = 0.0
		var total_torque: float = 0.0
		var count: int = 0
		var max_axle_load: float = 5000.0
		
		# Update tire temperatures and wear
		_update_tire_conditions()
		
		# Calculate advanced load transfers
		_calculate_advanced_load_transfer(vehicle_velocity, vehicle_basis)
		
		for wheel in wheels:
			if wheel.is_driven:
				# Enhanced wheel speed with tire model
				var wheel_linear_vel: float = _get_advanced_wheel_speed(wheel)
				
				# Vehicle speed with suspension geometry effects
				var wheel_forward: Vector3 = _get_wheel_forward_with_geometry(wheel, vehicle_basis)
				var forward_speed: float = vehicle_velocity.dot(wheel_forward)
				
				# Advanced slip calculation with tire model
				var slip_ratio: float = _calculate_pacejka_slip(wheel_linear_vel, forward_speed, wheel)
				
				# Dynamic load with suspension effects
				var dynamic_load: float = _calculate_advanced_wheel_load(wheel, max_axle_load, vehicle_velocity, vehicle_basis)
				
				# Tire grip with compound and temperature effects
				var grip_factor: float = _calculate_tire_grip(wheel, dynamic_load)
				
				# Effective slip with all realistic factors
				var effective_slip: float = slip_ratio * grip_factor
				
				# Apply advanced differential model
				effective_slip = _apply_advanced_differential(effective_slip, wheel, vehicle_velocity)
				
				# Suspension geometry effects on slip
				effective_slip = _apply_suspension_geometry(effective_slip, wheel)
				
				# Weight by torque distribution
				var wheel_torque: float = _get_realistic_wheel_torque(wheel)
				
				total_slip += effective_slip * wheel_torque
				total_torque += wheel_torque
				count += 1
		
		if count == 0 or total_torque == 0.0:
			return 0.0
		
		var avg_slip: float = total_slip / total_torque
		
		# Apply final realistic corrections
		avg_slip = _apply_advanced_corrections(avg_slip)
		
		# Update historical data
		_update_slip_history(avg_slip)
		
		return avg_slip

	# Update tire temperatures and wear realistically
	func _update_tire_conditions():
		for wheel in wheels:
			# Temperature buildup from friction
			var lateral_slip = wheel.get("lateral_slip") if wheel.has("lateral_slip") else 0.0
			var longitudinal_slip = wheel.get("longitudinal_slip") if wheel.has("longitudinal_slip") else 0.0
			var friction_heat: float = abs(lateral_slip) + abs(longitudinal_slip)
			var temp_increase: float = friction_heat * 0.1
			
			# Cooling from airflow
			var cooling_rate: float = 0.05
			var current_temp = wheel.get("tire_temperature") if wheel.has("tire_temperature") else 80.0
			wheel["tire_temperature"] = lerp(current_temp, 80.0 + temp_increase, cooling_rate)
			
			# Wear accumulation
			var wear_rate: float = friction_heat * 0.001
			var current_wear = wheel.get("tire_wear") if wheel.has("tire_wear") else 0.0
			wheel["tire_wear"] = current_wear + wear_rate

	# Advanced load transfer with anti-roll bars and suspension
	func _calculate_advanced_load_transfer(vehicle_velocity: Vector3, vehicle_basis: Basis):
		var lateral_vel: float = vehicle_velocity.dot(vehicle_basis.x)
		var lateral_accel: float = lateral_vel * 0.15  # More realistic lateral g calculation
		
		# Roll moment with anti-roll bar
		var roll_stiffness: float = anti_roll_bar_stiffness + 50000.0  # Base suspension roll stiffness
		var roll_moment: float = lateral_accel * 1500.0  # Vehicle mass * CG height
		var roll_angle: float = roll_moment / roll_stiffness
		
		# Load transfer with geometry
		lateral_load_transfer = (roll_moment / track_width) * 0.5
		
		# Longitudinal load transfer
		var longitudinal_accel: float = vehicle_velocity.length() * 0.1
		var pitch_transfer: float = longitudinal_accel * 0.4

	# Enhanced wheel speed with tire deformation and pressure effects
	func _get_advanced_wheel_speed(wheel) -> float:
		var base_speed: float = wheel.spin * wheel.tire_radius
		
		# Tire pressure effects on radius
		var pressure_effect: float = (tire_pressure - 2.2) * 0.01  # Nominal 2.2 bar
		var effective_radius: float = wheel.tire_radius * (1.0 + pressure_effect)
		
		# Load deformation (more realistic curve)
		var load_deformation: float = 1.0
		if "normal_force" in wheel:
			var normalized_load: float = wheel.normal_force / tire_vertical_stiffness
			load_deformation = 1.0 / (1.0 + normalized_load * 0.05)
		
		# Temperature effects on tire flexibility
		var temp_factor: float = 1.0
		if "tire_temperature" in wheel:
			var optimal_temp: float = 90.0
			var temp_diff: float = abs(wheel.tire_temperature - optimal_temp)
			temp_factor = 1.0 - (temp_diff / 200.0) * 0.1
		
		return base_speed * effective_radius * load_deformation * temp_factor

	# Wheel forward vector with suspension geometry
	func _get_wheel_forward_with_geometry(wheel, vehicle_basis: Basis) -> Vector3:
		var wheel_forward: Vector3 = vehicle_basis.z
		
		# Steering angle
		if "steering_angle" in wheel:
			var steering_rad: float = deg_to_rad(wheel.steering_angle)
			wheel_forward = wheel_forward.rotated(vehicle_basis.y, steering_rad)
		
		# Toe angle effect
		var toe_rad: float = deg_to_rad(toe_angle)
		wheel_forward = wheel_forward.rotated(vehicle_basis.y, toe_rad)
		
		# Camber doesn't affect forward direction but noted for completeness
		return wheel_forward

	# Pacejka-inspired tire model for slip calculation
	func _calculate_pacejka_slip(wheel_speed: float, vehicle_speed: float, wheel) -> float:
		var speed_threshold: float = 0.1
		var slip: float = 0.0
		
		if abs(vehicle_speed) < speed_threshold:
			slip = clamp(wheel_speed / speed_threshold, -3.0, 3.0)
		else:
			slip = (wheel_speed - vehicle_speed) / abs(vehicle_speed)
			slip = clamp(slip, -3.0, 3.0)
		
		# Apply Pacejka magic formula characteristics
		var normalized_slip: float = abs(slip) / 0.15  # Peak slip around 15%
		var pacejka_factor: float = tire_shape_factor * sin(tire_stiffness_factor * atan(normalized_slip))
		
		return sign(slip) * normalized_slip * pacejka_factor * 0.15

	# Advanced wheel load with suspension and aerodynamics
	func _calculate_advanced_wheel_load(wheel, max_load: float, vehicle_velocity: Vector3, vehicle_basis: Basis) -> float:
		var base_load: float = max_load * 0.25
		
		if "normal_force" in wheel:
			base_load = wheel.normal_force
		
		# Lateral load transfer
		var is_left_wheel: bool = wheels.find(wheel) % 2 == 0
		var lateral_transfer: float = lateral_load_transfer * (1.0 if is_left_wheel else -1.0)
		
		# Longitudinal load transfer (braking/acceleration)
		var is_front_wheel: bool = wheels.find(wheel) < 2
		var speed: float = vehicle_velocity.length()
		var longitudinal_transfer: float = speed * 10.0 * (1.0 if is_front_wheel else -1.0)
		
		# Aerodynamic downforce (speed dependent)
		var aero_downforce: float = pow(speed / 50.0, 2.0) * 200.0  # Increases with speed²
		
		# Unsprung mass effects
		var unsprung_effect: float = unsprung_mass * 9.81
		
		var total_load: float = base_load + lateral_transfer + longitudinal_transfer + aero_downforce + unsprung_effect
		return max(total_load, max_load * 0.05)  # Minimum 5% load

	# Realistic tire grip calculation
	func _calculate_tire_grip(wheel, dynamic_load: float) -> float:
		var base_grip: float = tire_peak_friction
		
		# Load sensitivity (realistic tire curve)
		var normalized_load: float = dynamic_load / 2500.0
		var load_factor: float = sqrt(normalized_load) * (2.0 - normalized_load * 0.3)
		load_factor = clamp(load_factor, 0.4, 1.2)
		
		# Temperature effects
		var temp_factor: float = 1.0
		if "tire_temperature" in wheel:
			var optimal_temp: float = 90.0
			var temp: float = wheel.tire_temperature
			if temp < optimal_temp:
				temp_factor = 0.7 + (temp / optimal_temp) * 0.3
			else:
				temp_factor = 1.0 - ((temp - optimal_temp) / 100.0) * 0.4
			temp_factor = clamp(temp_factor, 0.3, 1.1)
		
		# Tire compound effects
		var compound_factor: float = 1.0
		match tire_compound:
			"soft": compound_factor = 1.15
			"medium": compound_factor = 1.0
			"hard": compound_factor = 0.92
		
		# Wear effects
		var wear_factor: float = 1.0
		if "tire_wear" in wheel:
			wear_factor = 1.0 - wheel.tire_wear * 0.3
			wear_factor = clamp(wear_factor, 0.6, 1.0)
		
		# Pressure effects
		var pressure_factor: float = 1.0 - abs(tire_pressure - 2.2) * 0.1
		pressure_factor = clamp(pressure_factor, 0.8, 1.05)
		
		return base_grip * load_factor * temp_factor * compound_factor * wear_factor * pressure_factor

	# Advanced differential modeling
	func _apply_advanced_differential(slip: float, wheel, vehicle_velocity: Vector3) -> float:
		var modified_slip: float = slip
		var wheel_index: int = wheels.find(wheel)
		var opposite_wheel_index: int = wheel_index + 1 if wheel_index % 2 == 0 else wheel_index - 1
		
		if opposite_wheel_index < wheels.size():
			var opposite_wheel = wheels[opposite_wheel_index]
			var speed_diff: float = abs(wheel.spin - opposite_wheel.spin)
			
			match axle_type:
				"open":
					# Open differential - minimal resistance to speed difference
					modified_slip *= (1.0 + speed_diff * 0.01)
				
				"limited_slip":
					# LSD with ramp angles and clutch plates
					var torque_bias: float = _calculate_lsd_bias(speed_diff, slip > 0)
					modified_slip *= (1.0 - torque_bias * 0.3)
				
				"torsen":
					# Torsen differential - torque sensitive
					var torque_ratio: float = _calculate_torsen_ratio(wheel, opposite_wheel)
					modified_slip *= torque_ratio
				
				"locked":
					# Locked differential - force equal speeds
					modified_slip *= 0.5
		
		# Viscous coupling effects
		if differential_viscous_coupling > 0.0:
			var coupling_effect: float = differential_viscous_coupling * abs(slip) * 0.1
			modified_slip *= (1.0 - coupling_effect)
		
		return modified_slip

	# LSD bias calculation with ramp angles
	func _calculate_lsd_bias(speed_diff: float, is_power: bool) -> float:
		var ramp_angle: float = lsd_ramp_angles.x if is_power else lsd_ramp_angles.y
		var separation_force: float = speed_diff * tan(deg_to_rad(ramp_angle))
		var clutch_capacity: float = lsd_clutch_plates * 50.0  # Nm per plate
		
		var bias_ratio: float = separation_force / clutch_capacity
		return clamp(bias_ratio, 0.0, 0.8)

	# Torsen differential torque ratio
	func _calculate_torsen_ratio(wheel1, wheel2) -> float:
		var torque1: float = abs(wheel1.get("drive_torque", 100.0))
		var torque2: float = abs(wheel2.get("drive_torque", 100.0))
		
		if torque2 == 0.0:
			return 1.0
		
		var ratio: float = torque1 / torque2
		var bias_ratio: float = 3.0  # Typical Torsen bias ratio
		
		return clamp(ratio, 1.0/bias_ratio, bias_ratio) / bias_ratio

	# Suspension geometry effects on slip
	func _apply_suspension_geometry(slip: float, wheel) -> float:
		var modified_slip: float = slip
		
		# Camber effects on contact patch
		var camber_factor: float = cos(deg_to_rad(abs(camber_angle)))
		modified_slip /= camber_factor
		
		# Caster effects on self-aligning torque
		var caster_factor: float = 1.0 + (caster_angle / 90.0) * 0.1
		modified_slip *= caster_factor
		
		return modified_slip

	# Realistic wheel torque with advanced distribution
	func _get_realistic_wheel_torque(wheel) -> float:
		var base_torque: float = abs(wheel.get("drive_torque", 100.0))
		
		# Torque vectoring effects
		if torque_vectoring != 0.0:
			var is_left_wheel: bool = wheels.find(wheel) % 2 == 0
			var vectoring_factor: float = 1.0 + (torque_vectoring * 0.5 * (1.0 if is_left_wheel else -1.0))
			base_torque *= clamp(vectoring_factor, 0.2, 1.8)
		
		# Engine braking and coast effects
		var engine_braking: float = wheel.get("engine_braking", 0.0)
		base_torque += engine_braking
		
		return max(base_torque, 1.0)  # Minimum torque for calculation stability

	# Advanced corrections with realistic physics
	func _apply_advanced_corrections(slip: float) -> float:
		var corrected_slip: float = slip
		
		# Temperature correction with realistic curve
		var temp_correction: float = temperature_factor
		if temp_correction < 0.8:
			temp_correction *= 0.9  # Cold tires lose more grip
		elif temp_correction > 1.2:
			temp_correction *= 1.1  # Overheated tires lose grip faster
		
		corrected_slip *= temp_correction
		
		# Tire size correction
		corrected_slip *= (1.0 + tire_size_correction * 0.5)
		
		# Apply realistic smoothing with hysteresis
		var smoothing: float = 0.92
		if abs(corrected_slip) > abs(slip_history[-1] if slip_history.size() > 0 else 0.0):
			smoothing = 0.88  # Less smoothing when slip increases
		
		corrected_slip = lerp(slip_history[-1] if slip_history.size() > 0 else 0.0, corrected_slip, smoothing)
		
		# Final realistic range
		return clamp(corrected_slip, -2.5, 2.5)

	# Update slip history for realistic behavior
	func _update_slip_history(slip: float):
		slip_history.append(slip)
		if slip_history.size() > 10:
			slip_history.pop_front()
		
		# Update wear level
		wear_level += abs(slip) * 0.0001
		wear_level = clamp(wear_level, 0.0, 1.0)
	func get_average_spin() -> float:
		if wheels.is_empty():
			return 0.0
		
		var total_spin: float = 0.0
		var total_weight: float = 0.0
		var driven_count: int = 0
		
		for wheel in wheels:
			if wheel.is_driven:
				# Get base wheel spin
				var wheel_spin: float = wheel.spin
				
				# Calculate wheel weight based on load and differential effects
				var wheel_weight: float = _calculate_wheel_weight(wheel)
				
				# Apply realistic differential effects
				wheel_spin = _apply_differential_spin_effects(wheel_spin, wheel)
				
				# Clamp individual wheel spin to prevent extreme values
				wheel_spin = clamp(wheel_spin, -500.0, 500.0)  # Reasonable RPM limits
				
				total_spin += wheel_spin * wheel_weight
				total_weight += wheel_weight
				driven_count += 1
		
		if driven_count == 0 or total_weight == 0.0:
			return 0.0
		
		# Calculate weighted average
		var raw_average: float = total_spin / total_weight
		
		# Apply realistic corrections
		var corrected_spin: float = _apply_realistic_corrections(raw_average)
		
		# Smooth the output to prevent erratic behavior
		var smoothed_spin: float = lerp(_previous_avg_spin, corrected_spin, 1.0 - _spin_smoothing)
		_previous_avg_spin = smoothed_spin
		
		# Final safety clamp
		return clamp(smoothed_spin, -400.0, 400.0)  # Conservative limits for stability
	
	# Calculate realistic wheel weight for averaging
	func _calculate_wheel_weight(wheel) -> float:
		var base_weight: float = 1.0
		
		# Weight by load - more loaded wheels have more influence
		if "normal_force" in wheel:
			var load_factor: float = wheel.normal_force / 2500.0  # Normalize to typical load
			base_weight *= clamp(load_factor, 0.5, 1.5)  # Conservative range
		
		# Adjust for temperature effects (minimal impact for stability)
		base_weight *= lerp(0.95, 1.05, temperature_factor)
		
		return base_weight
	
	# Apply differential effects to individual wheel spin
	func _apply_differential_spin_effects(spin: float, wheel) -> float:
		var modified_spin: float = spin
		
		# Viscous coupling - reduces spin differences slightly
		if differential_viscous_coupling > 0.0:
			var coupling_effect: float = differential_viscous_coupling * 0.1  # Small effect
			modified_spin *= (1.0 - coupling_effect)
		
		# Preload torque effect - very minimal for stability
		if differential_preload_torque > 0.0:
			var preload_factor: float = differential_preload_torque / 1000.0  # Much smaller factor
			modified_spin *= (1.0 + preload_factor)
		
		return modified_spin
	
	# Apply realistic corrections while maintaining stability
	func _apply_realistic_corrections(spin: float) -> float:
		var corrected: float = spin
		
		# Temperature factor - small realistic effect
		corrected *= clamp(temperature_factor, 0.9, 1.1)  # Limited range for stability
		
		# Tire size correction - small effect
		var size_factor: float = 1.0 + (tire_size_correction * 0.1)  # Reduced impact
		corrected *= clamp(size_factor, 0.95, 1.05)
		
		# Differential lock effect - subtle
		if differential_lock_torque > 0.0:
			var lock_factor: float = 1.0 + (differential_lock_torque / 10000.0)  # Very small effect
			corrected *= clamp(lock_factor, 0.98, 1.02)
		
		return corrected

	

	func get_max_wheel_slip_y(vehicle_mass: float) -> float:
		if wheels.is_empty() or vehicle_mass <= 0.0:
			return 0.0
		
		var max_slip: float = 0.0
		var total_load: float = 0.0
		var effective_wheels: int = 0
		
		# Calculate total axle load for normalization
		for wheel in wheels:
			if wheel.has_method("get_load") and wheel.get_load() > 0.0:
				total_load += wheel.get_load()
				effective_wheels += 1
		
		if effective_wheels == 0 or total_load <= 0.0:
			return 0.0
		
		for wheel in wheels:
			if wheel.has_method("get_load") and wheel.get_load() > 0.0:
				# Get base lateral slip
				var base_slip_y: float = absf(wheel.slip_vector.y)
				
				# Calculate realistic load factor
				var load_factor: float = _calculate_realistic_load_factor(wheel, total_load, vehicle_mass)
				
				# Apply tire physics corrections
				var corrected_slip: float = _apply_tire_physics_corrections(base_slip_y, wheel, load_factor)
				
				# Account for suspension and geometry effects
				corrected_slip = _apply_suspension_effects(corrected_slip, wheel)
				
				# Temperature and wear effects
				corrected_slip = _apply_temperature_effects(corrected_slip)
				
				# Track maximum slip
				max_slip = max(max_slip, corrected_slip)
		
		# Apply final realistic corrections
		max_slip = _apply_final_slip_corrections(max_slip, vehicle_mass)
		
		# Smooth the output to prevent erratic behavior
		var smoothed_slip: float = lerp(_previous_max_slip, max_slip, 1.0 - _slip_smoothing_factor)
		_previous_max_slip = smoothed_slip
		
		# Conservative clamping for stability
		return clamp(smoothed_slip, 0.0, 2.0)  # Realistic slip range
	
	# Calculate realistic load factor with proper tire physics
	func _calculate_realistic_load_factor(wheel, total_axle_load: float, vehicle_mass: float) -> float:
		var wheel_load: float = wheel.get_load()
		var expected_load: float = vehicle_mass * 0.25  # 25% per wheel baseline
		
		# Normalized load factor
		var load_ratio: float = wheel_load / expected_load
		
		# Realistic tire load sensitivity curve
		# More load doesn't equal proportionally more grip
		var load_factor: float
		if load_ratio < 0.8:
			# Under-loaded: reduced grip efficiency
			load_factor = load_ratio * 1.1
		elif load_ratio > 1.5:
			# Over-loaded: diminishing returns
			load_factor = 0.8 + (load_ratio - 1.5) * 0.3
		else:
			# Normal load range: near-linear
			load_factor = load_ratio * 0.9 + 0.1
		
		# Account for lateral load transfer
		if lateral_load_transfer != 0.0:
			var is_left_wheel: bool = wheels.find(wheel) % 2 == 0
			var transfer_effect: float = lateral_load_transfer * 0.15  # 15% max effect
			if is_left_wheel:
				transfer_effect *= -1.0
			load_factor += transfer_effect
		
		return clamp(load_factor, 0.3, 1.4)  # Realistic bounds
	
	# Apply tire physics corrections for realistic behavior
	func _apply_tire_physics_corrections(slip: float, wheel, load_factor: float) -> float:
		var corrected_slip: float = slip
		
		# Load-dependent slip behavior
		corrected_slip *= load_factor
		
		# Tire pressure effects (simplified)
		if "tire_pressure" in wheel:
			var pressure_factor: float = clamp(wheel.tire_pressure / 30.0, 0.85, 1.15)  # 30 PSI baseline
			corrected_slip *= pressure_factor
		
		# Speed-dependent slip characteristics
		var speed_factor: float = 1.0
		if "linear_velocity" in wheel:
			var speed: float = wheel.linear_velocity.length()
			# Higher speeds = slightly more slip sensitivity
			speed_factor = 1.0 + (speed / 100.0) * 0.1  # 10% increase at 100 m/s
		corrected_slip *= clamp(speed_factor, 1.0, 1.2)
		
		return corrected_slip
	
	# Apply suspension geometry and compression effects
	func _apply_suspension_effects(slip: float, wheel) -> float:
		var modified_slip: float = slip
		
		# Suspension compression affects contact patch
		var compression_factor: float = 1.0
		var wheel_index: int = wheels.find(wheel)
		
		if wheel_index == 0 or wheel_index == 1:  # Front wheels
			compression_factor = 1.0 + (suspension_compression_left * 0.05)
		else:  # Rear wheels  
			compression_factor = 1.0 + (suspension_compression_right * 0.05)
		
		modified_slip *= clamp(compression_factor, 0.95, 1.1)
		
		# Camber effects (simplified)
		if "camber_angle" in wheel:
			var camber_effect: float = 1.0 + (abs(wheel.camber_angle) * 0.01)  # 1% per degree
			modified_slip *= clamp(camber_effect, 1.0, 1.15)
		
		return modified_slip
	
	# Apply temperature effects on tire grip
	func _apply_temperature_effects(slip: float) -> float:
		# Temperature affects grip characteristics
		var temp_corrected: float = slip
		
		# Hot tires have different slip characteristics
		if temperature_factor > 1.1:
			# Overheated: more slip
			temp_corrected *= temperature_factor * 1.05
		elif temperature_factor < 0.9:
			# Cold tires: also more slip initially
			temp_corrected *= (2.0 - temperature_factor) * 1.1
		
		return temp_corrected
	
	# Apply final corrections and safety measures
	func _apply_final_slip_corrections(slip: float, vehicle_mass: float) -> float:
		var final_slip: float = slip
		
		# Mass-dependent correction (heavier cars = different slip characteristics)
		var mass_factor: float = clamp(vehicle_mass / 1500.0, 0.8, 1.3)  # 1500kg baseline
		final_slip *= mass_factor
		
		# Anti-roll bar effects
		if anti_roll_stiffness > 0.0:
			var roll_factor: float = 1.0 + (anti_roll_stiffness / 50000.0)  # Minimal effect
			final_slip *= clamp(roll_factor, 1.0, 1.05)
		
		# Differential effects on slip distribution
		if differential_preload_torque > 0.0:
			var diff_factor: float = 1.0 + (differential_preload_torque / 1000.0)
			final_slip *= clamp(diff_factor, 1.0, 1.1)
		
		return final_slip


func _ready():
	initialize()

func _integrate_forces(state: PhysicsDirectBodyState3D):
	var dt = state.step
	if dt <= 0.0:
		return
	
	# Transform gravity to local space
	current_gravity = global_transform.basis.inverse() * state.total_gravity
	var gravity_magnitude = current_gravity.length()
	
	# Get velocity in local and global space
	var local_velocity = global_transform.basis.inverse() * state.linear_velocity
	var speed = state.linear_velocity.length()
	
	# Calculate aerodynamic forces
	var aero_forces = _calculate_aerodynamics(local_velocity, speed)
	
	# Calculate load transfer and wheel loads
	_calculate_load_transfer(state, local_velocity)
	
	# Get forces from wheels (assumes you have wheel raycast system)
	var wheel_forces = _get_wheel_forces(local_velocity, dt)
	
	# Combine all forces
	var total_force = Vector3.ZERO
	var total_torque = Vector3.ZERO
	
	# Add wheel forces and torques
	for i in range(4):
		var wheel_pos = _get_wheel_position(i)  # You'll need to implement this
		var force = wheel_forces[i]
		
		total_force += force
		total_torque += wheel_pos.cross(force)
	
	# Add aerodynamic forces
	total_force += aero_forces.drag + aero_forces.downforce
	total_torque += aero_forces.torque
	
	# Add body roll/pitch resistance
	var body_torque = _calculate_body_dynamics(state)
	total_torque += body_torque
	
	# Apply forces in global space
	state.apply_central_force(global_transform.basis * total_force)
	state.apply_torque(global_transform.basis * total_torque)

func _calculate_aerodynamics(local_velocity: Vector3, speed: float) -> Dictionary:
	var air_density = 1.225  # kg/m³ at sea level
	var dynamic_pressure = 0.5 * air_density * speed * speed
	
	# Drag force (always opposing motion)
	var drag_force = Vector3.ZERO
	if speed > 0.1:
		var drag_magnitude = DRAG_COEFFICIENT * FRONTAL_AREA * dynamic_pressure
		
	
	# Downforce (proportional to speed squared, acts downward)
	var downforce = Vector3(0, -DOWNFORCE_COEFFICIENT * FRONTAL_AREA * dynamic_pressure, 0)
	
	# Aerodynamic pitch moment (simplified)
	var aero_torque = Vector3(downforce.y * 0.1, 0, 0)  # Small pitch moment from downforce
	
	return {
		"drag": drag_force,
		"downforce": downforce,
		"torque": aero_torque
	}

func _calculate_load_transfer(state: PhysicsDirectBodyState3D, local_velocity: Vector3):
	var local_accel = global_transform.basis.inverse() * (state.linear_velocity - state.get_velocity_at_local_position(Vector3.ZERO))
	
	# Longitudinal load transfer (braking/acceleration)
	var longitudinal_g = -local_accel.z / current_gravity.length()
	longitudinal_load_transfer = mass * current_gravity.length() * center_of_mass_height * longitudinal_g / wheelbase
	
	# Lateral load transfer (cornering)
	var lateral_g = local_accel.x / current_gravity.length()
	lateral_load_transfer = mass * current_gravity.length() * center_of_mass_height * lateral_g / ((track_width_front + track_width_rear) * 0.5)
	
	# Calculate individual wheel loads
	var static_front = mass * current_gravity.length() * front_weight_bias * 0.5
	var static_rear = mass * current_gravity.length() * (1.0 - front_weight_bias) * 0.5
	
	# Front wheels
	wheel_loads[0] = static_front - longitudinal_load_transfer * front_weight_bias - lateral_load_transfer  # FL
	wheel_loads[1] = static_front - longitudinal_load_transfer * front_weight_bias + lateral_load_transfer  # FR
	
	# Rear wheels  
	wheel_loads[2] = static_rear + longitudinal_load_transfer * (1.0 - front_weight_bias) - lateral_load_transfer  # RL
	wheel_loads[3] = static_rear + longitudinal_load_transfer * (1.0 - front_weight_bias) + lateral_load_transfer  # RR
	
	# Ensure no negative loads
	for i in range(4):
		wheel_loads[i] = max(wheel_loads[i], 0.0)

func _calculate_body_dynamics(state: PhysicsDirectBodyState3D) -> Vector3:
	var body_torque = Vector3.ZERO
	var angular_velocity = state.angular_velocity
	var local_angular_vel = global_transform.basis.inverse() * angular_velocity
	
	# Roll resistance (around Z-axis)
	var roll_angle = atan2(global_transform.basis.y.x, global_transform.basis.y.y)
	var roll_torque = -ROLL_STIFFNESS * roll_angle - local_angular_vel.z * 8000.0  # Roll damping
	body_torque.z += roll_torque
	
	# Pitch resistance (around X-axis) 
	var pitch_angle = asin(-global_transform.basis.y.z)
	var pitch_torque = -PITCH_STIFFNESS * pitch_angle - local_angular_vel.x * 6000.0  # Pitch damping
	body_torque.x += pitch_torque
	
	# Anti-roll bars (simplified)
	var front_roll_diff = (wheel_loads[1] - wheel_loads[0]) / (wheel_loads[1] + wheel_loads[0] + 0.001)
	var rear_roll_diff = (wheel_loads[3] - wheel_loads[2]) / (wheel_loads[3] + wheel_loads[2] + 0.001)
	
	body_torque.z -= ANTI_ROLL_FRONT * front_roll_diff * 0.1
	body_torque.z -= ANTI_ROLL_REAR * rear_roll_diff * 0.1
	
	return body_torque

# Helper function - you'll need to implement based on your wheel positions
func _get_wheel_position(wheel_index: int) -> Vector3:
	# Return local position of each wheel relative to center of mass
	# This is crucial for proper torque calculation
	match wheel_index:
		0: return Vector3(-track_width_front * 0.5, -center_of_mass_height, wheelbase * front_weight_bias)  # FL
		1: return Vector3(track_width_front * 0.5, -center_of_mass_height, wheelbase * front_weight_bias)   # FR
		2: return Vector3(-track_width_rear * 0.5, -center_of_mass_height, -wheelbase * (1.0 - front_weight_bias))  # RL
		3: return Vector3(track_width_rear * 0.5, -center_of_mass_height, -wheelbase * (1.0 - front_weight_bias))   # RR
		_: return Vector3.ZERO

# Placeholder - integrate with your existing wheel force calculation
func _get_wheel_forces(local_velocity: Vector3, dt: float) -> Array:
	# This should return an array of 4 Vector3 forces from your wheel/tire model
	# Each force should be in local car coordinates
	# Use the calculated wheel_loads[i] for proper tire grip calculation
	return [Vector3.ZERO, Vector3.ZERO, Vector3.ZERO, Vector3.ZERO]


func initialize():
	# Check to verify that surface types are provided
	if tire_stiffnesses.size() == 0:
		push_error("No surface types provided for tire stiffness")
		return
	
	if coefficient_of_friction.size() == 0:
		push_error("No surface types provided for coefficient of friction")
		return
	
	if rolling_resistance.size() == 0:
		push_error("No surface types provided in rolling resistance")
		return
	
	if lateral_grip_assist.size() == 0:
		push_error("No surface types provided in lateral grip assist")
		return
	
	if longitudinal_grip_ratio.size() == 0:
		push_error("No surface types provided in longitudinal grip ratio")
		return
	
	var default_surface : String = tire_stiffnesses.keys()[0]
	
	center_of_mass_mode = RigidBody3D.CENTER_OF_MASS_MODE_CUSTOM
	mass = vehicle_mass
	var center_of_gravity := calculate_center_of_gravity(front_weight_distribution)
	center_of_gravity.y += center_of_gravity_height_offset
	center_of_mass = center_of_gravity
	max_clutch_torque = max_torque * max_clutch_torque_ratio
	
	front_axle = Axle.new()
	front_axle.wheels.append(front_left_wheel)
	front_axle.wheels.append(front_right_wheel)
	front_axle.torque_vectoring = front_torque_vectoring
	front_left_wheel.opposite_wheel = front_right_wheel
	front_left_wheel.beam_axle = 1.0 if front_beam_axle else 0.0
	front_right_wheel.opposite_wheel = front_left_wheel
	front_right_wheel.beam_axle = -1.0 if front_beam_axle else 0.0
	rear_axle = Axle.new()
	rear_axle.wheels.append(rear_left_wheel)
	rear_axle.wheels.append(rear_right_wheel)
	rear_axle.torque_vectoring = rear_torque_vectoring
	rear_left_wheel.opposite_wheel = rear_right_wheel
	rear_left_wheel.beam_axle = 1.0 if rear_beam_axle else 0.0
	rear_right_wheel.opposite_wheel = rear_left_wheel
	rear_right_wheel.beam_axle = -1.0 if rear_beam_axle else 0.0
	rear_axle.handbrake = true
	
	axles.append(front_axle)
	axles.append(rear_axle)
	
	wheel_array.append(front_left_wheel)
	wheel_array.append(front_right_wheel)
	wheel_array.append(rear_left_wheel)
	wheel_array.append(rear_right_wheel)
	
	var max_tire_radius := maxf(front_tire_radius, rear_tire_radius)
	front_axle.tire_size_correction = max_tire_radius / front_tire_radius
	rear_axle.tire_size_correction = max_tire_radius / rear_tire_radius
	
	front_axle.differential_lock_torque = front_locking_differential_engage_torque
	rear_axle.differential_lock_torque = rear_locking_differential_engage_torque
	
	for wheel in wheel_array:
		wheel.surface_type = default_surface
		wheel.tire_stiffnesses = tire_stiffnesses
		wheel.contact_patch = contact_patch
		wheel.braking_grip_multiplier = braking_grip_multiplier
		wheel.coefficient_of_friction = coefficient_of_friction
		wheel.rolling_resistance = rolling_resistance
		wheel.lateral_grip_assist = lateral_grip_assist
		wheel.longitudinal_grip_ratio = longitudinal_grip_ratio
		wheel.wheel_to_body_torque_multiplier = wheel_to_body_torque_multiplier
	
	var g = 9.81
	var front_weight_per_wheel = (vehicle_mass * g * front_weight_distribution) / 2.0
	var rear_weight_per_wheel = (vehicle_mass * g * (1.0 - front_weight_distribution)) / 2.0

	var front_spring_rate := calculate_spring_rate(front_weight_per_wheel, front_spring_length, front_resting_ratio)
	var front_damping_rate := calculate_damping(front_weight_per_wheel, front_spring_rate, front_damping_ratio)
	
	for wheel in front_axle.wheels:
		wheel.wheel_mass = front_wheel_mass
		wheel.tire_radius = front_tire_radius
		wheel.tire_width = front_tire_width
		wheel.steering_ratio = front_steering_ratio
		wheel.spring_length = front_spring_length
		wheel.spring_rate = front_spring_rate
		wheel.antiroll = front_spring_rate * front_arb_ratio
		wheel.slow_bump = front_damping_rate * front_bump_damp_multiplier
		wheel.slow_rebound = front_damping_rate * front_rebound_damp_multiplier
		wheel.fast_bump = front_damping_rate * front_bump_damp_multiplier * 0.5
		wheel.fast_rebound = front_damping_rate * front_rebound_damp_multiplier * 0.5
		wheel.bump_stop_multiplier = front_bump_stop_multiplier
		wheel.mass_over_wheel = vehicle_mass * front_weight_distribution * 0.5
		wheel.abs_pulse_time = front_abs_pulse_time
		wheel.abs_spin_difference_threshold = -absf(front_abs_spin_difference_threshold)
		
	
	var rear_spring_rate := calculate_spring_rate(rear_weight_per_wheel, rear_spring_length, rear_resting_ratio)
	var rear_damping_rate := calculate_damping(rear_weight_per_wheel, rear_spring_rate, rear_damping_ratio)
	
	for wheel in rear_axle.wheels:
		wheel.wheel_mass = rear_wheel_mass
		wheel.tire_radius = rear_tire_radius
		wheel.tire_width = rear_tire_width
		wheel.steering_ratio = rear_steering_ratio
		wheel.spring_length = rear_spring_length
		wheel.spring_rate = rear_spring_rate
		wheel.antiroll = rear_spring_rate * rear_arb_ratio
		wheel.slow_bump = rear_damping_rate * rear_bump_damp_multiplier
		wheel.slow_rebound = rear_damping_rate * rear_rebound_damp_multiplier
		wheel.fast_bump = rear_damping_rate * rear_bump_damp_multiplier * 0.5
		wheel.fast_rebound = rear_damping_rate * rear_rebound_damp_multiplier * 0.5
		wheel.bump_stop_multiplier = rear_bump_stop_multiplier
		wheel.mass_over_wheel = vehicle_mass * (1.0 - front_weight_distribution) * 0.5
		wheel.abs_pulse_time = rear_abs_pulse_time
		wheel.abs_spin_difference_threshold = -absf(rear_abs_spin_difference_threshold)
	
	var wheel_base := rear_left_wheel.position.z - front_left_wheel.position.z
	var front_track_width := front_right_wheel.position.x - front_left_wheel.position.x
	var front_ackermann := (atan((wheel_base * tan(max_steering_angle)) / (wheel_base - (front_track_width * 0.5 * tan(max_steering_angle)))) / max_steering_angle) - 1.0
	var rear_track_width := rear_right_wheel.position.x - rear_left_wheel.position.x
	var rear_ackermann := (atan((wheel_base * tan(max_steering_angle)) / (wheel_base - (rear_track_width * 0.5 * tan(max_steering_angle)))) / max_steering_angle) - 1.0
	
	front_left_wheel.ackermann = front_ackermann
	front_left_wheel.rotation.z = -front_camber
	front_left_wheel.toe = -front_toe
	front_right_wheel.ackermann = -front_ackermann
	front_right_wheel.rotation.z = front_camber
	front_right_wheel.toe = front_toe
	rear_left_wheel.ackermann = rear_ackermann
	rear_left_wheel.rotation.z = -rear_camber
	rear_left_wheel.toe = -rear_toe
	rear_right_wheel.ackermann = -rear_ackermann
	rear_right_wheel.rotation.z = rear_camber
	rear_right_wheel.toe = rear_toe
	
	if front_brake_bias < 0.0:
		var front_axle_spring_force := calculate_axle_spring_force(0.6, front_spring_length, front_spring_rate)
		var total_spring_froce := front_axle_spring_force + calculate_axle_spring_force(0.4, rear_spring_length, rear_spring_rate)
		front_brake_bias = front_axle_spring_force / total_spring_froce
	
# Brake bias dynamically calculated
	var front_axle_weight = 0.0
	for wheel in front_axle.wheels:
		front_axle_weight += wheel.mass_over_wheel * 9.81  # gravity

	var rear_axle_weight = 0.0
	for wheel in rear_axle.wheels:
		rear_axle_weight += wheel.mass_over_wheel * 9.81  # gravity

	front_axle.brake_bias = front_axle_weight / (front_axle_weight + rear_axle_weight)
	rear_axle.brake_bias = 1.0 - front_axle.brake_bias

	
	for wheel in wheel_array:
		wheel.initialize()
	
	if front_torque_split > 0.0 or variable_torque_split:
		front_axle.is_drive_axle = true
	if front_torque_split < 1.0 or variable_torque_split:
		rear_axle.is_drive_axle = true
	
	for wheel in front_axle.wheels:
		front_axle.inertia += wheel.wheel_moment
		if front_axle.is_drive_axle:
			drive_axles_inertia += wheel.wheel_moment
			drive_wheels.append(wheel)
			wheel.is_driven = true
			average_drive_wheel_radius += wheel.tire_radius
	
	for wheel in rear_axle.wheels:
		rear_axle.inertia += wheel.wheel_moment
		if rear_axle.is_drive_axle:
			drive_axles_inertia += wheel.wheel_moment
			drive_wheels.append(wheel)
			wheel.is_driven = true
			average_drive_wheel_radius += wheel.tire_radius
	
	average_drive_wheel_radius /= drive_wheels.size()
	previous_global_position = global_position
	
	calculate_brake_force()
	
	is_ready = true

func _physics_process(delta: float) -> void:
	if speed > 15.0 and current_gear == -1:
		speed == -15.0
	if not is_ready:
		return

	# Get direct state of the rigid body for inertia
	var direct_state := PhysicsServer3D.body_get_direct_state(get_rid())
	if direct_state:
		if not vehicle_inertia:
			# Vehicle inertia as Vector3 for each axis
			var inv_inertia := direct_state.inverse_inertia
			if inv_inertia.x > 0 and inv_inertia.y > 0 and inv_inertia.z > 0:
				vehicle_inertia = Vector3(1.0 / inv_inertia.x, 1.0 / inv_inertia.y, 1.0 / inv_inertia.z) * inertia_multiplier
				inertia = vehicle_inertia

	delta_time += delta

	# Transform global velocity into local vehicle space
	var global_velocity := (global_transform.origin - previous_global_position) / delta
	local_velocity = lerp(global_transform.basis.inverse() * global_velocity, local_velocity, 0.5)
	previous_global_position = global_transform.origin
	speed = local_velocity.length()

	# Calculate longitudinal and lateral accelerations
	var new_longitudinal_accel := (local_velocity.z - previous_local_velocity.z) / delta
	var new_lateral_accel := (local_velocity.x - previous_local_velocity.x) / delta
	longitudinal_accel = lerp(longitudinal_accel, new_longitudinal_accel, 0.5)
	lateral_accel = lerp(lateral_accel, new_lateral_accel, 0.5)
	previous_local_velocity = local_velocity

	# Process vehicle systems
	process_drag()
	process_braking(delta)
	process_steering(delta)
	process_throttle(delta)
	process_motor(delta)
	process_clutch(delta)
	process_transmission(delta)
	process_drive(delta)
	process_forces(delta)
	process_stability(delta)

# Put these at the top of your vehicle script so you can tune them:





# Enhanced drag variables (add these to your class if not present)



func process_drag() -> void:
	if speed <= MIN_SPEED_THRESHOLD:
		return
	
	var velocity_squared: float = speed * speed
	var air_density_corrected: float = air_density * ALTITUDE_FACTOR * cooling_drag_factor
	
	# Get velocity components in local space
	var longitudinal_speed: float = local_velocity.z
	var lateral_speed: float = abs(local_velocity.x)
	
	# === AERODYNAMIC DRAG ===
	var aero_drag: Vector3 = calculate_aerodynamic_drag(velocity_squared, air_density_corrected)
	
	# === ROLLING RESISTANCE ===
	var rolling_drag: Vector3 = calculate_rolling_resistance()
	
	# === LATERAL DRAG ===
	var lateral_drag: Vector3 = calculate_lateral_drag(lateral_speed, air_density_corrected)
	
	# === DRIVETRAIN LOSSES ===
	var drivetrain_drag: Vector3 = calculate_drivetrain_drag(longitudinal_speed)
	
	# === BRAKE DRAG ===
	var brake_drag: Vector3 = calculate_brake_drag()
	
	# Apply all drag forces
	var total_drag: Vector3 = aero_drag + rolling_drag + lateral_drag + drivetrain_drag + brake_drag
	apply_central_force(total_drag)
	
	# Apply downforce if present
	if abs(downforce_coefficient) > 0.001:
		apply_downforce(velocity_squared, air_density_corrected)

func calculate_aerodynamic_drag(v_squared: float, air_density: float) -> Vector3:
	# More accurate drag calculation considering local velocity direction
	var drag_magnitude: float = 0.5 * air_density * coefficient_of_drag * frontal_area * v_squared
	
	# Apply drag opposite to velocity direction in local space
	var local_velocity_normalized: Vector3 = local_velocity.normalized()
	var drag_force_local: Vector3 = -local_velocity_normalized * drag_magnitude
	
	# Weight drag components based on actual velocity distribution
	var forward_component: float = abs(local_velocity.z) / speed
	var lateral_component: float = abs(local_velocity.x) / speed
	
	# Forward drag (most significant)
	drag_force_local.z *= forward_component
	# Lateral drag (less efficient, higher drag per unit speed)
	drag_force_local.x *= lateral_component * 1.3  # 30% penalty for lateral movement
	
	return drag_force_local

func calculate_rolling_resistance() -> Vector3:
	if speed < MIN_SPEED_THRESHOLD:
		return Vector3.ZERO
	
	# Calculate normal force per wheel (simplified)
	var gravity: float = ProjectSettings.get_setting("physics/3d/default_gravity", 9.81)
	var base_normal_force: float = mass * gravity * 0.25  # Divided by 4 wheels
	
	# Dynamic load distribution affects rolling resistance
	var total_rolling_force: float = 0.0
	
	for i in range(4):
		# Use actual wheel loads if available (from previous enhanced physics)
		var wheel_normal_force: float = base_normal_force
		if wheel_loads.size() > i:
			wheel_normal_force = wheel_loads[i]
		
		# Rolling resistance increases with load and tire heat
		var rr_coefficient: float = tire_rolling_resistance[i] * TIRE_HEAT_FACTOR
		total_rolling_force += rr_coefficient * wheel_normal_force
	
	# Apply rolling resistance opposite to movement direction
	var velocity_direction: Vector3 = local_velocity.normalized()
	return -velocity_direction * total_rolling_force

func calculate_lateral_drag(lateral_speed: float, air_density: float) -> Vector3:
	if lateral_speed < MIN_SPEED_THRESHOLD:
		return Vector3.ZERO
	
	# Lateral drag is less efficient than forward airflow
	var side_drag_coefficient: float = coefficient_of_drag * 1.8  # Higher Cd for side profile
	var lateral_drag_magnitude: float = 0.5 * air_density * side_drag_coefficient * side_area * lateral_speed * lateral_speed
	
	# Apply lateral drag in world space (perpendicular to car)
	var lateral_direction: Vector3 = -sign(local_velocity.x) * Vector3.RIGHT
	return global_transform.basis * (lateral_direction * lateral_drag_magnitude)

func calculate_drivetrain_drag(longitudinal_speed: float) -> Vector3:
	if abs(longitudinal_speed) < MIN_SPEED_THRESHOLD:
		return Vector3.ZERO
	
	# More realistic drivetrain losses
	var base_friction: float = 15.0  # Base mechanical friction
	var speed_dependent: float = 0.8 * abs(longitudinal_speed)  # Increases with speed
	var gear_efficiency_loss: float = 5.0  # Additional losses in transmission
	
	# Engine braking effect (higher at high RPM, low throttle)
	var engine_braking: float = 0.0
	if longitudinal_speed > 0 and has_method("get_throttle_input"):
		var throttle: float = get_throttle_input() if has_method("get_throttle_input") else 0.5
		engine_braking = (1.0 - throttle) * 20.0 * (longitudinal_speed / 30.0)  # Scale with speed
	
	var total_drivetrain_loss: float = base_friction + speed_dependent + gear_efficiency_loss + engine_braking
	
	# Apply opposite to forward motion
	return Vector3(0, 0, -sign(longitudinal_speed) * total_drivetrain_loss)

func calculate_brake_drag() -> Vector3:
	if brake_drag_coefficient <= 0.0:
		return Vector3.ZERO
	
	# Additional aerodynamic drag from brake cooling, open vents, etc.
	var brake_aero_drag: float = brake_drag_coefficient * speed * speed * 0.1
	return -local_velocity.normalized() * brake_aero_drag

# Enhanced downforce variables (add these to your class)


func apply_downforce(v_squared: float, air_density: float) -> void:
	if v_squared < 1.0:  # Negligible at very low speeds
		return
	
	# Calculate total downforce components
	var front_downforce: float = calculate_front_downforce(v_squared, air_density)
	var rear_downforce: float = calculate_rear_downforce(v_squared, air_density)
	var ground_effect_downforce: float = calculate_ground_effect(v_squared, air_density)
	
	var total_downforce: float = front_downforce + rear_downforce + ground_effect_downforce
	
	# Apply aerodynamic pitch moment (realistic handling balance)
	apply_aerodynamic_moments(front_downforce, rear_downforce, v_squared)
	
	# Apply downforce at correct center of pressure
	apply_downforce_at_cop(total_downforce)
	
	# Distribute load to wheels realistically
	distribute_aerodynamic_load(front_downforce, rear_downforce, ground_effect_downforce)

func calculate_front_downforce(v_squared: float, air_density: float) -> float:
	var base_front_force: float = 0.5 * air_density * front_downforce_coefficient * frontal_area * v_squared * front_aero_balance
	
	# Wing stall effect at high angles of attack (simplified)
	var pitch_angle: float = get_pitch_angle()  # You'll need to implement this
	var stall_factor: float = 1.0
	if abs(pitch_angle) > deg_to_rad(stall_angle):
		stall_factor = maxf(0.3, 1.0 - (abs(pitch_angle) - deg_to_rad(stall_angle)) * 2.0)
	
	return base_front_force * stall_factor

func calculate_rear_downforce(v_squared: float, air_density: float) -> float:
	var base_rear_force: float = 0.5 * air_density * rear_downforce_coefficient * frontal_area * v_squared * (1.0 - front_aero_balance)
	
	# Rear wing is typically less affected by pitch but more by yaw
	var yaw_angle: float = get_yaw_slip_angle()  # Side slip angle
	var yaw_efficiency: float = maxf(0.8, 1.0 - abs(yaw_angle) * 0.5)  # Slight loss in yaw
	
	return base_rear_force * yaw_efficiency

func calculate_ground_effect(v_squared: float, air_density: float) -> float:
	var current_ride_height: float = get_average_ride_height()  # You'll need to implement this
	
	# Ground effect peaks at optimal ride height, falls off quickly when too low or high
	var height_efficiency: float = 1.0
	if current_ride_height < ride_height_optimal:
		# Too low - venturi effect breaks down
		var height_ratio: float = current_ride_height / ride_height_optimal
		height_efficiency = smoothstep(0.0, 1.0, height_ratio)
	else:
		# Too high - ground effect diminishes
		var excess_height: float = current_ride_height - ride_height_optimal
		height_efficiency = maxf(0.1, 1.0 - excess_height * 3.0)  # Falls off quickly
	
	var underbody_force: float = 0.5 * air_density * underbody_downforce * frontal_area * v_squared
	return underbody_force * height_efficiency * ground_effect_strength

func apply_aerodynamic_moments(front_force: float, rear_force: float, v_squared: float) -> void:
	# Calculate pitch moment from aerodynamic imbalance
	var aero_imbalance: float = rear_force - front_force
	var pitch_moment_arm: float = wheelbase * 0.5  # Distance from center of mass
	var pitch_torque: float = aero_imbalance * pitch_moment_arm * 0.1  # Scale factor
	
	# Apply pitch moment (affects handling balance)
	var pitch_torque_vector: Vector3 = Vector3(pitch_torque, 0, 0)
	apply_torque(pitch_torque_vector)
	
	# Yaw stability from rear downforce (acts like a vertical stabilizer)
	var yaw_angle: float = get_yaw_slip_angle()
	if abs(yaw_angle) > deg_to_rad(2.0):  # Only above small threshold
		var stability_torque: float = -sign(yaw_angle) * rear_force * 0.05 * abs(yaw_angle)
		var yaw_torque_vector: Vector3 = Vector3(0, stability_torque, 0)
		apply_torque(yaw_torque_vector)

func apply_downforce_at_cop(total_downforce: float) -> void:
	# Apply downforce at center of pressure, not center of mass
	var cop_position: Vector3 = Vector3(0, 0, center_of_pressure_offset)  # Local space
	var downforce_vector: Vector3 = Vector3(0, -total_downforce, 0)
	
	# Convert to world space and apply
	var world_cop_position: Vector3 = global_transform.basis * cop_position
	apply_force(downforce_vector, world_cop_position)

func distribute_aerodynamic_load(front_force: float, rear_force: float, ground_effect: float) -> void:
	if wheel_loads.size() < 4:
		return
	
	# Distribute front downforce to front wheels (50/50)
	var front_load_per_wheel: float = front_force * 0.5
	wheel_loads[0] += front_load_per_wheel  # FL
	wheel_loads[1] += front_load_per_wheel  # FR
	
	# Distribute rear downforce to rear wheels (50/50)  
	var rear_load_per_wheel: float = rear_force * 0.5
	wheel_loads[2] += rear_load_per_wheel   # RL
	wheel_loads[3] += rear_load_per_wheel   # RR
	
	# Ground effect affects all wheels but more at the rear (diffuser effect)
	var ground_front: float = ground_effect * 0.3
	var ground_rear: float = ground_effect * 0.7
	
	wheel_loads[0] += ground_front * 0.5
	wheel_loads[1] += ground_front * 0.5
	wheel_loads[2] += ground_rear * 0.5
	wheel_loads[3] += ground_rear * 0.5

# Helper functions - you'll need to implement these based on your car setup
func get_pitch_angle() -> float:
	# Calculate current pitch angle of the car
	# This is the rotation around the X-axis (nose up/down)
	var forward: Vector3 = global_transform.basis.z
	return asin(-forward.y)

func get_yaw_slip_angle() -> float:
	# Calculate slip angle (difference between heading and velocity direction)
	if speed < 1.0:
		return 0.0
	
	var velocity_global: Vector3 = linear_velocity if has_method("get") else Vector3.ZERO
	if velocity_global.length() < 1.0:
		return 0.0
	
	var heading: Vector3 = global_transform.basis.z
	var velocity_normalized: Vector3 = velocity_global.normalized()
	
	# Calculate angle between heading and velocity
	var dot_product: float = heading.dot(velocity_normalized)
	var cross_product: Vector3 = heading.cross(velocity_normalized)
	
	return atan2(cross_product.y, dot_product)

func get_average_ride_height() -> float:
	# Calculate average distance from ground
	# This is a simplified version - you might want to use actual raycast distances
	var space_state: PhysicsDirectSpaceState3D = get_world_3d().direct_space_state
	var from: Vector3 = global_position
	var to: Vector3 = global_position + Vector3(0, -2.0, 0)
	
	var query: PhysicsRayQueryParameters3D = PhysicsRayQueryParameters3D.create(from, to)
	var result: Dictionary = space_state.intersect_ray(query)
	
	if result.has("position"):
		return (global_position - result.position).length()
	
	return 1.0  # Default if no ground detected
# Helper function to get current brake input (implement based on your input system)
# Enhanced input variables (add these to your class)


func get_throttle_input() -> float:
	# Get raw input from your input system
	update_raw_throttle_input()
	
	# Apply deadzone
	var processed_input: float = apply_deadzone(raw_throttle_input, throttle_deadzone)
	
	# Apply throttle curve for more realistic feel
	processed_input = apply_throttle_curve(processed_input)
	
	# Rate limiting (prevents unrealistic instant changes)
	processed_input = apply_rate_limiting(processed_input, filtered_throttle, max_input_rate, get_physics_process_delta_time())
	
	# Smooth filtering
	filtered_throttle = lerpf(filtered_throttle, processed_input, input_filter_strength)
	
	# Update input history for advanced features
	update_input_history(throttle_history, filtered_throttle)
	
	return clampf(filtered_throttle, 0.0, 1.0)

func get_brake_input() -> float:
	# Get raw input from your input system  
	update_raw_brake_input()
	
	# Apply deadzone
	var processed_input: float = apply_deadzone(raw_brake_input, brake_deadzone)
	
	# Apply brake pressure curve (more realistic brake feel)
	processed_input = apply_brake_curve(processed_input)
	
	# Trail braking assistance (helps with smooth brake release)
	if trail_braking_assist > 0.0:
		processed_input = apply_trail_braking_assist(processed_input)
	
	# Rate limiting
	processed_input = apply_rate_limiting(processed_input, filtered_brake, max_input_rate, get_physics_process_delta_time())
	
	# Smooth filtering
	filtered_brake = lerpf(filtered_brake, processed_input, input_filter_strength)
	
	# Update input history
	update_input_history(brake_history, filtered_brake)
	
	return clampf(filtered_brake, 0.0, 1.0)

func update_raw_throttle_input() -> void:
	# Replace this with your actual input system
	# Examples for different input methods:
	
	# Keyboard input (binary)
	if Input.is_action_pressed("Throttle"):
		raw_throttle_input = 1.0
	elif Input.is_action_pressed("Brakes"):
		raw_throttle_input = -1.0  # For reverse gear
	else:
		raw_throttle_input = 0.0
	
	# Gamepad analog trigger (recommended)
	raw_throttle_input = Input.get_action_strength("Throttle")
	raw_throttle_input = Input.get_action_strength("Brakes")
	
	# Mouse/wheel input with accumulation
	if Input.is_action_just_pressed("Throttle"):
		raw_throttle_input = minf(raw_throttle_input + 0.1, 1.0)
	if Input.is_action_just_pressed("Brakes"):
		raw_throttle_input = maxf(raw_throttle_input - 0.1, 0.0)

func update_raw_brake_input() -> void:
	# Replace this with your actual input system
	
	# Keyboard input
	if Input.is_action_pressed("Brakes"):
		raw_brake_input = 1.0
	else:
		raw_brake_input = 0.0
	
	# Gamepad analog trigger (recommended)  
	# raw_brake_input = Input.get_action_strength("brake")

func apply_deadzone(input_value: float, deadzone: float) -> float:
	var abs_input: float = abs(input_value)
	if abs_input < deadzone:
		return 0.0
	
	# Scale remaining input to full range
	var sign_input: float = sign(input_value)
	return sign_input * ((abs_input - deadzone) / (1.0 - deadzone))

func apply_throttle_curve(input: float) -> float:
	if input <= 0.0:
		return input  # Preserve reverse input linearly
	
	# Progressive throttle curve - more sensitive at low inputs
	return pow(input, 1.0 / throttle_sensitivity)

func apply_brake_curve(input: float) -> float:
	if input <= 0.0:
		return 0.0
	
	# Brake pressure curve - initial pressure builds quickly, then levels off
	return pow(input, brake_pressure_curve)

func apply_trail_braking_assist(current_input: float) -> float:
	if brake_history.size() < 3:
		return current_input
	
	# Detect if driver is releasing brakes
	var recent_brake: float = brake_history[-1] if brake_history.size() > 0 else 0.0
	var is_releasing: bool = current_input < recent_brake
	
	if is_releasing and recent_brake > 0.3:  # Only assist during significant braking
		# Slightly smooth brake release to help with trail braking
		var assist_factor: float = trail_braking_assist * 0.5
		return lerpf(current_input, recent_brake, assist_factor)
	
	return current_input

func apply_rate_limiting(target: float, current: float, max_rate: float, delta: float) -> float:
	var max_change: float = max_rate * delta
	var desired_change: float = target - current
	
	if abs(desired_change) > max_change:
		return current + sign(desired_change) * max_change
	
	return target

func update_input_history(history: Array[float], value: float) -> void:
	history.append(value)
	
	# Keep only recent history (last 10 frames)
	if history.size() > 10:
		history.pop_front()

# Advanced input analysis functions
func get_throttle_smoothness() -> float:
	# Returns measure of how smooth throttle inputs are (0-1, higher = smoother)
	if throttle_history.size() < 3:
		return 1.0
	
	var total_variation: float = 0.0
	for i in range(1, throttle_history.size()):
		total_variation += abs(throttle_history[i] - throttle_history[i-1])
	
	var average_variation: float = total_variation / float(throttle_history.size() - 1)
	return clampf(1.0 - average_variation * 10.0, 0.0, 1.0)

func get_brake_smoothness() -> float:
	# Returns measure of how smooth brake inputs are
	if brake_history.size() < 3:
		return 1.0
	
	var total_variation: float = 0.0
	for i in range(1, brake_history.size()):
		total_variation += abs(brake_history[i] - brake_history[i-1])
	
	var average_variation: float = total_variation / float(brake_history.size() - 1)
	return clampf(1.0 - average_variation * 10.0, 0.0, 1.0)

func is_left_foot_braking() -> bool:
	# Detect if driver is using both throttle and brake simultaneously
	return get_throttle_input() > 0.1 and get_brake_input() > 0.1

func get_pedal_overlap() -> float:
	# Returns amount of throttle/brake overlap (for advanced driving analysis)
	var throttle: float = get_throttle_input()
	var brake: float = get_brake_input()
	return minf(throttle, brake)

# Configuration functions for different driving styles
func set_arcade_mode() -> void:
	# More forgiving, responsive controls
	throttle_sensitivity = 1.0
	brake_sensitivity = 1.0
	input_filter_strength = 0.25
	max_input_rate = 15.0
	trail_braking_assist = 0.3

func set_simulation_mode() -> void:
	# Realistic, challenging controls
	throttle_sensitivity = 2.5
	brake_sensitivity = 2.0
	input_filter_strength = 0.05
	max_input_rate = 6.0
	trail_braking_assist = 0.0

func set_assist_mode() -> void:
	# Assisted controls for beginners
	throttle_sensitivity = 1.5
	brake_sensitivity = 1.2
	input_filter_strength = 0.2
	max_input_rate = 10.0
	trail_braking_assist = 0.5
# Add these variables to your car class (optional enhancements)


func process_braking(delta: float) -> void:
	# Smooth inputs (same as original)
	var in_b := clamp(brake_input, 0.0, 1.0)
	var in_hb := clamp(handbrake_input, 0.0, 1.0)
	var lerp_rate := clamp(braking_speed * delta, 0.0, 1.0)
	brake_amount = lerp(brake_amount, in_b, lerp_rate)
	is_braking = (brake_amount > 0.001) or (in_hb > 0.001)
	
	# Base brake force
	var bf := brake_amount * max_brake_force
	
	# Realistic weight transfer during braking (subtle improvement)
	var deceleration_factor := clamp(brake_amount * 0.3, 0.0, 0.2)  # Max 20% transfer
	var front_bias :float= brake_balance + deceleration_factor
	front_bias = clamp(front_bias, 0.5, 0.8)
	
	# Apply original speed tapers (keep your working physics)
	if speed < 4.0:
		bf *= speed / 4.0
	
	var hi_speed_taper := 1.0
	if speed > 80.0:
		hi_speed_taper = clamp(1.0 - (speed - 80.0) / 120.0, 0.6, 1.0)
	bf *= hi_speed_taper
	
	# Keep your original braking scale - this is crucial!
	var braking_scale := 0.0002
	bf *= braking_scale
	
	# Simple brake temperature effect (optional realism)
	update_brake_temperatures_simple(delta, bf)
	var temp_multiplier := get_simple_temp_multiplier()
	bf *= temp_multiplier
	
	brake_force = bf
	
	# Handbrake (keep original logic)
	var hf: float = in_hb * max_handbrake_force
	if speed > 10.0:
		var hb_taper := clamp(1.0 - (speed - 10.0) / 90.0, 0.25, 1.0)
		hf *= hb_taper
	handbrake_force = hf

# Simplified brake temperature (optional - can be disabled)
func update_brake_temperatures_simple(delta: float, brake_force_applied: float) -> void:
	var heat_input := (brake_force_applied / (max_brake_force * 0.0002)) * 50.0 * delta
	brake_temp_front += heat_input
	brake_temp_rear += heat_input
	
	# Cool down
	var cooling := 20.0 * (1.0 + speed * 0.01) * delta
	brake_temp_front = max(20.0, brake_temp_front - cooling)
	brake_temp_rear = max(20.0, brake_temp_rear - cooling)

func get_simple_temp_multiplier() -> float:
	var avg_temp := (brake_temp_front + brake_temp_rear) * 0.5
	if avg_temp < 100.0:
		return 1.0
	elif avg_temp < 300.0:
		return 1.0 + (avg_temp - 100.0) * 0.0005  # Slight improvement when warm
	else:
		# Brake fade at high temps
		return max(0.7, 1.1 - (avg_temp - 300.0) * 0.001)

# Add these optional variables for enhanced realism (keep minimal)



func process_steering(delta: float) -> void:
	# --- Smooth steering input (keep your working approach) ---
	var target_steering: float = steering_input
	
	# Enhanced speed-sensitive steering with power steering simulation
	var base_speed_factor: float = clamp(1.0 / (1.0 + (speed * 0.05)), 0.3, 1.0)
	
	# Realistic power steering assistance (more help at low speeds)
	var power_assist_factor: float = power_steering_assist * (1.0 - clamp(speed * 0.01, 0.0, 0.7))
	var combined_speed_factor: float = base_speed_factor * (1.0 + power_assist_factor)
	
	var steer_speed_factor: float = (steering_speed * combined_speed_factor) / steering_speed_decay
	
	# If player reverses steering direction, soften countersteer (keep original logic)
	if signf(target_steering) != signf(steering_amount):
		steer_speed_factor *= 0.7
	
	# Add realistic steering friction/resistance
	var friction_resistance: float = 1.0 - (steering_friction * clamp(speed * 0.02, 0.0, 0.3))
	steer_speed_factor *= friction_resistance
	
	# Smooth steering response with better centering (keep original)
	steering_amount = lerp(steering_amount, target_steering, steer_speed_factor * delta)
	
	# Enhanced centering with load-sensitive return force
	if abs(target_steering) < 0.01 and abs(steering_amount) < 0.05:
		var centering_strength: float = 3.0 + (speed * 0.05)  # Stronger centering at speed
		steering_amount = lerp(steering_amount, 0.0, delta * centering_strength)
	
	# Zero out tiny values (keep original)
	if abs(steering_amount) < 0.01:
		steering_amount = 0.0
	
	# --- Steering sensitivity curve (keep original) ---
	var steering_adjust: float = pow(absf(steering_amount), steering_exponent) * signf(steering_amount)
	
	# --- Enhanced self-aligning torque with load sensitivity ---
	var load_factor: float = front_load_factor + clamp((speed * speed) * 0.0001, 0.0, 0.2)  # More load at speed
	var return_strength: float = clamp(speed * 0.015 * load_factor, 0.0, 0.8)
	steering_adjust = lerp(steering_adjust, 0.0, return_strength * delta)
	
	# --- Enhanced countersteer assist (keep your working logic but improve) ---
	var lateral_speed: float = local_velocity.x
	var steer_correction: float = 0.0
	
	if absf(lateral_speed) > steering_slip_assist and speed > 5.0 and abs(target_steering) > 0.02:
		var travel_angle: float = clamp(asin(lateral_speed / max(speed, 0.01)), -1.0, 1.0)
		
		# Enhanced countersteer with vehicle dynamics consideration
		var yaw_rate: float = get_angular_velocity().y if has_method("get_angular_velocity") else 0.0
		var yaw_factor: float = clamp(abs(yaw_rate) * 2.0, 0.5, 1.5)  # Adjust based on rotation
		
		steer_correction = (1.0 - absf(steering_adjust)) * travel_angle * countersteer_assist * 0.5 * yaw_factor
	
	# --- Realistic dynamic max steering angle with load effects ---
	var min_angle: float = deg_to_rad(4.0)  # Slightly less at high speed
	var max_angle: float = deg_to_rad(28.0)  # Slightly more at low speed
	
	# Factor in front axle loading (more load = less max steering)
	var load_steering_reduction: float = 1.0 - (load_factor - 0.6) * 0.3
	max_angle *= load_steering_reduction
	min_angle *= load_steering_reduction
	
	var dynamic_max_angle: float = lerp(max_angle, min_angle, clamp(speed / 200.0, 0.0, 1.0))
	
	# --- Final steering (keep your working logic exactly) ---
	true_steering_amount = clamp(steering_adjust + steer_correction, -dynamic_max_angle, dynamic_max_angle)
	
	# Apply to wheels (keep your working approach - no Ackermann complexity)
	for wheel in wheel_array:
		wheel.steer(true_steering_amount, dynamic_max_angle, max_steering_angle)
# Add these variables for enhanced engine realism


func process_throttle(delta: float) -> void:
	# --- Enhanced pedal response with realistic throttle body simulation ---
	var pedal_rate_up := 2.8  # Slightly slower for realism
	var pedal_rate_down := 4.5  # Faster release for safety
	var target_pedal := throttle_input
	
	# Apply pedal filtering
	if target_pedal > throttle_amount:
		throttle_amount = lerp(throttle_amount, target_pedal, pedal_rate_up * delta)
	else:
		throttle_amount = lerp(throttle_amount, target_pedal, pedal_rate_down * delta)
	
	# --- Realistic throttle body response (mechanical lag) ---
	var throttle_body_response := 12.0  # Throttle body actuator speed
	throttle_body_position = lerp(throttle_body_position, throttle_amount, throttle_body_response * delta)
	
	# --- Engine vacuum and manifold pressure simulation ---
	update_manifold_pressure(delta, throttle_body_position)
	
	# --- Enhanced engine response with intake dynamics ---
	var base_engine_response := 7.0
	var vacuum_response_modifier := 1.0 + (engine_vacuum * 0.5)  # Vacuum affects response
	var engine_response := base_engine_response * vacuum_response_modifier
	
	var effective_throttle := lerp(previous_throttle, throttle_body_position, engine_response * delta)
	previous_throttle = effective_throttle
	
	# --- Advanced engine load calculation ---
	var rpm_load_factor := calculate_rpm_load_factor()
	var temperature_factor := calculate_temperature_factor()
	var air_density_factor := calculate_air_density_factor()
	
	load_factor = rpm_load_factor * temperature_factor * air_density_factor
	effective_throttle *= load_factor
	
	# --- Enhanced rev-matching with engine inertia ---
	if current_gear < last_gear and current_gear > 0:
		var gear_ratio_new: float = gear_ratios[current_gear]
		var gear_ratio_old: float = gear_ratios[last_gear]
		var predicted_rpm: float = motor_rpm * (gear_ratio_new / gear_ratio_old)
		
		var rpm_diff: float = clamp((predicted_rpm - motor_rpm) / max_rpm, 0.0, 1.0)
		
		# More sophisticated blip calculation
		var engine_inertia_factor := 1.0 + (engine_inertia * 2.0)
		blip_timer = (0.12 + (rpm_diff * 0.08)) * engine_inertia_factor
		blip_strength = 0.35 + (rpm_diff * 0.45)
		
		# Add slight fuel cut before blip for realism
		fuel_cut_active = true
		fuel_cut_timer = 0.05
	
	last_gear = current_gear
	
	# --- Fuel cut management ---
	if fuel_cut_active:
		fuel_cut_timer -= delta
		if fuel_cut_timer <= 0.0:
			fuel_cut_active = false
	
	# --- Overrun fuel cut (engine braking efficiency) ---
	update_overrun_fuel_cut(effective_throttle)
	
	# Apply enhanced throttle blip
	if blip_timer > 0.0:
		blip_timer -= delta
		var blip_curve := sin((1.0 - (blip_timer / 0.2)) * PI)  # Smooth blip curve
		var current_blip_strength := blip_strength * blip_curve
		effective_throttle = max(effective_throttle, current_blip_strength)
	
	# --- Enhanced redline protection ---
	var redline_soft_limit := max_rpm * 0.95  # Start soft limiting at 95%
	var redline_protection := 1.0
	
	if motor_rpm > redline_soft_limit:
		var over_limit_factor := (motor_rpm - redline_soft_limit) / (max_rpm - redline_soft_limit)
		redline_protection = 1.0 - clamp(over_limit_factor, 0.0, 1.0)
	
	if motor_is_redline or is_shifting or fuel_cut_active:
		effective_throttle = 0.0
	else:
		effective_throttle *= redline_protection
	
	# --- Realistic clutch behavior with slip simulation ---
	var clutch_target := clutch_input
	var clutch_response_rate := 4.5
	
	if need_clutch or is_shifting:
		clutch_target = 1.0
		clutch_response_rate = 6.0  # Faster engagement for shifts
		
		# Realistic power reduction during clutch slip
		var slip_factor := clamp(clutch_amount, 0.0, 1.0)
		var power_transmission :float= 1.0 - (slip_factor * 0.85)  # Some power still transmits
		effective_throttle *= power_transmission
	
	clutch_amount = lerp(clutch_amount, clutch_target, clutch_response_rate * delta)
	
	# --- Final throttle with overrun fuel cut ---
	if overrun_fuel_cut:
		effective_throttle *= 0.1  # Minimal fuel during overrun
	
	true_throttle = clamp(effective_throttle, 0.0, 1.0)
	
	# Update engine temperature (simplified)
	update_engine_temperature(delta, effective_throttle)

# Enhanced helper functions for realistic engine behavior
func update_manifold_pressure(delta: float, throttle_pos: float) -> void:
	# Simulate intake manifold pressure dynamics
	var target_pressure := 0.3 + (throttle_pos * 0.7)  # Vacuum to atmospheric
	manifold_pressure = lerp(manifold_pressure, target_pressure, VACUUM_RESPONSE_RATE * delta)
	
	# Calculate engine vacuum (higher at closed throttle)
	engine_vacuum = 1.0 - manifold_pressure

func calculate_rpm_load_factor() -> float:
	# More realistic RPM-based load curve
	var rpm_ratio := motor_rpm / max_rpm
	
	if rpm_ratio < 0.2:
		# Poor low-end torque
		return 0.4 + (rpm_ratio * 2.0)
	elif rpm_ratio < 0.6:
		# Power band
		return 1.0 + (sin((rpm_ratio - 0.2) * PI * 1.25) * 0.2)
	else:
		# High RPM falloff
		return 1.2 - ((rpm_ratio - 0.6) * 1.5)

func calculate_temperature_factor() -> float:
	# Engine temperature affects performance
	if engine_temp < 60.0:
		# Cold engine, reduced power
		return 0.75 + ((engine_temp - 20.0) / 40.0) * 0.25
	elif engine_temp > 105.0:
		# Overheating protection
		var overheat_factor := clamp((engine_temp - 105.0) / 20.0, 0.0, 0.4)
		return 1.0 - overheat_factor
	else:
		# Optimal temperature range
		return 1.0

func calculate_air_density_factor() -> float:
	# Air temperature affects performance (colder air = more power)
	var temp_factor := 1.0 - ((intake_air_temp - 15.0) * 0.002)  # 0.2% per degree above 15°C
	return clamp(temp_factor, 0.85, 1.15)

func update_overrun_fuel_cut(throttle_pos: float) -> void:
	# Realistic overrun fuel cut for engine braking
	var rpm_threshold := OVERRUN_RPM_THRESHOLD
	var throttle_threshold := 0.05
	
	if motor_rpm > rpm_threshold and throttle_pos < throttle_threshold and speed > 10.0:
		overrun_fuel_cut = true
	elif throttle_pos > throttle_threshold or motor_rpm < (rpm_threshold * 0.8):
		overrun_fuel_cut = false

func update_engine_temperature(delta: float, throttle_level: float) -> void:
	# Simplified engine temperature simulation
	var heat_production := throttle_level * 2.0 + 0.5  # Base heat + throttle heat
	var cooling_rate := 1.5 + (speed * 0.02)  # Airflow cooling
	
	var temp_change := (heat_production - cooling_rate) * delta * 0.5
	engine_temp = clamp(engine_temp + temp_change, 20.0, 130.0)  # Realistic temp range

# Add these variables for enhanced engine realism


func process_motor(delta: float) -> void:
	# --- Enhanced throttle response with realistic characteristics ---
	var target_throttle := clamp(throttle_input, 0.0, 1.0)
	
	# Throttle response varies with RPM (slower at low RPM due to intake dynamics)
	var rpm_response_factor := clamp(0.4 + (motor_rpm / max_rpm) * 0.6, 0.4, 1.0)
	var throttle_response_rate :float= 5.0 * rpm_response_factor
	var max_change := throttle_response_rate * delta
	
	throttle_amount = clamp(throttle_amount + clamp(target_throttle - throttle_amount, -max_change, max_change), 0.0, 1.0)
	
	# --- Enhanced clutch simulation with slip modeling ---
	var clutch_target := clutch_input
	var clutch_engagement_rate := 5.0
	
	if need_clutch or is_shifting:
		clutch_target = 1.0
		clutch_engagement_rate = 8.0  # Faster for shifts
	
	clutch_amount = lerp(clutch_amount, clutch_target, clutch_engagement_rate * delta)
	
	# Realistic clutch slip characteristics
	var clutch_slip := clamp(clutch_amount, 0.0, 1.0)
	var clutch_factor := calculate_clutch_transmission_factor(clutch_slip, torque_output)
	
	# --- Advanced engine torque calculation ---
	var rpm_ratio := motor_rpm / max_rpm
	# Temporary test - flatter power curve
	var base_torque_curve = 1.0 - (rpm_ratio * 0.2)  # Only lose 20% at redline instead of more
	
	# Apply throttle with realistic airflow characteristics
	var airflow_factor := calculate_airflow_factor(throttle_amount, motor_rpm)
	var fuel_factor := calculate_fuel_efficiency(rpm_ratio)
	
	# Get base torque and apply realistic factors
	var base_torque :float= get_torque_at_rpm(motor_rpm) * base_torque_curve
	base_torque *= airflow_factor * fuel_factor * throttle_amount
	
	# Apply temperature effects (using your existing function)
	var temp_factor := calculate_temperature_factor()
	base_torque *= temp_factor
	
	# Engine knock protection
	var knock_factor := calculate_knock_protection(rpm_ratio, throttle_amount)
	base_torque *= knock_factor
	
	# Valve float effects at high RPM
	if valve_float_rpm > 0.0 and motor_rpm > valve_float_rpm:
		var float_factor = 1.0 - clamp((motor_rpm - valve_float_rpm) / (max_rpm - valve_float_rpm), 0.0, 0.2)
		base_torque *= float_factor
	
	# --- Enhanced engine braking and friction ---
	var pumping_losses := calculate_pumping_losses(throttle_amount, motor_rpm)
	var mechanical_friction := calculate_mechanical_friction(motor_rpm)
	var total_drag := pumping_losses + mechanical_friction
	
	# --- Rev limiter with realistic behavior ---
	var rev_limit_factor := 1.0
	if motor_rpm > max_rpm:
		if not rev_limiter_active:
			rev_limiter_active = true
			rev_limiter_timer = REV_LIMITER_DURATION
		rev_limit_factor = 0.0
	
	if rev_limiter_active:
		rev_limiter_timer -= delta
		if rev_limiter_timer <= 0.0:
			rev_limiter_active = false
		else:
			rev_limit_factor = 0.1  # Slight fuel during limiter
	
	# --- Realistic shift torque reduction ---
	var shift_factor := 1.0
	if is_shifting:
		# More gradual torque reduction with ignition retard simulation
		var shift_progress := clamp(delta * 4.0, 0.0, 1.0)
		shift_factor = lerp(1.0, 0.15, shift_progress)
	
	# --- Calculate final torque output ---
	var gross_torque := base_torque * rev_limit_factor * shift_factor
	torque_output = (gross_torque - total_drag) * clutch_factor
	
	# --- Prevent stalling with realistic idle control ---
	if motor_rpm <= idle_rpm:
		if torque_output < 0.0:
			torque_output = 0.0
		# Idle air control simulation
		var idle_support := calculate_idle_support(motor_rpm)
		torque_output += idle_support
	
	# --- Enhanced RPM calculation with engine inertia ---
	var effective_inertia := motor_moment * engine_inertia_factor
	var rpm_change := (torque_output / effective_inertia) * ANGULAR_VELOCITY_TO_RPM * delta
	
	# Add RPM damping for stability
	var rpm_damping := clamp(abs(rpm_change) * 0.02, 0.0, 0.1)
	rpm_change *= (1.0 - rpm_damping)
	
	motor_rpm = max(motor_rpm + rpm_change, idle_rpm * 0.8)  # Allow slight undershoot for realism
	
	# --- Update engine state flags ---
	motor_is_redline = motor_rpm > max_rpm
	need_clutch = motor_rpm < (idle_rpm + 100)  # Slightly higher threshold
	
	# --- Soft rev limiter approach (pre-limiter) ---
	if motor_rpm > max_rpm * 0.96:
		var limiter_approach := (motor_rpm - max_rpm * 0.96) / (max_rpm * 0.04)
		throttle_amount *= (1.0 - limiter_approach * 0.5)
		
		# Add after each calculation:


# Enhanced helper functions for realistic engine behavior
func calculate_realistic_torque_curve(rpm_ratio: float) -> float:
	# More realistic torque curve with proper power band
	if rpm_ratio <= 0.15:
		# Very low RPM - poor torque
		return 0.3 + (rpm_ratio / 0.15) * 0.4
	elif rpm_ratio <= 0.35:
		# Low-mid RPM torque rise
		return 0.7 + ((rpm_ratio - 0.15) / 0.2) * 0.25
	elif rpm_ratio <= 0.65:
		# Peak torque band
		return 0.95 + sin((rpm_ratio - 0.35) * PI * 1.67) * 0.05
	elif rpm_ratio <= 0.85:
		# Power band - torque starts falling but power peaks
		return 1.0 - ((rpm_ratio - 0.65) * 0.3)
	else:
		# High RPM falloff
		return 0.94 - ((rpm_ratio - 0.85) * 2.0)

func calculate_airflow_factor(throttle_pos: float, rpm: float) -> float:
	# Realistic airflow through throttle body and intake
	var base_flow := throttle_pos
	
	# Intake resonance effects (simplified)
	var resonance_rpm := max_rpm * 0.4  # Peak intake efficiency
	var rpm_efficiency :float= 1.0 - abs(rpm - resonance_rpm) / max_rpm * 0.1
	
	# Manifold vacuum effects
	manifold_vacuum = 1.0 - throttle_pos
	var vacuum_efficiency := 1.0 - (manifold_vacuum * 0.05)
	
	return base_flow * rpm_efficiency * vacuum_efficiency

func calculate_fuel_efficiency(rpm_ratio: float) -> float:
	# Fuel injection efficiency varies with RPM
	if rpm_ratio < 0.1:
		return 0.8  # Rich mixture at very low RPM
	elif rpm_ratio < 0.8:
		return 1.0  # Optimal range
	else:
		return 0.95 - ((rpm_ratio - 0.8) * 0.25)  # Slight richening at high RPM

func calculate_knock_protection(rpm_ratio: float, throttle_pos: float) -> float:
	# Engine knock protection reduces power under high load
	if rpm_ratio > engine_knock_threshold and throttle_pos > 0.7:
		var knock_risk := (rpm_ratio - engine_knock_threshold) * (throttle_pos - 0.7) * 2.0
		var octane_protection := fuel_octane / 100.0
		return 1.0 - (knock_risk * (1.0 - octane_protection) * 0.3)
	return 1.0

func calculate_pumping_losses(throttle_pos: float, rpm: float) -> float:
	# Pumping losses increase with manifold vacuum and RPM
	var vacuum_loss := manifold_vacuum * rpm * 0.00008
	var base_pumping := rpm * 0.00005
	return vacuum_loss + base_pumping

func calculate_mechanical_friction(rpm: float) -> float:
	# Mechanical friction increases with RPM squared
	return rpm * rpm * engine_friction_coefficient * 0.000001

func calculate_clutch_transmission_factor(slip_amount: float, current_torque: float) -> float:
	# Realistic clutch slip behavior under load
	if slip_amount < 0.1:
		return 1.0  # Fully engaged
	
	# Progressive slip with load dependency
	var base_transmission := 1.0 - (slip_amount * 0.9)
	var load_factor := clamp(abs(current_torque) / 400.0, 0.0, 1.0)  # Assume 400Nm reference
	var slip_under_load :float= slip_amount * (1.0 + load_factor * 0.3)
	
	return max(0.1, 1.0 - slip_under_load)

func calculate_idle_support(current_rpm: float) -> float:
	# Idle air control valve simulation
	if current_rpm < idle_rpm:
		var support_needed := (idle_rpm - current_rpm) / idle_rpm
		return support_needed * 30.0  # Torque to maintain idle
	return 0.0
# Add these optional variables for enhanced realism (keep minimal)

func process_clutch(delta: float) -> void:
	if current_gear == 0:
		return
	
	# --- Keep your exact working tuning parameters ---
	var clutch_damping: float = 0.12
	var slip_damping: float = 0.08
	var clutch_torque_target: float = 0.0
	
	# --- Keep your exact working drivetrain calculations ---
	var gear_ratio: float = get_gear_ratio(current_gear)
	var drive_inertia: float = motor_moment + pow(absf(gear_ratio), 2) * gear_inertia + drive_axles_inertia
	var drive_inertia_r: float = drive_inertia / (gear_ratio * gear_ratio)
	var reaction_torque: float = get_drive_wheels_reaction_torque() / gear_ratio
	
	# --- Keep your exact working speed difference calculation ---
	var speed_diff: float = (motor_rpm / ANGULAR_VELOCITY_TO_RPM) - (get_drivetrain_spin() * gear_ratio)
	if speed_diff < 0.0:
		speed_diff = -sqrt(-speed_diff)
	
	# --- Enhanced clutch engagement with bite point (subtle improvement) ---
	var raw_clutch_factor: float = (1.0 - clutch_amount)
	var enhanced_clutch_factor: float = 0.0
	
	if clutch_amount > clutch_bite_point:
		# Progressive engagement after bite point
		var engagement_range: float = 1.0 - clutch_bite_point
		var engagement_position: float = (clutch_amount - clutch_bite_point) / engagement_range
		enhanced_clutch_factor = pow(engagement_position, 1.2)  # Slightly progressive curve
	
	# Blend between original and enhanced for safety
	var clutch_factor: float = lerp(raw_clutch_factor, enhanced_clutch_factor, 0.7)
	
	# --- Keep your exact working base clutch torque calculation ---
	var a: float = motor_moment * drive_inertia_r * speed_diff / delta
	var b: float = motor_moment * reaction_torque
	var c: float = drive_inertia_r * torque_output
	clutch_torque_target = ((a - b + c) / (motor_moment + drive_inertia_r)) * clutch_factor
	
	# --- Enhanced torque limiting with temperature effects ---
	var temp_factor: float = 1.0
	if clutch_temperature > 150.0:
		temp_factor = clamp(1.0 - (clutch_temperature - 150.0) * 0.001, 0.8, 1.0)
	
	var effective_max_clutch_torque: float = max_clutch_torque * clutch_factor * temp_factor
	clutch_torque_target = clamp(clutch_torque_target, -effective_max_clutch_torque, effective_max_clutch_torque)
	
	# --- Keep your exact working smooth torque application ---
	clutch_torque = lerp(clutch_torque, clutch_torque_target, clutch_damping)
	
	# --- Enhanced TCS with better wheel analysis ---
	var max_slip: float = 0.0
	if traction_control_max_slip > 0.0:
		var total_slip: float = 0.0
		var wheel_count: int = 0
		
		for axle in axles:
			var axle_slip: float = axle.get_max_wheel_slip_y(delta)
			max_slip = max(max_slip, axle_slip)
			total_slip += axle_slip
			wheel_count += 1
		
		if max_slip > traction_control_max_slip:
			# Progressive TCS intervention
			var slip_excess: float = max_slip - traction_control_max_slip
			var intervention_strength: float = clamp(slip_excess / traction_control_max_slip, 0.0, 1.0)
			var enhanced_slip_damping: float = slip_damping * (1.0 + intervention_strength)
			
			clutch_torque = lerp(clutch_torque, 0.0, enhanced_slip_damping)
			tcs_active = true
		else:
			tcs_active = false
	
	# --- Differential simulation (subtle enhancement) ---
	if lsd_factor > 1.0:
		# Limited slip differential increases effective torque transfer
		clutch_torque *= lsd_factor
	
	# --- Keep your exact working torque application to wheels ---
	for wheel in drive_wheels:
		if wheel.is_colliding():
			var forward_dir: Vector3 = -wheel.global_transform.basis.z.normalized()
			apply_force(forward_dir * clutch_torque * 0.5, wheel.global_transform.origin - global_transform.origin)
	
	# --- Keep your exact working RPM update ---
	var total_clutch_effect: float = clutch_torque
	if tcs_active:
		total_clutch_effect *= 0.5
	var new_rpm: float = motor_rpm - (ANGULAR_VELOCITY_TO_RPM * delta * total_clutch_effect / motor_moment)
	new_rpm = clamp(new_rpm, idle_rpm, max_rpm * 1.1)
	motor_rpm = new_rpm
	
	# --- Enhanced stall protection ---
	var stall_threshold: float = idle_rpm + 100 + (clutch_factor * 50.0)
	need_clutch = motor_rpm < stall_threshold
	
	# --- Simple temperature simulation (optional) ---
	var slip_power: float = abs(clutch_torque * speed_diff) * 0.01
	var heat_input: float = slip_power * delta
	var cooling: float = (1.0 + speed * 0.02) * delta * 20.0
	clutch_temperature += heat_input - cooling
	clutch_temperature = clamp(clutch_temperature, 20.0, 250.0)
# Bulletproof Assetto Corsa-Level Transmission System
# Zero crashes, maximum realism, handles all edge cases

# Required class variables (add these to your car class)


func process_transmission(delta: float) -> void:
	# Critical safety validation first
	if not _validate_critical_systems(delta):
		return
	
	# Handle emergency states
	if transmission_state == 2:
		_handle_emergency_state(delta)
		return
	
	# Handle active shifting
	if transmission_state == 1:
		_process_gear_shift(delta)
		return
	
	# Update filtered values for stability
	_update_filtered_values(delta)
	
	# Calculate safe drivetrain dynamics
	var drivetrain_data = _calculate_safe_drivetrain_dynamics()
	if drivetrain_data.error:
		_enter_emergency_state("Drivetrain calculation failed")
		return
	
	# Update transmission temperature
	_update_transmission_temperature(drivetrain_data, delta)
	
	# Process automatic transmission logic
	if automatic_transmission and current_gear != -1:
		_process_automatic_transmission(drivetrain_data, delta)
	
	# Handle reverse/drive selection
	_process_reverse_drive_logic(drivetrain_data, delta)
	
	# Calculate realistic clutch dynamics
	var clutch_data = _calculate_advanced_clutch_dynamics(drivetrain_data, delta)
	
	# Calculate final torque output
	var torque_output_data = _calculate_realistic_torque_output(drivetrain_data, clutch_data, delta)
	engine_to_wheel_torque = torque_output_data.final_torque
	
	# Apply engine braking
	if throttle_input < 0.02 and current_gear != 0:
		_apply_advanced_engine_braking(drivetrain_data, clutch_data)
	
	# Update lockout timers
	_update_transmission_timers(delta)

# ================================
# CRITICAL SAFETY VALIDATION
# ================================
func _validate_critical_systems(delta: float) -> bool:
	# Check for invalid numeric values
	if not is_finite(speed) or not is_finite(motor_rpm) or not is_finite(delta):
		print("ERROR: Invalid numeric values detected")
		_enter_emergency_state("Invalid numeric values")
		return false
	
	# Validate delta time
	if delta <= 0.0 or delta > 0.1:  # Cap at 100ms to prevent huge jumps
		return false
	
	# Validate gear bounds
	if current_gear < -1 or current_gear > gear_ratios.size():
		print("ERROR: Invalid gear detected: ", current_gear)
		current_gear = clamp(current_gear, -1, gear_ratios.size())
	
	# Validate RPM bounds
	if motor_rpm < 0.0 or motor_rpm > MAX_SAFE_RPM:
		motor_rpm = clamp(motor_rpm, MIN_SAFE_RPM, MAX_SAFE_RPM)
	
	# Validate speed bounds
	if speed < 0.0 or speed > MAX_SAFE_SPEED:
		speed = clamp(speed, 0.0, MAX_SAFE_SPEED)
	
	# Check drive_wheels array
	if not drive_wheels or drive_wheels.size() == 0:
		print("ERROR: No drive wheels available")
		_enter_emergency_state("No drive wheels")
		return false
	
	return true

func _enter_emergency_state(reason: String):
	print("TRANSMISSION EMERGENCY: ", reason)
	transmission_state = 2
	current_gear = 0  # Force neutral
	engine_to_wheel_torque = 0.0
	shift_timer = EMERGENCY_TIMEOUT
	stall_prevention_active = true

func _handle_emergency_state(delta: float):
	shift_timer -= delta
	engine_to_wheel_torque = 0.0  # No torque during emergency
	
	if shift_timer <= 0.0:
		print("Exiting emergency state")
		transmission_state = 0
		current_gear = 0  # Stay in neutral
		shift_timer = 0.0

# ================================
# FILTERED VALUE UPDATES
# ================================
func _update_filtered_values(delta: float):
	# Speed filtering to prevent spikes
	speed_filter_buffer.push_back(speed)
	if speed_filter_buffer.size() > 5:
		speed_filter_buffer.pop_front()
	
	var filtered_speed = 0.0
	for s in speed_filter_buffer:
		filtered_speed += s
	filtered_speed /= speed_filter_buffer.size()
	
	# Only update if change is reasonable
	if abs(filtered_speed - last_valid_speed) < 20.0 or last_valid_speed == 0.0:
		last_valid_speed = filtered_speed
	
	# RPM filtering
	rpm_filter_buffer.push_back(motor_rpm)
	if rpm_filter_buffer.size() > 3:
		rpm_filter_buffer.pop_front()

# ================================
# SAFE DRIVETRAIN DYNAMICS
# ================================
func _calculate_safe_drivetrain_dynamics() -> Dictionary:
	var total_wheel_rpm: float = 0.0
	var total_wheel_speed: float = 0.0
	var total_slip: float = 0.0
	var valid_wheels: int = 0
	var max_individual_slip: float = 0.0
	
	for wheel in drive_wheels:
		# Validate wheel object
		if not wheel or not is_instance_valid(wheel):
			continue
		
		# Get wheel radius safely
		var wheel_radius = 0.35  # Default
		if wheel.tire_radius:
			wheel_radius = max(0.2, wheel.tire_radius)

		
		# Get wheel spin safely
		var wheel_spin = 0.0
		if wheel.spin:
			wheel_spin = wheel.spin
			# Validate wheel spin
			if not is_finite(wheel_spin):
				wheel_spin = 0.0
			wheel_spin = clamp(wheel_spin, -100.0, 100.0)
		
		# Calculate wheel speeds
		var wheel_linear_speed = abs(wheel_spin * wheel_radius)
		var wheel_rpm = abs(wheel_spin) * ANGULAR_TO_RPM
		
		total_wheel_rpm += wheel_rpm
		total_wheel_speed += wheel_linear_speed
		
		# Calculate slip safely
		var expected_speed = max(last_valid_speed, 0.1)
		if expected_speed > 0.1:
			var slip_ratio = abs(wheel_linear_speed - expected_speed) / expected_speed
			slip_ratio = clamp(slip_ratio, 0.0, 3.0)  # Reasonable slip limits
			total_slip += slip_ratio
			max_individual_slip = max(max_individual_slip, slip_ratio)
		
		valid_wheels += 1
	
	# Validate we have wheels
	if valid_wheels == 0:
		return {"error": true}
	
	# Calculate averages
	var avg_wheel_rpm = total_wheel_rpm / valid_wheels
	var avg_wheel_speed = total_wheel_speed / valid_wheels
	var avg_slip = total_slip / valid_wheels
	
	# Calculate drivetrain RPM
	var gear_ratio = get_gear_ratio(current_gear)
	var total_ratio = gear_ratio * final_drive
	var drivetrain_rpm = avg_wheel_rpm * abs(total_ratio)
	
	# Validate drivetrain RPM
	drivetrain_rpm = clamp(drivetrain_rpm, 0.0, MAX_SAFE_RPM * 1.5)
	
	return {
		"error": false,
		"wheel_rpm": avg_wheel_rpm,
		"wheel_speed": avg_wheel_speed,
		"drivetrain_rpm": drivetrain_rpm,
		"slip_factor": clamp(avg_slip, 0.0, 2.0),
		"max_slip": clamp(max_individual_slip, 0.0, 2.0),
		"traction_available": (avg_slip < 0.25),
		"valid_wheels": valid_wheels,
		"gear_ratio": gear_ratio,
		"total_ratio": total_ratio
	}

# ================================
# AUTOMATIC TRANSMISSION LOGIC
# ================================
func _process_automatic_transmission(drivetrain_data: Dictionary, delta: float):
	# Don't shift if we're in lockout, emergency, or just came from reverse
	if gear_change_lockout > 0.0 or transmission_state != 0:
		return
	
	# CRITICAL: Don't process automatic if we just disengaged reverse
	if reverse_lockout_timer > 1.5:  # Give extra time after reverse
		return
	
	# Don't shift if wheels are slipping badly
	if drivetrain_data.max_slip > 0.4:
		return
	
	# Special handling for neutral after reverse
	if current_gear == 0 and shift_target_gear == 1:
		# Handle delayed first gear engagement after reverse
		if shift_timer < 0.0:
			shift_timer += delta
			return
		else:
			# Now engage first gear
			_initiate_gear_change(1)
			shift_target_gear = 0  # Clear target
			return
	
	# Calculate intelligent shift points
	var upshift_rpm = _calculate_upshift_point()
	var downshift_rpm = _calculate_downshift_point()
	
	# Upshift logic - ONLY for forward gears
	if current_gear > 0 and current_gear < gear_ratios.size():
		if drivetrain_data.drivetrain_rpm > upshift_rpm or motor_rpm > max_rpm * 0.95:
			if drivetrain_data.traction_available:
				_initiate_gear_change(current_gear + 1)
	
	# Downshift logic - ONLY for forward gears 2+
	if current_gear > 1:
		var would_overrev = _check_downshift_overrev(current_gear - 1, drivetrain_data.wheel_rpm)
		if not would_overrev:
			if (drivetrain_data.drivetrain_rpm < downshift_rpm and throttle_input > 0.3) or brake_input > 0.7:
				_initiate_gear_change(current_gear - 1)
	
	# Neutral to first engagement - ONLY if not coming from reverse
	if current_gear == 0 and reverse_lockout_timer <= 0.0:
		if throttle_input > 0.08 and motor_rpm > idle_rpm * 1.15 and last_valid_speed < 1.5:
			_initiate_gear_change(1)

func _calculate_upshift_point() -> float:
	var base_rpm = max_rpm * 0.83
	base_rpm += throttle_input * max_rpm * 0.08  # Higher throttle = later shift
	base_rpm += engine_load_factor * max_rpm * 0.04  # Load compensation
	return clamp(base_rpm, max_rpm * 0.75, max_rpm * 0.94)

func _calculate_downshift_point() -> float:
	var base_rpm = max_rpm * 0.32
	base_rpm += brake_input * max_rpm * 0.15  # Braking = higher downshift
	base_rpm += throttle_input * max_rpm * 0.08  # Acceleration = higher downshift
	return clamp(base_rpm, max_rpm * 0.25, max_rpm * 0.5)

func _check_downshift_overrev(target_gear: int, wheel_rpm: float) -> bool:
	var target_gear_ratio = get_gear_ratio(target_gear)
	var predicted_rpm = wheel_rpm * target_gear_ratio * final_drive
	return predicted_rpm > max_rpm * 0.9

# ================================
# REVERSE/DRIVE LOGIC
# ================================
func _process_reverse_drive_logic(drivetrain_data: Dictionary, delta: float):
	# Don't interfere during active shifting or emergency
	if transmission_state != 0:
		return
	
	# Only process if we're at very low speed
	if last_valid_speed > 3.0:
		return
	
	# Must have significant brake input for safety
	if brake_input < 0.8:
		return
	
	# Check reverse lockout
	if reverse_lockout_timer > 0.0:
		return
	
	var forward_velocity = -local_velocity.z  # Forward is negative Z in Godot
	
	# Reverse engagement logic
	if current_gear >= 0 and _wants_reverse():
		if last_valid_speed < 0.8 and abs(forward_velocity) < 0.3:
			print("Engaging reverse from gear: ", current_gear)
			_initiate_safe_reverse_engagement()
			reverse_lockout_timer = 2.5
	
	# Forward engagement from reverse - CRITICAL FIX
	elif current_gear == -1 and _wants_forward():
		if last_valid_speed < 0.8 and abs(forward_velocity) < 0.3:
			print("Engaging forward from reverse")
			_initiate_safe_forward_engagement()
			reverse_lockout_timer = 2.0

func _initiate_safe_reverse_engagement():
	# Force through neutral first for safety
	if current_gear != 0:
		current_gear = 0
		# Brief pause in neutral
		gear_change_lockout = 0.3
	_initiate_gear_change(-1)

func _initiate_safe_forward_engagement():
	# CRITICAL: Go to neutral first, then to first gear
	current_gear = 0  # Force neutral
	transmission_state = 0  # Clear any shifting state
	gear_change_lockout = 0.2  # Brief pause
	
	# Queue first gear engagement after brief delay
	shift_target_gear = 1
	shift_timer = -0.2  # Negative timer creates delay

func _wants_reverse() -> bool:
	# More restrictive reverse conditions
	return (brake_input > 0.95 and 
			throttle_input < 0.02 and 
			abs(local_velocity.z) < 0.2 and
			last_valid_speed < 0.5)

func _wants_forward() -> bool:
	# Clear forward intent
	return (throttle_input > 0.05 or 
			(-local_velocity.z > 0.1))

# ================================
# GEAR SHIFT PROCESSING
# ================================
func _initiate_gear_change(new_gear: int):
	# Prevent invalid gear changes
	if new_gear == current_gear or transmission_state != 0:
		return
	
	# CRITICAL: Validate gear change is safe
	if new_gear < -1 or new_gear > gear_ratios.size():
		print("Invalid gear change attempted: ", new_gear)
		return
	
	# Special validation for reverse/forward transitions
	if current_gear == -1 and new_gear > 0:
		print("ERROR: Direct reverse to forward gear not allowed")
		return  # Must go through neutral first
	
	if current_gear > 0 and new_gear == -1:
		print("ERROR: Direct forward to reverse not allowed")
		return  # Must go through neutral first
	
	shift_target_gear = new_gear
	transmission_state = 1
	shift_timer = 0.0
	gear_change_lockout = 1.0  # Longer lockout to prevent issues
	
	print("Shifting from ", current_gear, " to ", new_gear)

func _process_gear_shift(delta: float):
	shift_timer += delta
	
	# Safety timeout
	if shift_timer > GEAR_CHANGE_TIME * 3.0:  # Longer timeout for safety
		print("Gear shift timeout, aborting")
		_abort_gear_shift()
		return
	
	var shift_progress = shift_timer / GEAR_CHANGE_TIME
	
	if shift_progress < 0.3:
		# Disengagement phase - reduce torque
		var torque_reduction = lerp(1.0, 0.05, shift_progress / 0.3)
		engine_to_wheel_torque *= torque_reduction
	elif shift_progress < 0.7:
		# Synchronization phase - in neutral
		var old_gear = current_gear
		current_gear = 0
		engine_to_wheel_torque *= 0.05  # Minimal torque in neutral
		
		# Rev matching for smooth shifts
		_perform_rev_matching()
		
		# Log gear state changes
		if old_gear != 0:
			print("In neutral during shift to ", shift_target_gear)
	elif shift_progress < 1.0:
		# Engagement phase - gradually restore torque
		if current_gear != shift_target_gear:
			current_gear = shift_target_gear
			print("Engaging gear ", current_gear)
		
		var engage_factor = (shift_progress - 0.7) / 0.3
		# Torque will be calculated normally with engagement factor
	else:
		# Shift complete
		current_gear = shift_target_gear
		transmission_state = 0
		shift_timer = 0.0
		print("Shift complete to gear ", current_gear)
		
		# CRITICAL: Reset any delayed targets
		shift_target_gear = 0

func _perform_rev_matching():
	# Enhanced rev matching with safety
	if shift_target_gear == 0:
		return
	
	# Calculate target RPM more accurately
	var target_drivetrain_rpm = last_valid_speed / (0.35 * 2.0 * PI) * 60.0  # Estimate from speed
	var target_gear_ratio = get_gear_ratio(shift_target_gear)
	var target_engine_rpm = target_drivetrain_rpm * target_gear_ratio * final_drive
	
	# Clamp target to reasonable range
	target_engine_rpm = clamp(target_engine_rpm, idle_rpm, max_rpm * 0.9)
	
	# Gentle rev matching (ECU would handle this in real cars)
	if abs(motor_rpm - target_engine_rpm) > 200.0:
		var adjustment_rate = 1500.0 * get_process_delta_time()
		if motor_rpm < target_engine_rpm:
			motor_rpm = min(motor_rpm + adjustment_rate, target_engine_rpm)
		else:
			motor_rpm = max(motor_rpm - adjustment_rate, target_engine_rpm)

func _abort_gear_shift():
	print("Aborting gear shift - returning to neutral")
	transmission_state = 0
	current_gear = 0  # Always return to neutral on abort
	shift_timer = 0.0
	shift_target_gear = 0
	engine_to_wheel_torque = 0.0
	gear_change_lockout = 1.5  # Longer lockout after abort

# ================================
# ADVANCED CLUTCH DYNAMICS
# ================================
func _calculate_advanced_clutch_dynamics(drivetrain_data: Dictionary, delta: float) -> Dictionary:
	var clutch_engagement = clamp(clutch_amount, 0.0, 1.0)
	
	# Calculate clutch slip
	var rpm_difference = motor_rpm - drivetrain_data.drivetrain_rpm
	rpm_difference = clamp(rpm_difference, -3000.0, 3000.0)
	clutch_slip_velocity = rpm_difference
	
	var slip_ratio = 0.0
	if motor_rpm > MIN_SAFE_RPM:
		slip_ratio = rpm_difference / motor_rpm
		slip_ratio = clamp(slip_ratio, -1.5, 1.5)
	
	# Heat generation
	var heat_rate = abs(slip_ratio) * abs(rpm_difference) * 0.00008
	clutch_temperature += heat_rate * delta
	clutch_temperature = max(0.0, clutch_temperature - delta * 0.35)  # Cooling
	clutch_temperature = clamp(clutch_temperature, 0.0, 2.5)
	
	# Clutch engagement curve (non-linear)
	var effective_engagement = _clutch_engagement_curve(clutch_engagement)
	
	# Temperature effects
	var temp_factor = max(0.4, 1.0 - (clutch_temperature * 0.1))
	
	# Torque capacity and efficiency
	var torque_capacity = 450.0 * effective_engagement * temp_factor
	var slip_efficiency = max(0.25, 1.0 - (abs(slip_ratio) * 0.25))
	var transfer_efficiency = effective_engagement * slip_efficiency * temp_factor
	
	return {
		"engagement": effective_engagement,
		"slip_velocity": clutch_slip_velocity,
		"slip_ratio": slip_ratio,
		"torque_capacity": torque_capacity,
		"transfer_efficiency": transfer_efficiency,
		"temperature": clutch_temperature,
		"overheated": clutch_temperature > 2.0
	}

func _clutch_engagement_curve(position: float) -> float:
	# Realistic clutch bite point
	if position < 0.12:
		return 0.0
	elif position < 0.35:
		var t = (position - 0.12) / 0.23
		return pow(t, 1.8) * 0.3  # Gradual initial bite
	elif position < 0.85:
		var t = (position - 0.35) / 0.5
		return 0.3 + (t * 0.65)  # Linear main engagement
	else:
		var t = (position - 0.85) / 0.15
		return 0.95 + (t * 0.05)  # Full lock-up

# ================================
# REALISTIC TORQUE OUTPUT
# ================================
func _calculate_realistic_torque_output(drivetrain_data: Dictionary, clutch_data: Dictionary, delta: float) -> Dictionary:
	# Base torque through clutch
	var clutch_torque = torque_output * clutch_data.transfer_efficiency
	
	# Apply gear ratio
	var geared_torque = clutch_torque * drivetrain_data.gear_ratio
	
	# Apply final drive
	var final_torque = geared_torque * final_drive
	
	# Drivetrain efficiency
	var efficiency = _calculate_drivetrain_efficiency(drivetrain_data, clutch_data)
	final_torque *= efficiency
	
	# Apply torque delivery characteristics
	final_torque = _apply_torque_characteristics(final_torque, drivetrain_data, clutch_data)
	
	# Safety torque limiting
	var torque_limit = _calculate_safe_torque_limit(drivetrain_data, clutch_data)
	final_torque = clamp(final_torque, -torque_limit, torque_limit)
	
	return {
		"final_torque": final_torque,
		"efficiency": efficiency,
		"limit": torque_limit
	}

func _calculate_drivetrain_efficiency(drivetrain_data: Dictionary, clutch_data: Dictionary) -> float:
	var base_efficiency = 0.94
	
	# Losses from slip
	base_efficiency -= drivetrain_data.slip_factor * 0.04
	
	# Temperature losses
	base_efficiency -= (transmission_temperature - 80.0) / 200.0 * 0.03
	base_efficiency -= clutch_data.temperature * 0.01
	
	# Gear ratio efficiency (higher ratios = more loss)
	base_efficiency -= abs(drivetrain_data.total_ratio) * 0.001
	
	return clamp(base_efficiency, 0.85, 0.96)

func _apply_torque_characteristics(base_torque: float, drivetrain_data: Dictionary, clutch_data: Dictionary) -> float:
	var modified_torque = base_torque
	
	# Low speed torque limiting for smooth starts
	if last_valid_speed < 3.0:
		var speed_factor = last_valid_speed / 3.0
		var smoothing = lerp(0.4, 1.0, speed_factor)
		modified_torque *= smoothing
	
	# Traction-based torque reduction
	if drivetrain_data.max_slip > 0.15:
		var traction_factor = 1.0 - ((drivetrain_data.max_slip - 0.15) * 0.4)
		modified_torque *= clamp(traction_factor, 0.5, 1.0)
	
	# Clutch slip protection
	if abs(clutch_data.slip_velocity) > 300.0:
		var slip_protection = 1.0 - ((abs(clutch_data.slip_velocity) - 300.0) / 1000.0) * 0.3
		modified_torque *= clamp(slip_protection, 0.6, 1.0)
	
	# Anti-stall protection
	if motor_rpm < idle_rpm * 1.3 and current_gear > 0:
		var anti_stall = motor_rpm / (idle_rpm * 1.3)
		modified_torque *= clamp(anti_stall, 0.4, 1.0)
		stall_prevention_active = (anti_stall < 0.8)
	else:
		stall_prevention_active = false
	
	return modified_torque

func _calculate_safe_torque_limit(drivetrain_data: Dictionary, clutch_data: Dictionary) -> float:
	var base_limit = 700.0
	
	# Reduce for wheel slip
	base_limit *= (1.0 - drivetrain_data.max_slip * 0.3)
	
	# Reduce for clutch overheating
	if clutch_data.overheated:
		base_limit *= 0.7
	
	# Reduce for transmission overheating
	if transmission_temperature > 120.0:
		base_limit *= (1.0 - (transmission_temperature - 120.0) / 80.0 * 0.2)
	
	# Gear-specific limits (lower gears can handle more)
	if current_gear > 0:
		var gear_factor = 1.0 + (0.15 * max(0, 4 - current_gear))
		base_limit *= clamp(gear_factor, 0.8, 1.4)
	
	return clamp(base_limit, 200.0, 900.0)

# ================================
# ENGINE BRAKING
# ================================
func _apply_advanced_engine_braking(drivetrain_data: Dictionary, clutch_data: Dictionary):
	var engine_friction = drivetrain_data.drivetrain_rpm * 0.025
	var compression_braking = drivetrain_data.drivetrain_rpm * 0.03
	var pumping_losses = drivetrain_data.drivetrain_rpm * 0.012
	
	var total_braking = (engine_friction + compression_braking + pumping_losses)
	total_braking *= clutch_data.engagement
	total_braking *= abs(drivetrain_data.gear_ratio)
	
	# More braking in lower gears
	if current_gear > 0 and current_gear <= 3:
		total_braking *= (1.0 + (0.3 * (4 - current_gear)))
	
	total_braking = clamp(total_braking, 0.0, 300.0)
	
	# Apply in correct direction
	if current_gear > 0:
		engine_to_wheel_torque -= total_braking
	elif current_gear == -1:
		engine_to_wheel_torque += total_braking

# ================================
# HELPER FUNCTIONS
# ================================
func _update_transmission_temperature(drivetrain_data: Dictionary, delta: float):
	# Heat from torque transfer
	var heat_generation = abs(engine_to_wheel_torque) * 0.0008
	
	# Heat from gear mesh losses
	heat_generation += drivetrain_data.slip_factor * 2.0
	
	transmission_temperature += heat_generation * delta
	
	# Cooling (airflow, oil circulation)
	var cooling_rate = 1.5 + (last_valid_speed * 0.02)  # Air cooling
	transmission_temperature = max(25.0, transmission_temperature - cooling_rate * delta)
	transmission_temperature = clamp(transmission_temperature, 25.0, 160.0)

func _update_transmission_timers(delta: float):
	reverse_lockout_timer = max(0.0, reverse_lockout_timer - delta)
	gear_change_lockout = max(0.0, gear_change_lockout - delta)

# Update engine load factor for intelligent shifting
func _update_engine_load_factor(delta: float):
	var current_load = torque_output / max(motor_rpm, 1.0)  # Simplified load calculation
	engine_load_factor = lerp(engine_load_factor, current_load, delta * 2.0)
	engine_load_factor = clamp(engine_load_factor, 0.5, 3.0)	
func process_drive(delta: float) -> void:
	var current_gear_ratio := get_gear_ratio(current_gear)
	var base_drive_torque := 0.0
	var drive_inertia := motor_moment + pow(abs(current_gear_ratio), 2) * gear_inertia
	var is_slipping := get_is_a_wheel_slipping()
	
	# Enhanced engine RPM calculation (Assetto Corsa style)
	var wheel_speed_avg := 0.0
	if front_axle and rear_axle:
		var front_speed :float= front_axle.get_average_wheel_speed() if front_axle.has_method("get_average_wheel_speed") else 0.0
		var rear_speed :float= rear_axle.get_average_wheel_speed() if rear_axle.has_method("get_average_wheel_speed") else 0.0
		wheel_speed_avg = (front_speed + rear_speed) * 0.5
	
	var engine_rpm := max(idle_rpm, abs(wheel_speed_avg * abs(current_gear_ratio) * 9.549)) # rad/s to RPM
	var normalized_rpm := clamp(engine_rpm / max_rpm, 0.0, 1.0)
	
	# Realistic torque curve (like AC's engine modeling)
	var torque_curve_multiplier := calculate_realistic_torque_curve(normalized_rpm)
	
	# Calculate base engine torque with realistic response
	if current_gear != 0:
		# Throttle response with engine characteristics
		var effective_throttle := apply_throttle_response(throttle_amount, engine_rpm, delta)
		
		# Reverse gear speed limiting (AC-style electronic limiter)
		if current_gear == -1:
			var backward_speed := -linear_velocity.dot(transform.basis.z) * 3.6 # km/h backward
			
			if backward_speed > 5.0:
				effective_throttle = 0.0 # Hard electronic cut
			elif backward_speed > 4.0:
				var reduction := smoothstep(5.0, 4.0, backward_speed) # Smooth transition
				effective_throttle *= reduction
		
		# Base torque with realistic engine characteristics
		var max_torque_at_rpm := clutch_torque * torque_curve_multiplier
		base_drive_torque = max_torque_at_rpm * current_gear_ratio * effective_throttle
		
		# Advanced engine braking (compression + friction)
		if effective_throttle < 0.05:
			var engine_brake_strength :float= 0.12 + (normalized_rpm * 0.08) # Stronger at higher RPM
			var engine_brake :float= -max_torque_at_rpm * engine_brake_strength * current_gear_ratio
			base_drive_torque += engine_brake
		
		# Clutch slip under extreme load (like AC)
		var clutch_capacity :float= clutch_torque * abs(current_gear_ratio) * 1.2
		if abs(base_drive_torque) > clutch_capacity:
			var slip_factor :float= clutch_capacity / abs(base_drive_torque)
			base_drive_torque *= lerp(slip_factor, 1.0, 0.95) # Gradual slip
	
	# Enhanced AWD system with realistic center differential
	var target_torque_split := front_torque_split
	if variable_torque_split:
		# Advanced traction-based torque distribution
		var front_traction := calculate_axle_traction(front_axle)
		var rear_traction := calculate_axle_traction(rear_axle)
		var total_traction := front_traction + rear_traction
		
		if total_traction > 0.001 and throttle_amount > 0.1:
			# Bias toward axle with better traction (like Haldex/Torsen)
			var traction_bias := front_traction / total_traction
			target_torque_split = lerp(front_torque_split, traction_bias, 0.6)
			target_torque_split = clamp(target_torque_split, 0.1, 0.9)
		
		# Smooth transition with realistic response time
		var split_rate := 3.0 if is_slipping else 1.5
		current_torque_split = lerp(current_torque_split, 
			(target_torque_split - front_torque_split) / (front_variable_split - front_torque_split), 
			delta * split_rate)
		current_torque_split = clamp(current_torque_split, 0.0, 1.0)
	
	# Apply enhanced differential physics (closer to AC's modeling)
	true_torque_split = lerp(front_torque_split, front_variable_split, current_torque_split)
	var axle_a := front_axle
	var axle_b := rear_axle
	if true_torque_split <= 0.5:
		axle_a = rear_axle
		axle_b = front_axle
	
	# More accurate axle coupling with realistic center diff behavior
	var axle_a_spin :float= axle_a.get_average_wheel_speed() if axle_a.has_method("get_average_wheel_speed") else (axle_a.wheels[0].spin + axle_a.wheels[1].spin) * 0.5
	var axle_b_spin :float= axle_b.get_average_wheel_speed() if axle_b.has_method("get_average_wheel_speed") else (axle_b.wheels[0].spin + axle_b.wheels[1].spin) * 0.5
	var axle_difference :float= axle_a_spin - axle_b_spin
	
	# Enhanced center differential with realistic preload and slip characteristics
	var diff_preload := 50.0 # Nm preload torque
	var diff_efficiency := 0.96 # Mechanical losses
	
	var a: float = (axle_a.inertia * axle_b.inertia * axle_difference) / delta
	var b: float = axle_a.inertia * diff_preload * sign(axle_difference)
	var c: float = axle_b.inertia * base_drive_torque * diff_efficiency
	var transfer_torque := (a - b + c) / (axle_a.inertia + axle_b.inertia)
	
	# Realistic torque limiting with center diff characteristics
	var max_transfer :float= abs(base_drive_torque) * 0.8 # 80% max transfer
	transfer_torque = clamp(transfer_torque, -max_transfer, max_transfer)
	
	# Apply torque split bias with smooth transition
	var split_bias :float= (1.0 - abs((0.5 - true_torque_split) * 2.0))
	transfer_torque *= split_bias
	var transfer_torque_2 := base_drive_torque - transfer_torque
	
	# Final torque application with drivetrain losses
	var drivetrain_efficiency := 0.94 # Realistic drivetrain losses
	transfer_torque *= drivetrain_efficiency
	transfer_torque_2 *= drivetrain_efficiency
	
	process_axle_drive(axle_b, transfer_torque, drive_inertia, delta)
	process_axle_drive(axle_a, transfer_torque_2, drive_inertia, delta)

# Realistic torque curve calculation (AC-style)

# Enhanced throttle response modeling
func apply_throttle_response(throttle: float, rpm: float, delta: float) -> float:
	# Simulate intake manifold filling and engine response
	var response_time :float= 0.05 + (1.0 / max(rpm * 0.001, 1.0)) * 0.02 # Faster at higher RPM
	var target_throttle := clamp(throttle, 0.0, 1.0)
	
	# Use a more realistic throttle filter
	if not has_meta("filtered_throttle"):
		set_meta("filtered_throttle", 0.0)
	
	var filtered := get_meta("filtered_throttle")
	filtered = lerp(filtered, target_throttle, clamp(delta / response_time, 0.0, 1.0))
	set_meta("filtered_throttle", filtered)
	
	return filtered

# Calculate axle traction for AWD logic
func calculate_axle_traction(axle) -> float:
	if not axle or axle.wheels.size() < 2:
		return 0.0
	
	var total_traction := 0.0
	for wheel in axle.wheels:
		# Estimate traction based on wheel slip and load
		var wheel_slip := abs(wheel.slip_ratio) if wheel.has_method("get_slip_ratio") else 0.0
		var traction := clamp(1.0 - wheel_slip * 2.0, 0.0, 1.0) # Lower slip = better traction
		total_traction += traction
	
	return total_traction / axle.wheels.size()
func process_axle_drive(axle: Axle, torque: float, drive_inertia: float, delta: float) -> void:
	if not axle.is_drive_axle:
		torque = 0.0
		drive_inertia = 0.0
	
	# ------------------------------
	# Realistic brake system
	# ------------------------------
	var effective_brake_force: float = brake_force
	var allow_abs: bool = true
	
	if axle.handbrake:
		effective_brake_force += handbrake_force
		allow_abs = false # Handbrake bypasses ABS
	
	# Brake bias and balance - realistic distribution
	var brake_bias_factor: float = axle.brake_bias
	if axle == front_axle:
		# Front brakes typically do 60-70% of braking
		brake_bias_factor *= 1.2
	else:
		# Rear brakes get less force to prevent lockup
		brake_bias_factor *= 0.8
	
	effective_brake_force *= brake_bias_factor
	
	# ------------------------------
	# Advanced differential modeling
	# ------------------------------
	var left_wheel_torque: float = 0.0
	var right_wheel_torque: float = 0.0
	var target_split: float = 0.5 # Default 50/50 split
	
	if axle.is_drive_axle and torque != 0.0:
		# Get wheel speeds for differential calculations
		var left_speed: float = axle.wheels[0].angular_velocity if axle.wheels[0].has_method("get_angular_velocity") else axle.wheels[0].spin
		var right_speed: float = axle.wheels[1].angular_velocity if axle.wheels[1].has_method("get_angular_velocity") else axle.wheels[1].spin
		var speed_difference: float = left_speed - right_speed
		var average_speed: float = (left_speed + right_speed) * 0.5
		
		# Get wheel loads for realistic torque distribution
		var left_load: float = get_wheel_load(axle.wheels[0])
		var right_load: float = get_wheel_load(axle.wheels[1])
		var total_load: float = max(left_load + right_load, 0.001)
		
		# Load-sensitive torque distribution (more realistic)
		var load_split: float = left_load / total_load
		
		# Differential type behavior
		if axle.differential_lock_torque >= 0.0:
			var abs_torque: float = abs(torque)
			var lock_percentage: float = clamp(abs_torque / max(axle.differential_lock_torque, 1.0), 0.0, 1.0)
			
			if lock_percentage > 0.8: # Near full lock
				# Limited slip differential - high lock
				var torque_vectoring_influence: float = 0.0
				if axle.torque_vectoring > 0.0:
					torque_vectoring_influence = axle.torque_vectoring * -steering_input * 0.3
				
				target_split = 0.5 + torque_vectoring_influence
				target_split = clamp(target_split, 0.2, 0.8) # Limit extreme splits
				
				# Smooth transition to avoid sudden changes
				axle.rotation_split = lerp(axle.rotation_split, (target_split * 2.0) - 1.0, delta * 8.0)
				
			else:
				# Open differential with preload
				var preload_effect: float = axle.differential_preload_torque / max(abs_torque, 1.0)
				preload_effect = clamp(preload_effect, 0.0, 0.8)
				
				# Wheel with less grip gets more torque (open diff behavior)
				var left_grip: float = get_wheel_grip_factor(axle.wheels[0])
				var right_grip: float = get_wheel_grip_factor(axle.wheels[1])
				var grip_difference: float = (right_grip - left_grip) * 0.5
				
				# Combine load and grip effects
				target_split = load_split + grip_difference
				
				# Apply preload to bias toward equal distribution
				target_split = lerp(target_split, 0.5, preload_effect)
				target_split = clamp(target_split, 0.1, 0.9)
				
				# Speed-sensitive differential response
				var speed_factor: float = clamp(abs(speed_difference) / max(abs(average_speed), 1.0), 0.0, 1.0)
				var diff_response: float = lerp(0.1, 1.0, speed_factor)
				
				axle.rotation_split = lerp(axle.rotation_split, (target_split * 2.0) - 1.0, delta * diff_response * 5.0)
		else:
			# Locked differential (spool)
			axle.rotation_split = 0.0 # Perfect 50/50 split
			target_split = 0.5
	
	# ------------------------------
	# Realistic torque split calculation
	# ------------------------------
	var current_split: float = (axle.rotation_split + 1.0) * 0.5
	current_split = clamp(current_split, 0.0, 1.0)
	
	# Anti-jitter filtering for smooth operation
	var split_change: float = abs(current_split - target_split)
	if split_change < 0.05: # Small changes get smoothed
		current_split = lerp(current_split, target_split, delta * 2.0)
	
	axle.applied_split = current_split
	
	# Calculate individual wheel torques
	left_wheel_torque = torque * current_split
	right_wheel_torque = torque * (1.0 - current_split)
	
	# ------------------------------
	# Load transfer effects on braking
	# ------------------------------
	var left_brake: float = effective_brake_force * 0.5
	var right_brake: float = effective_brake_force * 0.5
	
	# Under braking, load transfers forward and to outside wheel in corners
	if effective_brake_force > 0.1:
		var lateral_load_transfer: float = abs(steering_input) * 0.2
		if steering_input > 0.0: # Turning right
			left_brake *= (1.0 + lateral_load_transfer) # Outside wheel gets more brake
			right_brake *= (1.0 - lateral_load_transfer)
		else: # Turning left
			right_brake *= (1.0 + lateral_load_transfer)
			left_brake *= (1.0 - lateral_load_transfer)
	
	# ------------------------------
	# Apply forces to wheels with realistic physics
	# ------------------------------
	var left_rotation_result: float = 0.0
	var right_rotation_result: float = 0.0
	
	if axle.wheels.size() >= 2:
		# Process left wheel
		left_rotation_result = axle.wheels[0].process_torque(
			left_wheel_torque, 
			drive_inertia * current_split, 
			left_brake, 
			allow_abs, 
			delta
		)
		
		# Process right wheel
		right_rotation_result = axle.wheels[1].process_torque(
			right_wheel_torque, 
			drive_inertia * (1.0 - current_split), 
			right_brake, 
			allow_abs, 
			delta
		)
	
	# ------------------------------
	# Differential coupling effects
	# ------------------------------
	var rotation_sum: float = left_rotation_result + right_rotation_result
	
	# Realistic differential feedback - wheels influence each other
	if axle.is_drive_axle and axle.differential_lock_torque >= 0.0:
		var coupling_strength: float = clamp(axle.differential_lock_torque / max(abs(torque), 1.0), 0.0, 1.0)
		var average_rotation: float = rotation_sum * 0.5
		
		# Limited slip coupling - wheels try to match speeds
		var left_coupling: float = lerp(left_rotation_result, average_rotation, coupling_strength * 0.3)
		var right_coupling: float = lerp(right_rotation_result, average_rotation, coupling_strength * 0.3)
		
		rotation_sum = left_coupling + right_coupling
	
	# Update axle rotation split with anti-oscillation
	var new_split: float = clamp(rotation_sum, -1.0, 1.0)
	var split_rate_limit: float = 8.0 * delta # Limit how fast split can change
	var split_difference: float = new_split - axle.rotation_split
	
	if abs(split_difference) > split_rate_limit:
		new_split = axle.rotation_split + sign(split_difference) * split_rate_limit
	
	axle.rotation_split = new_split

# Helper functions for realistic wheel physics
func get_wheel_load(wheel) -> float:
	# Estimate wheel load based on suspension compression or force
	if wheel.has_method("get_suspension_force"):
		return max(wheel.get_suspension_force(), 100.0) # Minimum load
	elif wheel.has_method("get_suspension_compression"):
		var compression: float = wheel.get_suspension_compression()
		return lerp(200.0, 1200.0, clamp(compression, 0.0, 1.0)) # Realistic load range
	else:
		return 800.0 # Default wheel load (kg force equivalent)

func get_wheel_grip_factor(wheel) -> float:
	# Estimate grip based on slip or surface conditions
	if wheel.has_method("get_grip_factor"):
		return wheel.get_grip_factor()
	elif wheel.has_method("get_slip_ratio"):
		var slip: float = abs(wheel.get_slip_ratio())
		return clamp(1.0 - (slip * 0.8), 0.2, 1.0) # Grip decreases with slip
	else:
		return 1.0 # Default full grip
func process_forces(delta: float) -> void:
	# ------------------------------
	# Realistic suspension parameters
	# ------------------------------
	var anti_roll_strength := 0.6 # Base anti-roll bar stiffness
	var damper_rebound_factor := 1.2 # Rebound damping is typically higher than compression
	var load_transfer_rate := 0.8 # How quickly load transfers (0-1)
	var ride_height_influence := 0.3 # How much ride height affects handling
	var aerodynamic_downforce := 0.0 # Will be calculated based on speed
	
	# Calculate vehicle dynamics for load transfer
	var vehicle_speed: float = linear_velocity.length()
	var lateral_acceleration: float = calculate_lateral_acceleration()
	var longitudinal_acceleration: float = calculate_longitudinal_acceleration(delta)
	
	# Aerodynamic downforce increases with speed squared
	aerodynamic_downforce = calculate_aerodynamic_downforce(vehicle_speed)
	
	# Total vehicle weight including downforce
	var effective_weight: float = mass * 9.81 + aerodynamic_downforce
	
	# ------------------------------
	# Load transfer calculations
	# ------------------------------
	var longitudinal_load_transfer: float = calculate_longitudinal_load_transfer(longitudinal_acceleration, effective_weight)
	var lateral_load_transfer: float = calculate_lateral_load_transfer(lateral_acceleration, effective_weight)
	
	# Process each axle with realistic physics
	for axle_index in range(axles.size()):
		var axle = axles[axle_index]
		var is_front_axle: bool = (axle_index == 0) # Assumes first axle is front
		
		# ------------------------------
		# Base weight distribution
		# ------------------------------
		var axle_weight_ratio: float = 0.5 # Default 50/50 distribution
		if is_front_axle:
			axle_weight_ratio = front_weight_distribution if "front_weight_distribution" in self else 0.6
		else:
			axle_weight_ratio = 1.0 - axle_weight_ratio
		
		var base_axle_load: float = effective_weight * axle_weight_ratio
		
		# ------------------------------
		# Apply load transfers
		# ------------------------------
		var axle_longitudinal_transfer: float = 0.0
		if is_front_axle:
			axle_longitudinal_transfer = longitudinal_load_transfer
		else:
			axle_longitudinal_transfer = -longitudinal_load_transfer
		
		var adjusted_axle_load: float = base_axle_load + axle_longitudinal_transfer
		adjusted_axle_load = max(adjusted_axle_load, base_axle_load * 0.1) # Minimum 10% of base load
		
		# ------------------------------
		# Store previous compression for damping calculations
		# ------------------------------
		var prev_compression_left: float = axle.suspension_compression_left
		var prev_compression_right: float = axle.suspension_compression_right
		
		# Calculate compression velocities for realistic damping
		var compression_velocity_left: float = 0.0
		var compression_velocity_right: float = 0.0
		if delta > 0.0:
			compression_velocity_left = (axle.suspension_compression_left - prev_compression_left) / delta
			compression_velocity_right = (axle.suspension_compression_right - prev_compression_right) / delta
		
		# ------------------------------
		# Individual wheel load distribution
		# ------------------------------
		var left_load: float = adjusted_axle_load * 0.5
		var right_load: float = adjusted_axle_load * 0.5
		
		# Apply lateral load transfer
		if lateral_acceleration != 0.0:
			var lateral_transfer: float = lateral_load_transfer * (1.0 if is_front_axle else 0.8) # Front transfers more
			if lateral_acceleration > 0.0: # Right turn
				right_load += lateral_transfer * 0.5
				left_load -= lateral_transfer * 0.5
			else: # Left turn
				left_load += lateral_transfer * 0.5
				right_load -= lateral_transfer * 0.5
		
		# Ensure minimum loads
		left_load = max(left_load, base_axle_load * 0.05)
		right_load = max(right_load, base_axle_load * 0.05)
		
		# ------------------------------
		# Advanced damping system
		# ------------------------------
		var damping_force_left: float = calculate_damping_force(compression_velocity_left, axle, true)
		var damping_force_right: float = calculate_damping_force(compression_velocity_right, axle, false)
		
		# ------------------------------
		# Process wheel forces (compatible with existing wheel setup)
		# ------------------------------
		var new_compression_left: float = 0.0
		var new_compression_right: float = 0.0
		
		if axle.wheels.size() >= 2:
			# Use existing wheel process_forces method
			new_compression_left = axle.wheels[0].process_forces(prev_compression_right, is_braking, delta)
			new_compression_right = axle.wheels[1].process_forces(prev_compression_left, is_braking, delta)
		
		# ------------------------------
		# Advanced anti-roll bar simulation
		# ------------------------------
		var compression_difference: float = new_compression_left - new_compression_right
		
		# Dynamic anti-roll strength based on conditions
		var dynamic_anti_roll: float = anti_roll_strength
		
		# Increase anti-roll in corners for better handling
		var cornering_factor: float = clamp(abs(lateral_acceleration) / 10.0, 0.0, 1.0)
		dynamic_anti_roll *= (1.0 + cornering_factor * 0.3)
		
		# Reduce anti-roll over bumps to maintain traction
		var bump_factor: float = clamp(abs(compression_difference) / 0.1, 0.0, 1.0)
		dynamic_anti_roll *= (1.0 - bump_factor * 0.2)
		
		# Speed-sensitive anti-roll (stiffer at higher speeds)
		var speed_factor: float = clamp(vehicle_speed / 50.0, 0.5, 1.5)
		dynamic_anti_roll *= speed_factor
		
		# Calculate anti-roll bar forces
		var arb_force: float = compression_difference * dynamic_anti_roll
		var arb_correction_left: float = -arb_force * 0.5
		var arb_correction_right: float = arb_force * 0.5
		
		# ------------------------------
		# Progressive anti-roll response
		# ------------------------------
		# Small differences get less correction (comfort)
		# Large differences get full correction (handling)
		var correction_factor: float = 1.0
		if abs(compression_difference) < 0.02:
			correction_factor = 0.3 # Soft response for small bumps
		elif abs(compression_difference) < 0.05:
			correction_factor = 0.7 # Medium response
		# Full response for large differences
		
		arb_correction_left *= correction_factor
		arb_correction_right *= correction_factor
		
		# ------------------------------
		# Apply anti-roll corrections with rate limiting
		# ------------------------------
		var max_correction_rate: float = 2.0 * delta # Limit how fast ARB can change
		
		arb_correction_left = clamp(arb_correction_left, -max_correction_rate, max_correction_rate)
		arb_correction_right = clamp(arb_correction_right, -max_correction_rate, max_correction_rate)
		
		# Apply final corrections
		axle.suspension_compression_left = clamp(new_compression_left + arb_correction_left, 0.0, 1.0)
		axle.suspension_compression_right = clamp(new_compression_right + arb_correction_right, 0.0, 1.0)
		
		# ------------------------------
		# Store calculated forces for potential use elsewhere
		# ------------------------------
		if axle.has_method("set_wheel_loads"):
			axle.set_wheel_loads(left_load, right_load)
		if axle.has_method("set_damping_forces"):
			axle.set_damping_forces(damping_force_left, damping_force_right)

# ------------------------------
# Helper functions for realistic physics
# ------------------------------

func calculate_lateral_acceleration() -> float:
	# Calculate lateral G-force based on velocity change
	if not global_transform or not angular_velocity:
		return 0.0
	
	var forward_dir: Vector3 = -global_transform.basis.z
	var right_dir: Vector3 = global_transform.basis.x
	var lateral_velocity: float = linear_velocity.dot(right_dir)
	var forward_speed: float = linear_velocity.dot(forward_dir)
	
	# Lateral acceleration from cornering
	return abs(forward_speed * angular_velocity.y)

func calculate_longitudinal_acceleration(delta: float) -> float:
	# Calculate forward/backward acceleration
	if delta <= 0.0:
		return 0.0
	
	var forward_dir: Vector3 = -global_transform.basis.z
	var current_forward_speed: float = linear_velocity.dot(forward_dir)
	
	# Store previous speed (you might want to add this as a class variable)
	var previous_speed: float = current_forward_speed # Simplified - you'd store this properly
	return (current_forward_speed - previous_speed) / delta

func calculate_aerodynamic_downforce(speed: float) -> float:
	# Realistic downforce calculation (increases with speed squared)
	var downforce_coefficient: float = 0.3 # Adjustable per vehicle
	var air_density: float = 1.225 # kg/m³ at sea level
	var frontal_area: float = 2.5 # m² - typical car frontal area
	
	return 0.5 * air_density * (speed * speed) * downforce_coefficient * frontal_area

func calculate_longitudinal_load_transfer(accel: float, total_weight: float) -> float:
	# Load transfers from rear to front under braking, front to rear under acceleration
	var wheelbase: float = 2.7 # meters - distance between axles
	var cg_height: float = 0.5 # meters - center of gravity height
	
	return (total_weight * accel * cg_height) / (wheelbase * 9.81)

func calculate_lateral_load_transfer(lateral_accel: float, total_weight: float) -> float:
	# Load transfers to outside wheels during cornering
	var track_width: float = 1.6 # meters - distance between left and right wheels
	var cg_height: float = 0.5 # meters - center of gravity height
	var roll_stiffness: float = 0.8 # Suspension roll resistance factor
	
	return (total_weight * lateral_accel * cg_height * roll_stiffness) / (track_width * 9.81)

func calculate_damping_force(compression_velocity: float, axle, is_left_wheel: bool) -> float:
	# Realistic damping force calculation
	var base_damping: float = 1500.0 # N⋅s/m - base damping coefficient
	var rebound_factor: float = 1.2 # Rebound damping is typically higher
	
	# Get axle-specific damping if available
	if axle.has_method("get_damping_coefficient"):
		base_damping = axle.get_damping_coefficient()
	
	var damping_force: float = 0.0
	if compression_velocity > 0.0: # Compression
		damping_force = base_damping * compression_velocity
	else: # Rebound
		damping_force = base_damping * rebound_factor * compression_velocity
	
	return damping_force
# Enhanced realistic vehicle stability system


func process_stability(delta: float) -> void:
	if not enable_stability:
		stability_active = false
		current_intervention_time = 0.0
		return
	
	stability_yaw_torque = 0.0
	var is_stability_on := false
	
	# Calculate vehicle dynamics
	var local_vel := global_transform.basis.inverse() * linear_velocity
	var speed := local_vel.length()
	var lateral_velocity := local_vel.x
	var longitudinal_velocity := local_vel.z
	
	if speed < 2.0:  # Minimal speed threshold
		stability_active = false
		current_intervention_time = 0.0
		return
	
	# Calculate slip angle (vehicle attitude vs velocity direction)
	var slip_angle := 0.0
	if abs(longitudinal_velocity) > 0.1:
		slip_angle = atan2(lateral_velocity, abs(longitudinal_velocity))
	
	# Calculate lateral acceleration (cornering G-force)
	var current_lateral_accel :float= lateral_velocity / max(delta, 0.001)
	lateral_acceleration_filter = lerp(lateral_acceleration_filter, current_lateral_accel, delta * 8.0)
	
	# Get wheel contact forces and calculate available grip
	var total_downforce := 0.0
	var wheel_count := get_wheel_contact_count()
	
	# Calculate load transfer and grip distribution
	for i in range(wheel_count):
		var wheel_load := get_wheel_normal_force(i)  # You'll need this function
		total_downforce += wheel_load
	
	if total_downforce < 100.0:  # Not enough grip for stability control
		stability_active = false
		return
	
	# Detect understeer/oversteer conditions
	var understeer_detected := false
	var oversteer_detected := false
	
	# Understeer: Front wheels losing grip, vehicle won't turn enough
	var front_slip := calculate_front_wheel_slip()
	if front_slip > lateral_grip_threshold and abs(slip_angle) < 0.2:
		understeer_detected = true
	
	# Oversteer: Rear losing grip, vehicle rotating too much
	var rear_slip := calculate_rear_wheel_slip()
	var yaw_rate := angular_velocity.y
	var expected_yaw_rate := (lateral_acceleration_filter / speed) if speed > 0.1 else 0.0
	
	if rear_slip > lateral_grip_threshold or abs(yaw_rate) > abs(expected_yaw_rate) * 1.3:
		oversteer_detected = true
	
	# Progressive intervention buildup
	var intervention_needed :float= understeer_detected or oversteer_detected or abs(slip_angle) > stability_yaw_engage_angle
	
	if intervention_needed:
		current_intervention_time = min(current_intervention_time + delta, stability_intervention_delay)
		var intervention_factor := current_intervention_time / stability_intervention_delay
		is_stability_on = true
		
		# ESC-style yaw moment control
		if oversteer_detected:
			apply_oversteer_correction(slip_angle, yaw_rate, intervention_factor, delta)
		elif understeer_detected:
			apply_understeer_correction(front_slip, intervention_factor, delta)
		else:
			# Standard slip angle correction
			apply_slip_angle_correction(slip_angle, yaw_rate, intervention_factor, delta)
			
	else:
		current_intervention_time = max(current_intervention_time - delta * 2.0, 0.0)
	
	# Traction control for individual wheels
	apply_traction_control(delta)
	
	# Roll stability (anti-rollover)
	apply_roll_stability_control(delta)
	
	stability_active = is_stability_on

func apply_oversteer_correction(slip_angle: float, yaw_rate: float, intervention: float, delta: float) -> void:
	# Brake outer front wheel to create counter-yaw moment
	var correction_torque := yaw_rate * vehicle_inertia.y * stability_yaw_strength * intervention
	correction_torque = clampf(correction_torque, 0.0, 2000.0)  # Limit max torque
	
	# Apply opposite to excessive rotation
	stability_yaw_torque = -correction_torque * signf(yaw_rate)
	apply_torque(global_transform.basis.y * stability_yaw_torque)
	
	# Simulate individual wheel braking (you'd implement per-wheel braking)
	var brake_side := 1 if yaw_rate > 0 else -1
	apply_individual_wheel_brake(brake_side, correction_torque * 0.3)

func apply_understeer_correction(front_slip: float, intervention: float, delta: float) -> void:
	# Reduce power and brake inner rear wheel
	var power_reduction := (front_slip - lateral_grip_threshold) * intervention
	apply_power_reduction(power_reduction)
	
	# Light rear braking to transfer weight forward
	var brake_torque := power_reduction * 500.0 * vehicle_inertia.y
	apply_individual_wheel_brake(-1, brake_torque * 0.2)

func apply_slip_angle_correction(slip_angle: float, yaw_rate: float, intervention: float, delta: float) -> void:
	if abs(slip_angle) > stability_yaw_engage_angle:
		var slip_correction :float= (abs(slip_angle) - stability_yaw_engage_angle) * stability_yaw_strength
		slip_correction *= vehicle_inertia.y * intervention
		
		# Modulate by speed - less intervention at higher speeds
		var speed_factor := 1.0 / (1.0 + linear_velocity.length() * 0.02)
		slip_correction *= speed_factor
		
		stability_yaw_torque = -slip_correction * signf(slip_angle)
		apply_torque(global_transform.basis.y * stability_yaw_torque)

func apply_traction_control(delta: float) -> void:
	# Check individual wheel spin
	for i in range(4):  # Assuming 4 wheels
		var wheel_speed := get_wheel_rotational_speed(i)
		var surface_speed := calculate_wheel_surface_speed(i)
		var slip_ratio := 0.0
		
		if abs(surface_speed) > 0.1:
			slip_ratio = (wheel_speed - surface_speed) / abs(surface_speed)
		
		# Limit wheel spin
		if slip_ratio > longitudinal_grip_threshold:
			apply_individual_wheel_brake(i, slip_ratio * 300.0)

func apply_roll_stability_control(delta: float) -> void:
	var roll_angle := asin(clampf(global_transform.basis.x.y, -1.0, 1.0))
	var roll_rate := angular_velocity.x
	
	# Detect potential rollover
	if abs(roll_angle) > 0.3 or abs(roll_rate) > 2.0:
		var stabilizing_torque := Vector3.ZERO
		
		# Counter-roll torque
		stabilizing_torque.x = -roll_angle * vehicle_inertia.x * stability_upright_spring
		stabilizing_torque.x -= roll_rate * stability_upright_damping
		
		# Emergency braking on rollover risk
		if abs(roll_angle) > 0.5:
			apply_emergency_braking(0.7)
		
		apply_torque(stabilizing_torque)

# Helper functions (implement based on your wheel system)
func calculate_front_wheel_slip() -> float:
	# Calculate slip based on front wheel lateral forces
	return 0.0  # Placeholder - implement based on your raycast wheels

func calculate_rear_wheel_slip() -> float:
	# Calculate slip based on rear wheel lateral forces
	return 0.0  # Placeholder

func get_wheel_normal_force(wheel_index: int) -> float:
	# Return normal force on specific wheel
	return 1000.0  # Placeholder

func get_wheel_rotational_speed(wheel_index: int) -> float:
	# Return wheel rotation speed
	return 0.0  # Placeholder

func calculate_wheel_surface_speed(wheel_index: int) -> float:
	# Calculate speed of contact patch
	return 0.0  # Placeholder

func apply_individual_wheel_brake(wheel_or_side: int, brake_force: float) -> void:
	# Apply braking to specific wheel or side
	pass  # Implement based on your braking system

func apply_power_reduction(reduction_factor: float) -> void:
	# Reduce engine power output
	pass  # Implement based on your engine system

func apply_emergency_braking(intensity: float) -> void:
	# Apply emergency braking
	pass  # Implement based on your braking system
# Enhanced realistic transmission system


func manual_shift(count: int) -> void:
	if not automatic_transmission:
		# Check for money shifts (dangerous over-revs on downshift)
		if count < 0 and would_cause_money_shift(count):
			# Allow but with consequences - realistic behavior
			apply_money_shift_damage(count)
		
		# Check clutch state for manual control

			
		shift(count)

func shift(count: int) -> void:
	if is_shifting:
		return
	
	var new_gear := current_gear + count
	
	# Enhanced gear validation
	if new_gear < -1 or new_gear >= gear_ratios.size():
		return
	
	# Realistic shift protection
	if not can_shift_to_gear(new_gear, count):
		return
	
	# Neutral handling with clutch behavior
	if new_gear == 0:
		current_gear = 0
		requested_gear = 0
		is_shifting = false
		# Clutch stays partially engaged in neutral for realistic feel
		clutch_amount = 0.3
		return
	
	# Store shift request
	requested_gear = new_gear
	is_up_shifting = count > 0
	
	# Calculate optimal shift timing based on current conditions
	var dynamic_shift_time := calculate_shift_time(count)
	complete_shift_delta_time = delta_time + dynamic_shift_time
	
	# Start clutch disengagement
	clutch_amount = 1.0
	is_shifting = true
	
	# Rev-matching preparation for downshifts
	if count < 0:
		prepare_rev_match(new_gear)

func calculate_shift_time(shift_direction: int) -> float:
	# Realistic shift timing based on conditions
	var base_time := shift_time
	
	# Faster shifts at higher RPM (racing technique)
	var rpm_factor := 1.0 - (motor_rpm - idle_rpm) / (max_rpm - idle_rpm) * 0.3
	rpm_factor = clampf(rpm_factor, 0.7, 1.0)
	
	# Downshifts slightly slower (heel-toe time)
	if shift_direction < 0:
		base_time *= 1.2
	
	# Load affects shift speed
	var load_factor := 1.0 + last_engine_load * 0.3
	
	return base_time * rpm_factor * load_factor

func can_shift_to_gear(new_gear: int, direction: int) -> bool:
	# Prevent reverse at speed
	if new_gear == -1 and local_velocity.z > 0.5:
		return false
	
	# Prevent money shifts (over-rev protection)
	if direction < 0:
		var predicted_rpm := calculate_predicted_engine_rpm(new_gear)
		if predicted_rpm > shift_protection_rpm_high:
			return false
	
	# Prevent lugging (under-rev protection)
	if direction > 0:
		var predicted_rpm := calculate_predicted_engine_rpm(new_gear)
		if predicted_rpm < shift_protection_rpm_low and speed > 5.0:
			return false
	
	# Skip-shifting validation (e.g., 2nd to 4th)
	if abs(direction) > 1 and not allow_skip_shifts():
		return false
	
	return true

func calculate_predicted_engine_rpm(target_gear: int) -> float:
	if target_gear <= 0:
		return motor_rpm
	
	var gear_ratio := gear_ratios[target_gear - 1] if target_gear > 0 else reverse_ratio
	var total_ratio := gear_ratio * final_drive
	
	var wheel_speed := speed / average_drive_wheel_radius if average_drive_wheel_radius > 0.0 else 0.0
	return wheel_speed * ANGULAR_VELOCITY_TO_RPM * total_ratio

func allow_skip_shifts() -> bool:
	# Allow skip shifting at lower loads/speeds
	return last_engine_load < 0.7 and speed < 30.0

func prepare_rev_match(target_gear: int) -> void:
	# Calculate target RPM for smooth downshift
	var target_rpm := calculate_predicted_engine_rpm(target_gear)
	target_rpm = clampf(target_rpm, idle_rpm, max_rpm * 0.95)
	
	# Apply rev-matching throttle blip
	if target_rpm > motor_rpm:
		var rev_match_amount := (target_rpm - motor_rpm) / max_rpm * rev_match_strength
		apply_rev_match_throttle(rev_match_amount)

func apply_rev_match_throttle(amount: float) -> void:
	# Simulate throttle blip for rev-matching
	# This would integrate with your engine system
	var blip_torque: float = amount * max_torque * 0.6
	# Apply momentary torque increase (you'd implement this in engine code)
	pass

func complete_shift() -> void:
	# Enhanced shift completion with realistic behavior
	
	# Clear brake assist when leaving reverse
	if current_gear == -1:
		brake_amount = 0.0
	
	# Calculate gear ratios
	var old_ratio := get_current_gear_ratio()
	var new_ratio := get_requested_gear_ratio()
	
	# Calculate RPM change and shift smoothness
	var old_engine_rpm := motor_rpm
	var target_rpm := calculate_target_rpm_for_shift()
	
	# Apply realistic RPM transition with flywheel inertia
	var rpm_difference := abs(target_rpm - motor_rpm)
	var shift_harshness := calculate_shift_harshness(rpm_difference)
	
	# Smooth or harsh transition based on RPM matching
	if rpm_difference > shift_shock_threshold:
		apply_harsh_shift_effects(rpm_difference)
		motor_rpm = lerpf(motor_rpm, target_rpm, 0.7)  # Sudden change
	else:
		motor_rpm = lerpf(motor_rpm, target_rpm, calculate_smooth_blend())
	
	# Apply engine braking on downshifts
	if requested_gear < current_gear and requested_gear > 0:
		apply_engine_braking(rpm_difference)
	
	# Commit gear change
	current_gear = requested_gear
	last_shift_delta_time = delta_time
	is_shifting = false
	is_up_shifting = false
	
	# Realistic torque interruption
	var torque_cut_duration := 0.1 + shift_harshness * 0.05
	apply_torque_cut(torque_cut_duration)
	
	# Update engine load tracking
	last_engine_load = calculate_current_engine_load()

func get_current_gear_ratio() -> float:
	if current_gear > 0:
		return gear_ratios[current_gear - 1] * final_drive
	elif current_gear == -1:
		return reverse_ratio * final_drive
	return 0.0

func get_requested_gear_ratio() -> float:
	if requested_gear > 0:
		return gear_ratios[requested_gear - 1] * final_drive
	elif requested_gear == -1:
		return reverse_ratio * final_drive
	return 0.0

func calculate_target_rpm_for_shift() -> float:
	var wheel_angular_velocity := speed / average_drive_wheel_radius if average_drive_wheel_radius > 0.0 else 0.0
	var target_rpm := wheel_angular_velocity * ANGULAR_VELOCITY_TO_RPM * get_requested_gear_ratio()
	
	# Account for flywheel inertia in RPM changes
	if requested_gear == 0:
		target_rpm = lerpf(motor_rpm, idle_rpm, 0.3)
	else:
		target_rpm = clampf(target_rpm, idle_rpm, max_rpm * 1.05)
	
	return target_rpm

func calculate_shift_harshness(rpm_diff: float) -> float:
	return clampf(rpm_diff / shift_shock_threshold, 0.0, 1.0)

func calculate_smooth_blend() -> float:
	# Smoother blending based on gear change type
	if requested_gear == 0:
		return 0.15  # Gentle to neutral
	elif requested_gear < current_gear:
		return 0.4   # Moderate for downshift
	else:
		return 0.25  # Standard for upshift

func apply_harsh_shift_effects(rpm_difference: float) -> void:
	# Simulate driveline shock from bad shifts
	var shock_force := rpm_difference / max_rpm * 2000.0
	
	# Apply momentary chassis disturbance (you'd implement this)
	# Could affect suspension, cause wheel hop, etc.
	
	# Reduce clutch life, cause wear (for advanced simulation)
	gear_whine_factor = clampf(rpm_difference / 1000.0, 0.0, 1.0)

func apply_engine_braking(rpm_increase: float) -> void:
	# Realistic engine braking on downshifts
	var braking_force := rpm_increase * engine_brake_factor
	braking_force = clampf(braking_force, 0.0, 1500.0)
	
	# Apply as negative torque (you'd integrate this with your wheel system)
	var engine_brake_torque := -braking_force * get_requested_gear_ratio()
	# This would be applied through your drivetrain
	pass

func apply_torque_cut(duration: float) -> void:
	# Realistic torque interruption during shifts
	torque_output = 0.0
	# You might want to implement a timer system for torque restoration
	# create_tween().tween_method(restore_torque, 0.0, 1.0, duration)

func calculate_current_engine_load() -> float:
	# Calculate current engine load for realistic behavior
	var load := throttle_input * (motor_rpm / max_rpm)
	return clampf(load, 0.0, 1.0)

func would_cause_money_shift(shift_count: int) -> bool:
	# Check if downshift would cause dangerous over-rev
	var target_gear := current_gear + shift_count
	if target_gear <= 0:
		return false
	
	var predicted_rpm := calculate_predicted_engine_rpm(target_gear)
	return predicted_rpm > max_rpm * 1.15  # 15% over redline = money shift

func apply_money_shift_damage(shift_count: int) -> void:
	# Simulate consequences of money shifting
	var over_rev_amount := calculate_predicted_engine_rpm(current_gear + shift_count) - max_rpm
	if over_rev_amount > 0:
		# Could reduce engine power, cause misfires, etc.
		# This is where you'd implement engine damage simulation
		pass

# Enhanced realistic wheel contact system


func get_wheel_contact_count() -> float:
	var contact_sum := 0.0
	var total_vehicle_weight := mass * 9.81
	var current_g_force := calculate_current_g_forces()
	
	# Initialize arrays if needed
	if wheel_load_distribution.size() != wheel_array.size():
		wheel_load_distribution.resize(wheel_array.size())
		contact_patch_quality.resize(wheel_array.size())
		surface_grip_multipliers.resize(wheel_array.size())
	
	# Calculate dynamic load distribution
	calculate_dynamic_wheel_loads(current_g_force, total_vehicle_weight)
	
	for i in range(wheel_array.size()):
		var wheel = wheel_array[i]
		var wheel_contact_value := 0.0
		
		if wheel.is_colliding():
			var contact_normal := wheel.get_collision_normal()
			var contact_point := wheel.get_collision_point()
			
			# Enhanced surface normal evaluation
			var surface_angle_factor := evaluate_surface_contact_quality(contact_normal)
			if surface_angle_factor <= 0.0:
				wheel_load_distribution[i] = 0.0
				contact_patch_quality[i] = 0.0
				continue
			
			# Get suspension compression with realistic scaling
			var compression := get_realistic_suspension_compression(wheel, i)
			if compression < suspension_travel_threshold:
				wheel_load_distribution[i] = 0.0
				contact_patch_quality[i] = 0.0
				continue
			
			# Calculate contact patch quality based on multiple factors
			var patch_quality := calculate_contact_patch_quality(wheel, i, compression, surface_angle_factor)
			contact_patch_quality[i] = patch_quality
			
			# Surface grip analysis
			surface_grip_multipliers[i] = analyze_surface_grip(wheel, contact_point)
			
			# Load-dependent contact effectiveness
			var load_factor := calculate_load_dependent_contact(i, total_vehicle_weight)
			
			# Combine all factors for final contact value
			wheel_contact_value = compression * patch_quality * surface_angle_factor * load_factor
			wheel_contact_value = clampf(wheel_contact_value, 0.0, 1.0)
			
		else:
			# Wheel is airborne
			wheel_load_distribution[i] = 0.0
			contact_patch_quality[i] = 0.0
			surface_grip_multipliers[i] = 0.0
		
		contact_sum += wheel_contact_value
	
	# Apply realistic contact sum processing
	contact_sum = process_contact_sum_realism(contact_sum)
	
	# Detect airborne/landing states
	detect_airborne_and_landing_states(contact_sum)
	
	last_contact_count = contact_sum
	return contact_sum

func calculate_current_g_forces() -> Vector3:
	# Calculate current G-forces affecting the vehicle
	var local_acceleration := global_transform.basis.inverse() * (linear_velocity - get_last_velocity()) / get_physics_process_delta_time()
	return local_acceleration / 9.81  # Convert to G-forces

func get_last_velocity() -> Vector3:
	# You'd need to track this - placeholder for now
	return Vector3.ZERO

func calculate_dynamic_wheel_loads(g_forces: Vector3, total_weight: float) -> void:
	# Realistic load transfer calculation
	var wheelbase := get_wheelbase_distance()
	var track_width := get_track_width()
	var cg_height := center_of_mass.y
	
	# Base static load (25% per wheel for 4 wheels)
	var static_load_per_wheel := total_weight / wheel_array.size()
	
	for i in range(wheel_array.size()):
		var wheel_position := get_wheel_position_relative_to_cg(i)
		var load := static_load_per_wheel
		
		# Longitudinal load transfer (braking/acceleration)
		if wheelbase > 0.0:
			var longitudinal_transfer := g_forces.z * total_weight * (cg_height / wheelbase)
			if is_front_wheel(i):
				load += longitudinal_transfer  # Front gets more under braking
			else:
				load -= longitudinal_transfer  # Rear loses under braking
		
		# Lateral load transfer (cornering)
		if track_width > 0.0:
			var lateral_transfer := g_forces.x * total_weight * (cg_height / track_width)
			if is_left_wheel(i):
				load -= lateral_transfer * signf(g_forces.x)
			else:
				load += lateral_transfer * signf(g_forces.x)
		
		wheel_load_distribution[i] = maxf(load, 0.0)

func evaluate_surface_contact_quality(normal: Vector3) -> float:
	# More nuanced surface angle evaluation
	var upward_dot := normal.dot(Vector3.UP)
	
	# Allow banking and mild slopes
	if upward_dot < minimum_contact_normal_dot:
		return 0.0
	
	# Optimal contact at perfectly flat surfaces
	var angle_quality := 1.0
	if upward_dot < 0.9:  # Not perfectly flat
		# Gradual degradation on slopes
		angle_quality = smoothstep(minimum_contact_normal_dot, 0.95, upward_dot)
	
	return angle_quality

func get_realistic_suspension_compression(wheel: RayCast3D, wheel_index: int) -> float:
	var compression := 1.0
	
	if wheel.has_method("get_suspension_compression"):
		compression = wheel.get_suspension_compression()
	else:
		# Fallback: calculate from raycast distance
		var max_distance := wheel.target_position.length()
		var current_distance := wheel.get_collision_point().distance_to(wheel.global_position)
		compression = 1.0 - (current_distance / max_distance)
	
	# Non-linear suspension response (more realistic)
	compression = ease_out_quad(clampf(compression, 0.0, 1.0))
	
	return compression

func ease_out_quad(x: float) -> float:
	return 1.0 - (1.0 - x) * (1.0 - x)

func calculate_contact_patch_quality(wheel: RayCast3D, wheel_index: int, compression: float, surface_factor: float) -> float:
	var base_quality := compression * surface_factor
	
	# Tire pressure effects (optimal around 0.7-0.9 compression)
	var pressure_factor := 1.0
	if compression < 0.3:
		pressure_factor = compression / 0.3  # Under-inflated/too soft
	elif compression > 0.95:
		pressure_factor = 1.0 - (compression - 0.95) / 0.05  # Over-compressed
	
	# Speed-dependent contact patch changes
	var speed_factor := 1.0
	var wheel_speed := calculate_wheel_surface_speed(wheel_index)
	if wheel_speed > 30.0:  # High speed reduces contact quality
		speed_factor = 1.0 - clampf((wheel_speed - 30.0) / 100.0, 0.0, 0.3)
	
	return base_quality * pressure_factor * speed_factor

func analyze_surface_grip(wheel: RayCast3D, contact_point: Vector3) -> float:
	var grip_multiplier := 1.0
	
	# Get surface material if available
	var collider := wheel.get_collider()
	if collider and collider.has_method("get_surface_material"):
		var surface_type = collider.get_surface_material()
		grip_multiplier = get_surface_grip_coefficient(surface_type)
	else:
		# Fallback: analyze collision object for surface hints
		grip_multiplier = estimate_surface_grip_from_collider(collider)
	
	# Temperature effects (simplified)
	grip_multiplier *= calculate_tire_temperature_factor(wheel)
	
	return clampf(grip_multiplier, 0.1, 2.0)

func get_surface_grip_coefficient(surface_type: String) -> float:
	# Realistic surface grip values
	match surface_type.to_lower():
		"asphalt", "tarmac":
			return 1.0
		"concrete":
			return 0.95
		"gravel":
			return 0.6
		"grass":
			return 0.4
		"sand":
			return 0.3
		"ice":
			return 0.15
		"snow":
			return 0.25
		_:
			return 0.8  # Default

func estimate_surface_grip_from_collider(collider: Node) -> float:
	# Estimate grip from object name or groups
	if not collider:
		return 0.8
	
	var name := collider.name.to_lower()
	if "ice" in name or "snow" in name:
		return 0.2
	elif "grass" in name or "dirt" in name:
		return 0.5
	elif "gravel" in name:
		return 0.6
	else:
		return 1.0  # Assume good grip

func calculate_tire_temperature_factor(wheel: RayCast3D) -> float:
	# Simplified tire temperature model
	# In a full sim, this would track actual tire temps
	var base_temp_factor := 1.0
	
	# High speeds generate heat, affecting grip
	var speed := linear_velocity.length()
	if speed > 50.0:
		var heat_factor := (speed - 50.0) / 100.0
		base_temp_factor = 1.0 + clampf(heat_factor * 0.1, 0.0, 0.15)  # Slight grip increase when warm
	
	return base_temp_factor

func calculate_load_dependent_contact(wheel_index: int, total_weight: float) -> float:
	# Tire load affects contact patch
	var wheel_load := wheel_load_distribution[wheel_index]
	var average_load := total_weight / wheel_array.size()
	
	if average_load <= 0:
		return 0.0
	
	var load_ratio := wheel_load / average_load
	
	# Optimal load around 1.0, degradation at extremes
	var load_factor := 1.0
	if load_ratio < 0.3:
		load_factor = load_ratio / 0.3  # Low load = poor contact
	elif load_ratio > 2.0:
		load_factor = 1.0 - clampf((load_ratio - 2.0) / 2.0, 0.0, 0.5)  # Overload = degraded contact
	
	return load_factor

func process_contact_sum_realism(raw_sum: float) -> float:
	# Apply realistic contact sum processing
	var processed_sum := raw_sum
	
	# Hysteresis for ground contact detection (prevents oscillation)
	if last_contact_count > airborne_threshold and processed_sum < airborne_threshold:
		# Require slightly lower threshold to declare airborne
		if processed_sum > airborne_threshold * 0.8:
			processed_sum = airborne_threshold + 0.01
	
	return processed_sum

func detect_airborne_and_landing_states(contact_sum: float) -> void:
	var was_airborne := last_contact_count < airborne_threshold
	var is_airborne := contact_sum < airborne_threshold
	
	if was_airborne and not is_airborne:
		# Landing detected
		on_vehicle_landing(contact_sum)
	elif not was_airborne and is_airborne:
		# Takeoff detected
		on_vehicle_airborne()

func on_vehicle_landing(contact_strength: float) -> void:
	# Handle landing effects
	var landing_force := calculate_landing_impact()
	if landing_force > 5.0:  # Hard landing
		apply_landing_suspension_effects(landing_force)

func on_vehicle_airborne() -> void:
	# Handle airborne state
	# Could affect aerodynamics, disable traction control, etc.
	pass

func calculate_landing_impact() -> float:
	# Calculate landing force based on vertical velocity
	return abs(linear_velocity.y) * mass / 1000.0

func apply_landing_suspension_effects(impact_force: float) -> void:
	# Apply realistic landing effects to suspension/chassis
	# This would integrate with your suspension system
	pass

# Helper functions (implement based on your vehicle setup)
func get_wheelbase_distance() -> float:
	return 2.8  # Placeholder - measure your vehicle

func get_track_width() -> float:
	return 1.6  # Placeholder - measure your vehicle

func get_wheel_position_relative_to_cg(wheel_index: int) -> Vector3:
	return Vector3.ZERO  # Implement based on your wheel positions

func is_front_wheel(wheel_index: int) -> bool:
	return wheel_index < 2  # Assuming first 2 wheels are front

func is_left_wheel(wheel_index: int) -> bool:
	return wheel_index % 2 == 0  # Assuming even indices are left


# Enhanced realistic wheel slip detection system


func get_is_a_wheel_slipping() -> bool:
	# Initialize arrays if needed
	if wheel_slip_states.size() != drive_wheels.size():
		wheel_slip_states.resize(drive_wheels.size())
		wheel_slip_ratios.resize(drive_wheels.size())
		wheel_lateral_slip.resize(drive_wheels.size())
		surface_slip_multipliers.resize(drive_wheels.size())
		wheel_slip_states.fill(0)
	
	var any_wheel_slipping := false
	var vehicle_speed := linear_velocity.length()
	
	for i in range(drive_wheels.size()):
		var wheel = drive_wheels[i]
		var slip_detected := false
		
		if not wheel.is_colliding():
			wheel_slip_states[i] = 0  # No slip if airborne
			wheel_slip_ratios[i] = 0.0
			wheel_lateral_slip[i] = 0.0
			continue
		
		# Validate wheel has required methods/data
		if not wheel.has_method("get_spin") or not wheel.has_meta("radius"):
			continue
		
		# Calculate surface-specific slip behavior
		surface_slip_multipliers[i] = calculate_surface_slip_factor(wheel)
		
		# Get wheel dynamics
		var wheel_radius: float = wheel.get_meta("radius")
		var wheel_angular_velocity: float = wheel.get("spin")
		var wheel_transform :float= wheel.global_transform
		
		# Calculate longitudinal slip (drive/brake slip)
		var longitudinal_slip := calculate_longitudinal_slip(wheel, i, wheel_radius, wheel_angular_velocity, vehicle_speed)
		wheel_slip_ratios[i] = longitudinal_slip
		
		# Calculate lateral slip (cornering slip)
		var lateral_slip := calculate_lateral_slip(wheel, i, vehicle_speed)
		wheel_lateral_slip[i] = lateral_slip
		
		# Determine slip state with realistic thresholds
		var slip_state := determine_wheel_slip_state(longitudinal_slip, lateral_slip, i)
		wheel_slip_states[i] = slip_state
		
		# Check if this wheel is slipping
		if slip_state > 0:
			slip_detected = true
			any_wheel_slipping = true
	
	return any_wheel_slipping

func calculate_longitudinal_slip(wheel: Node, wheel_index: int, radius: float, angular_vel: float, vehicle_speed: float) -> float:
	# Get wheel's forward direction in global space
	var wheel_forward :Vector3= wheel.global_transform.basis.z.normalized()
	
	# Project vehicle velocity onto wheel's forward direction
	var wheel_longitudinal_velocity := linear_velocity.dot(wheel_forward)
	
	# Calculate wheel surface speed (circumferential velocity)
	var wheel_surface_speed := angular_vel * radius
	
	# Calculate slip ratio using proper tire physics formula
	var slip_ratio := 0.0
	
	if abs(wheel_longitudinal_velocity) < 0.1:
		# Low speed/stationary case - use wheel spin as reference
		if abs(wheel_surface_speed) > 0.1:
			slip_ratio = abs(wheel_surface_speed) / max(abs(wheel_surface_speed), 0.1)
		else:
			slip_ratio = 0.0
	else:
		# Standard slip ratio calculation
		# Positive = wheel spin (acceleration), Negative = wheel lock (braking)
		var reference_speed := max(abs(wheel_longitudinal_velocity), abs(wheel_surface_speed))
		slip_ratio = (wheel_surface_speed - wheel_longitudinal_velocity) / reference_speed
	
	# Apply surface and tire compound corrections
	slip_ratio *= surface_slip_multipliers[wheel_index] * tire_compound_factor
	
	return abs(slip_ratio)  # Return absolute value for threshold comparison

func calculate_lateral_slip(wheel: Node, wheel_index: int, vehicle_speed: float) -> float:
	if vehicle_speed < 2.0:  # Skip lateral calculation at very low speeds
		return 0.0
	
	# Get wheel's lateral direction (sideways)
	var wheel_right :Vector3= wheel.global_transform.basis.x.normalized()
	
	# Calculate lateral velocity component
	var lateral_velocity := linear_velocity.dot(wheel_right)
	
	# Calculate slip angle (lateral slip angle in radians)
	var wheel_forward :Vector3= wheel.global_transform.basis.z.normalized()
	var longitudinal_velocity := linear_velocity.dot(wheel_forward)
	
	var slip_angle := 0.0
	if abs(longitudinal_velocity) > 0.1:
		slip_angle = atan2(abs(lateral_velocity), abs(longitudinal_velocity))
	
	# Convert to slip ratio equivalent for consistent threshold handling
	var lateral_slip_ratio := sin(slip_angle)
	
	# Apply load sensitivity (more loaded wheels slip less laterally)
	var load_factor := calculate_wheel_load_factor(wheel_index)
	lateral_slip_ratio /= load_factor
	
	return abs(lateral_slip_ratio)

func calculate_surface_slip_factor(wheel: Node) -> float:
	# Get surface properties if available
	var surface_factor := 1.0
	var collider: CollisionObject3D = wheel.get_collider()
	
	if collider:
		var surface_name := ""
		if collider.has_method("get_surface_type"):
			surface_name = collider.get_surface_type().to_lower()
		else:
			surface_name = collider.name.to_lower()
		
		# Different surfaces have different slip characteristics
		match surface_name:
			"ice", "snow":
				surface_factor = 2.5  # Much easier to slip on ice
			"wet", "rain":
				surface_factor = 1.8  # Wet surfaces slip easier
			"gravel", "dirt":
				surface_factor = 1.4  # Loose surfaces
			"grass":
				surface_factor = 1.6  # Low grip surface
			"asphalt", "tarmac":
				surface_factor = 1.0  # Reference surface
			"concrete":
				surface_factor = 0.9  # Slightly better grip
			_:
				surface_factor = 1.0
	
	return surface_factor

func calculate_wheel_load_factor(wheel_index: int) -> float:
	# Use load distribution from contact system if available
	if wheel_load_distribution.size() > wheel_index:
		var wheel_load := wheel_load_distribution[wheel_index]
		var average_load := mass * 9.81 / drive_wheels.size()
		
		if average_load > 0:
			var load_ratio := wheel_load / average_load
			# More load = better grip (up to a point)
			return clampf(sqrt(load_ratio), 0.5, 1.5)
	
	return 1.0  # Default if no load data available

func determine_wheel_slip_state(longitudinal_slip: float, lateral_slip: float, wheel_index: int) -> int:
	var current_state := wheel_slip_states[wheel_index]
	var combined_slip := sqrt(longitudinal_slip * longitudinal_slip + lateral_slip * lateral_slip)
	
	# Apply hysteresis to prevent oscillation
	var long_threshold := longitudinal_slip_threshold
	var lat_threshold := lateral_slip_threshold
	var combined_threshold := combined_slip_threshold
	
	if current_state > 0:  # Currently slipping - require lower threshold to stop slipping
		long_threshold -= slip_hysteresis
		lat_threshold -= slip_hysteresis
		combined_threshold -= slip_hysteresis
	
	# Determine slip state
	if longitudinal_slip > lockup_threshold:
		return 3  # Wheel lockup (severe braking slip)
	elif longitudinal_slip > spin_threshold:
		return 2  # Wheel spin (severe acceleration slip)
	elif combined_slip > combined_threshold or longitudinal_slip > long_threshold or lateral_slip > lat_threshold:
		return 1  # General slipping
	else:
		return 0  # Good grip
	
func get_wheel_slip_info() -> Dictionary:
	# Additional function for detailed slip information
	var slip_info := {
		"any_slipping": false,
		"wheel_states": wheel_slip_states.duplicate(),
		"longitudinal_slip": wheel_slip_ratios.duplicate(),
		"lateral_slip": wheel_lateral_slip.duplicate(),
		"peak_grip_wheels": [],
		"locked_wheels": [],
		"spinning_wheels": []
	}
	
	for i in range(wheel_slip_states.size()):
		var state := wheel_slip_states[i]
		if state > 0:
			slip_info.any_slipping = true
		
		match state:
			1:
				# Check if wheel is at peak grip (optimal slip ratio)
				if abs(wheel_slip_ratios[i] - peak_grip_slip_ratio) < 0.02:
					slip_info.peak_grip_wheels.append(i)
			2:
				slip_info.spinning_wheels.append(i)
			3:
				slip_info.locked_wheels.append(i)
	
	return slip_info

func get_individual_wheel_slip_ratio(wheel_index: int) -> float:
	# Get slip ratio for specific wheel
	if wheel_index >= 0 and wheel_index < wheel_slip_ratios.size():
		return wheel_slip_ratios[wheel_index]
	return 0.0

func get_worst_slipping_wheel() -> int:
	# Find wheel with highest slip ratio
	var worst_wheel := -1
	var worst_slip := 0.0
	
	for i in range(wheel_slip_ratios.size()):
		var combined_slip := sqrt(wheel_slip_ratios[i] * wheel_slip_ratios[i] + wheel_lateral_slip[i] * wheel_lateral_slip[i])
		if combined_slip > worst_slip:
			worst_slip = combined_slip
			worst_wheel = i
	
	return worst_wheel

func is_wheel_at_peak_grip(wheel_index: int) -> bool:
	# Check if wheel is operating at optimal slip ratio
	if wheel_index >= 0 and wheel_index < wheel_slip_ratios.size():
		var slip := wheel_slip_ratios[wheel_index]
		return abs(slip - peak_grip_slip_ratio) < 0.02
	return false

func get_traction_circle_utilization(wheel_index: int) -> float:
	# Calculate how much of the tire's grip circle is being used
	if wheel_index >= 0 and wheel_index < wheel_slip_ratios.size():
		var long_slip := wheel_slip_ratios[wheel_index]
		var lat_slip := wheel_lateral_slip[wheel_index]
		var combined := sqrt(long_slip * long_slip + lat_slip * lat_slip)
		
		# Normalize to peak grip capability
		return clampf(combined / peak_grip_slip_ratio, 0.0, 2.0)
	return 0.0

func set_tire_compound(compound_factor: float) -> void:
	# Allow adjustment of tire compound characteristics
	# Soft compound = lower thresholds (easier to slip but more grip at peak)
	# Hard compound = higher thresholds (harder to slip but less peak grip)
	tire_compound_factor = clampf(compound_factor, 0.5, 2.0)
	
	# Adjust thresholds based on compound
	longitudinal_slip_threshold = 0.12 * tire_compound_factor
	lateral_slip_threshold = 0.08 * tire_compound_factor
	peak_grip_slip_ratio = 0.08 * (2.0 - tire_compound_factor)  # Softer = lower peak slip


func get_drivetrain_spin() -> float:
	if drive_wheels.size() == 0:
		return 0.0
	
	var total_weight := 0.0
	var weighted_spin := 0.0
	
	for wheel in drive_wheels:
		# Skip wheels not in contact with the ground
		if not wheel.is_colliding():
			continue
		
		# Make sure wheel has radius defined
		var wheel_radius := 0.33
		if wheel.has_meta("radius"):
			wheel_radius = wheel.get("radius")
		
		# Approximate load on wheel: proportional to vertical suspension compression
		var wheel_load := 1.0
		if wheel.has_meta("suspension_load"):
			wheel_load = wheel.get("suspension_load")
		
		weighted_spin += wheel.spin * wheel_radius * wheel_load
		total_weight += wheel_radius * wheel_load
	
	if total_weight <= 0.0:
		return 0.0
	
	return weighted_spin / total_weight


func get_drive_wheels_reaction_torque() -> float:
	if drive_wheels.size() == 0:
		return 0.0
	
	var total_torque := 0.0
	
	for wheel in drive_wheels:
		# Skip wheels not on the ground
		if not wheel.is_colliding():
			continue
		
		# Get vertical load
		var wheel_load := 1.0
		if wheel.has_meta("suspension_load"):
			wheel_load = wheel.get("suspension_load")
		
		# Use longitudinal force to compute torque contribution
		var longitudinal_force := 0.0
		if wheel.has_meta("longitudinal_force"):
			longitudinal_force = wheel.get("longitudinal_force")
		
		# Effective torque = longitudinal force * effective radius * load factor
		var wheel_radius := 0.33
		if wheel.has_meta("radius"):
			wheel_radius = wheel.get("radius")
		
		total_torque += longitudinal_force * wheel_radius * wheel_load
	
	return total_torque


func get_gear_ratio(gear : int) -> float:
	var base_ratio := 0.0
	
	if gear > 0:
		base_ratio = gear_ratios[gear - 1] * final_drive
	elif gear == -1:
		base_ratio = -reverse_ratio * final_drive
	else:
		base_ratio = 0.0  # Neutral
	
	# Apply realistic transmission efficiency (0.95–0.99 typical)
	var efficiency := 0.97
	if base_ratio < 0:
		# Reverse usually slightly less efficient
		efficiency = 0.95
	
	return base_ratio * efficiency

func get_torque_at_rpm(lookup_rpm : float) -> float:
	# Clamp RPM to engine limits
	var clamped_rpm : float = clamp(lookup_rpm, idle_rpm, max_rpm)
	
	# Normalize RPM for torque curve sampling
	var rpm_factor : float = clamp(clamped_rpm / max_rpm, 0.0, 1.0)
	
	# Sample torque curve
	var torque_factor : float = torque_curve.sample_baked(rpm_factor)
	
	# Return torque scaled by max torque
	return torque_factor * max_torque

func get_max_steering_slip_angle() -> float:
	var max_slip : float = 0.0
	for wheel in front_axle.wheels:
		# Use the absolute slip but preserve the original sign for direction
		var wheel_slip : float = wheel.slip_vector.x
		if abs(wheel_slip) > abs(max_slip):
			max_slip = wheel_slip
	return max_slip

func calculate_average_tire_friction(weight : float, surface : String) -> float:
	var total_friction : float = 0.0
	var num_wheels := wheel_array.size()
	if num_wheels == 0:
		return 0.0

	for wheel in wheel_array:
		# Realistic friction decreases slightly under high load; you can scale it by tire width and contact patch if available
		var wheel_load := weight / num_wheels
		var wheel_friction := wheel.get_friction(wheel_load, surface)
		total_friction += wheel_friction

	return total_friction / num_wheels

func calculate_brake_force() -> void:
	# Calculate average tire friction
	var friction := calculate_average_tire_friction(vehicle_mass * 9.8, "Road")
	
	# Static front/rear weight distribution ratio (realistic, e.g., 55% front / 45% rear)
	var front_ratio := 0.55
	var rear_ratio := 0.55
	
	# Total brake forces
	var total_front_brake := friction * braking_grip_multiplier * front_ratio * vehicle_mass * 9.8
	var total_rear_brake := friction * braking_grip_multiplier * rear_ratio * vehicle_mass * 9.8
	
	# Average per wheel
	var front_wheel_count := wheel_array.size() / 2
	var rear_wheel_count := wheel_array.size() - front_wheel_count
	
	# Assign calculated forces to global brake variables
	max_brake_force = (total_front_brake + total_rear_brake) / wheel_array.size()
	max_handbrake_force = (total_rear_brake * 0.1) / rear_wheel_count

func calculate_center_of_gravity(front_distribution : float) -> Vector3:
	# Midpoints of axles
	var front_axle_pos := front_left_wheel.position.lerp(front_right_wheel.position, 0.5)
	var rear_axle_pos := rear_left_wheel.position.lerp(rear_right_wheel.position, 0.5)

	# Interpolate horizontally based on front/rear weight distribution
	var com_horizontal := rear_axle_pos.lerp(front_axle_pos, front_distribution)

	# Keep vertical component around midpoint between front and rear axles
	var com_vertical := (front_axle_pos.y + rear_axle_pos.y) * 0.5

	return Vector3(com_horizontal.x, com_vertical, com_horizontal.z)


func calculate_spring_rate(weight : float, spring_length : float, resting_ratio : float) -> float:
	var corrected_resting_ratio := (spring_length * resting_ratio) / spring_length
	var target_compression := spring_length * corrected_resting_ratio * 1000.0
	return weight / target_compression

func calculate_damping(weight : float, spring_rate : float, damping_ratio : float) -> float:
	return damping_ratio * 2.0 * sqrt(spring_rate * weight) * 0.01

func calculate_axle_spring_force(compression : float, spring_length : float, spring_rate : float) -> float:
	# Reduce maximum effective compression per wheel to prevent violent launches
	compression = clamp(compression, 0.0, 0.15)  # 15% of spring length max per bump
	var displacement := compression * spring_length
	
	# Apply a softening factor for extreme displacements
	var force := spring_rate * displacement
	if compression > 0.1:
		force *= 0.8  # soften the spring beyond 10% travel

	return force



#func apply_anti_roll(axle : Axle, delta : float) -> void:
	# Compute compression difference between left and right wheels
	#var left_compression := axle.suspension_compression_left
	#var right_compression := axle.suspension_compression_right
	#var roll_diff := left_compression - right_compression
	
	# Compute stabilizing torque
	#var roll_torque := roll_diff * anti_roll_stiffness
	#var roll_damping := (left_compression - right_compression) * anti_roll_damping * delta
	
	# Apply counteracting forces to wheels
	#axle.wheels[0].apply_vertical_force(-roll_torque - roll_damping)
	#axle.wheels[1].apply_vertical_force(roll_torque - roll_damping)
