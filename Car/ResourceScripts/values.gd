## Tuning data for a single car: chassis, suspension, steering, brakes,
## driver assists, engine, drivetrain, wheels, tires and debug options.
##
## This resource only stores values. It contains no simulation logic, so the
## vehicle scripts (for example the engine and drivetrain script) read these
## properties and do the actual calculations. Create a [code].tres[/code] file
## of this type to define a car, and edit it in the Inspector, where the
## properties are grouped by system.
##
## [b]Per-wheel arrays[/b] use the order 0 = front left (FL), 1 = front right
## (FR), 2 = rear left (RL), 3 = rear right (RR).
##
## [b]Units[/b] are SI unless stated otherwise: meters, kilograms, seconds,
## newton-meters for torque, kg·m² for inertia, and RPM for engine speed.
## Where a unit is not obvious from the name it is described on the property.
##
## [b]Note:[/b] the empty functions such as [method car] and [method suspension]
## do nothing. They are placeholders that mark the start of each section of the
## file.
class_name CarValues
extends Resource

@export_group("Car")

## Distance between the front and rear axles, in meters.
@export var wheel_base = 2.265 
## Distance between the left and right wheels of an axle (track width), in
## meters.
@export var track = 1.41

## Placeholder marking the "Car" section. Does nothing.
func car():
	pass

@export_group("Suspension")

## Natural frequency of each corner's spring, in Hz. Used to derive the spring
## stiffness for that wheel.
@export var ride_frequency = [1.0, 1.0, 1.0, 1.0]
## Maximum suspension compression of each wheel, in meters.
@export var max_compression = [0.11, 0.11, 0.11, 0.11]
## Share of the car's weight carried by each wheel. The four values are
## fractions and should add up to 1.0.
@export var weight_distribution = [0.25, 0.25, 0.25, 0.25]
## Damping ratio of each corner's damper. A value of 1.0 is critical damping;
## lower values are underdamped and bouncier.
@export var damper_ratio = [0.69, 0.69, 0.75, 0.75]
## Enables the rear anti-roll bar when true.
@export var rear_antiroll_bar = true
## Enables the front anti-roll bar when true.
@export var front_antiroll_bar = true
## Stiffness of the front anti-roll bar. Has no effect if
## [member front_antiroll_bar] is false.
@export var front_antiroll_bar_stiffness = 8500.0
## Stiffness of the rear anti-roll bar. Has no effect if
## [member rear_antiroll_bar] is false.
@export var rear_antiroll_bar_stiffness = 2500.0
## Exponent applied to suspension velocity when computing damping force.
## 1.0 gives ordinary linear damping.
@export var velocity_exponent = 1.0

## Placeholder marking the "Suspension" section. Does nothing.
func suspension():
	pass

@export_group("Steering")

## Maximum steering angle of the front tires, in degrees.
@export var max_tire_turn_angle = 38.0
## Total travel of the steering wheel from one lock to the other, in degrees.
@export var steering_wheel_travel = 1000.0
## Steering ratio: how many degrees the steering wheel turns for one degree of
## tire turn.
@export var steering_ratio = 15.0
## How quickly the tires move toward the requested steering angle. Higher
## values respond faster.
@export var tire_turn_speed = 3.5
## Stiffness of the steering, which affects how strongly the wheel returns to
## center and how it feels under load.
@export var steering_stiffness = 720.0
## Coefficient that scales the amount of steering reduction as speed increases.
## Higher values reduce steering more at speed.
@export var speed_factor_coeff = 0.035
## Amount of Ackermann steering geometry. 0.0 turns both front wheels by the
## same angle, 1.0 is full Ackermann (the inner wheel turns more than the outer).
@export var ackermann_factor = 1.0

## Placeholder marking the "Steering" section. Does nothing.
func steering():
	pass

@export_group("Brake")

## Maximum braking torque the brakes can apply, in N·m.
@export var max_brake_torque = 900.0

## Placeholder marking the "Brake" section. Does nothing.
func brake():
	pass

@export_group("Assists")

## Placeholder marking the "Assists" section. Does nothing.
func assists():
	pass

@export_subgroup("ABS")

enum ABSType {
	## ABS Modes, changing how ABS behaves per wheel.
	PerWheel,
	SingleUnit
}

var current_abs_mode = ABSType.SingleUnit

## Enables the anti-lock braking system when true.
@export var ABS = false
## Rate at which ABS acts (how quickly it applies brake pressure). On a 1.0-0.0 scale of input.
@export var abs_rate_apply = 0.0
## Rate at which ABS acts (how quickly it releases brake pressure). On a 1.0-0.0 scale of input.
@export var abs_rate_release = 0.0
## Slip ratio  which ABS starts reducing brake force. Braking slip is
## negative, so this is a negative number.
@export var abs_slip_apply = -0.25
## Slip ratio  which ABS starts increasing brake force. Braking slip is
## negative, so this is a negative number.
@export var abs_slip_release = -0.10
## Target brake input after ABS is enabled. On a 1.0-0.0 scale of input.
@export var abs_pedal_target = 0.2

@export_subgroup("TC")

## Enables traction control when true.
@export var TC = false 
## Slip ratio above which traction control starts cutting drive torque.
@export var tc_slip_threshold = 0.12
## Rate at which traction control reduces torque once the threshold is crossed.
## Negative by convention.
@export var tc_decay_rate = -20.0

@export_subgroup("Stability")

## Stability control switch. Not implemented yet, and not exported, so it does
## not appear in the Inspector.
var Stability = false # a thing to work on

@export_group("Motor")

## Placeholder marking the "Motor" section. Does nothing.
func motor():
	pass

## Peak engine torque, in N·m. The [member torque_curve] is scaled by this
## value.
@export var max_torque = 136.0
## Rev limiter. The engine is not driven above this speed, in RPM. Also used to
## normalize RPM when sampling [member torque_curve].
@export var max_rpm = 7200.0
## Engine idle speed, in RPM.
@export var idle_rpm = 800.0
## RPM below which the engine stalls. Engine stalling is not implemented yet.
@export var stall_rpm = 0.0
## Rotational inertia of the engine (flywheel and rotating parts), in kg·m².
@export var engine_inertia = 0.1
## Constant term of the engine friction polynomial, in N·m.
@export var friction_c0 = 3.0
## Linear term of the engine friction polynomial. Multiplied by normalized RPM
## (RPM divided by [member max_rpm]).
@export var friction_c1 = 4.5
## Quadratic term of the engine friction polynomial. Multiplied by the square of
## normalized RPM.
@export var friction_c2 = 8.0
## Whether the front left wheel receives engine torque.
@export var FL_torque_engine = false
## Whether the front right wheel receives engine torque.
@export var FR_torque_engine = false
## Whether the rear left wheel receives engine torque.
@export var RL_torque_engine = true
## Whether the rear right wheel receives engine torque.
@export var RR_torque_engine = true
## Whether the front right wheel has brakes.
@export var FR_torque_brake = false
## Whether the front left wheel has brakes.
@export var FL_torque_brake = false
## Whether the rear right wheel has brakes.
@export var RR_torque_brake = true
## Whether the rear left wheel has brakes.
@export var RL_torque_brake = true
## Engine torque curve. The x axis is normalized RPM (0 to 1, where 1 equals
## [member max_rpm]) and the y axis is the fraction of [member max_torque]
## (0 to 1).
@export var torque_curve: Curve

@export_subgroup("Limited Slip Differential")

## Type of differential used on each driven axle.
enum DiffType {
	## Always splits the axle torque evenly between the two wheels.
	OPEN,
	## Torsen-style limited slip. Biases torque toward the wheel with more grip
	## using [member TBR].
	TORSEN_LSD,
	## Clutch-pack limited slip. Locks the wheels together using
	## [member minimum_clutch_lsd_force] and [member clutch_lsd_ramp_factor].
	CLUTCH_LSD,
	## Electronically controlled limited slip. Not implemented yet.
	ELECTRONIC_LSD
}

## The differential type used on the driven axle(s).
@export var differential: DiffType = DiffType.OPEN

## Preload of the clutch-type LSD, in N·m. This lock torque is always present,
## even with little axle torque. Only used by [constant DiffType.CLUTCH_LSD].
@export var minimum_clutch_lsd_force = 100.0
## How much extra lock torque the clutch-type LSD gains per unit of axle torque.
## Higher values lock harder under power. Only used by
## [constant DiffType.CLUTCH_LSD].
@export var clutch_lsd_ramp_factor = 1.0
## Share of the drive torque sent to the front axle when both axles are driven
## (all-wheel drive). The rear axle gets the remainder, so 0.5 is an even split.
@export var center_diff_split = 0.5
## Torque bias ratio of the Torsen LSD. A value of 2.0 means the wheel with more
## grip can receive up to twice the torque of the other wheel. Only used by
## [constant DiffType.TORSEN_LSD].
@export var TBR = 2.0
## Minimum difference in slip ratio between the two wheels of an axle before the
## differential starts biasing torque. Smaller differences are split evenly.
@export var SLIP_THRESHOLD = 0.01

@export_group("Transmission")

## Placeholder marking the "Transmission" section. Does nothing.
func transmisson():
	pass

## Gear ratios indexed by gear number: index 0 is reverse (negative), index 1 is
## neutral (0.0), and indices 2 and up are the forward gears (first gear is at
## index 2).
@export var gear_ratio = [-3.758, 0.0, 3.136, 1.888, 1.330, 1.000, 0.814]
## Fraction of torque that survives the drivetrain (1.0 is lossless, 0.9 loses
## 10 percent).
@export var drive_train_efficeny = 1.0
## Final drive ratio, applied after the gearbox.
@export var final_drive = 4.3
## Maximum torque the clutch can transmit when fully engaged, in N·m.
@export var max_clutch_torque = 262.5

@export_group("Wheel")

## Static camber angle of each wheel, in degrees. Negative means the top of the
## wheel leans inward.
@export var camber_angles = [-1.2, -1.2, -1.7, -1.7]
## Camber change per unit of suspension travel for each wheel. Typically
## negative so that the wheel gains negative camber as it compresses.
@export var camber_gain = [-18.0, -18.0, -25.0, -25.0]
## Radius of the wheel including the tire, in meters.
@export var wheel_radius = 0.2885
## Mass of one wheel, in kg.
@export var wheel_mass = 13.4
## Rolling resistance coefficient (dimensionless).
@export var rolling_resistance_coeff = 0.0105
## Rotational inertia of one wheel, in kg·m².
@export var wheel_inertia: float = 0.8

## Placeholder marking the "Wheel" section. Does nothing.
func wheel():
	pass

## Available tire force models.
enum TireModelType {
	## A brush tire model. Configured under "Brush Model" below.
	Brush_Model
}

## The tire model used to compute tire forces.
@export var TireModel: TireModelType = TireModelType.Brush_Model
# Brush Model

@export_subgroup("Brush Model")

## Placeholder marking the "Brush Model" section. Does nothing.
func brush_model():
	pass

## Longitudinal slip stiffness of the tire, in N per unit slip ratio. Controls
## how quickly the tire builds force under braking and acceleration.
@export var brush_Csx: float = 65000.0  
## Lateral slip stiffness of the tire, in N per radian of slip angle. Controls
## how quickly the tire builds cornering force.
@export var brush_Csy: float = 50000.0  
## Friction coefficient between tire and road. Limits the maximum tire force.
@export var brush_mu: float = 1.25

	
@export_group("Debug")

## Enables the debug visuals when true.
@export var DEBUG = true
## Color of the suspension force arrows.
@export var Suspension_Arrow_Color: Color = Color.GREEN
## Color of the longitudinal (forward and back) tire force arrows. (The
## spelling of the name is kept as in the source.)
@export var Longtiude_Arrow_Color: Color = Color.RED
## Color of the lateral (sideways) tire force arrows.
@export var Lateral_Arrow_Color: Color = Color.BLUE
## Offset applied to the debug arrows relative to their wheel, so they do not
## overlap the car.
@export var Position_Offset: Vector3 = Vector3(2, 0, 0)
## Shows the numeric force magnitude next to the debug arrows when true. (The
## spelling of the name is kept as in the source.)
@export var Display_Force_Magntiude = false
