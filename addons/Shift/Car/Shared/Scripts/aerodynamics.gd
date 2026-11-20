extends Node


# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	pass # Replace with function body.


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	pass

func aerodynamics(Values: Resource, car: RigidBody3D, wheels: Array[RayCast3D]):
	
	drag(Values, car)
	lift_and_downforce(Values, car, wheels)

func drag(Values: Resource, car: RigidBody3D):
	
	var force = -(0.5 * Values.air_density * car.linear_velocity * car.linear_velocity.length() * Values.drag_coefficient * Values.drag_frontal_area)
	car.apply_central_force(force)

func lift_and_downforce(Values: Resource, car: RigidBody3D, wheels: Array[RayCast3D]):
	
	var force = 0.5 * Values.air_density * car.linear_velocity.length_squared() * Values.downforce_lift_coefficient * Values.downforce_lift_area

	var front_z = INF
	var rear_z = -INF
	for wheel in wheels:
		var z = car.to_local(wheel.global_position).z
		front_z = min(front_z, z)
		rear_z = max(rear_z, z)

	var cop_z = lerp(rear_z, front_z, Values.aero_balance_front)
	var local_point = Vector3(0.0, 0.0, cop_z)

	var offset = car.global_transform.basis * (local_point - car.center_of_mass)
	car.apply_force(car.global_transform.basis.y * force, offset)
