extends Node3D

@export var car: RigidBody3D
var force_path: Vector3
var force: Vector3


func _ready() -> void:
	pass


func _process(_delta: float) -> void:
	
	force = force_path
	
	if car.DEBUG == false:
		visible = false
	update_force_arrow()

func update_force_arrow():
	var magnitude = force.length()
	var direction = force.normalized()
	
	var length = magnitude * 0.001
	
	var cyl_mesh: CylinderMesh = $Center/Arrow.mesh
	cyl_mesh.height = length
	
	$Center/Arrow.position = direction * (length * 0.5)
	$Center/Arrow.basis = Basis(Quaternion(Vector3.UP, direction))
