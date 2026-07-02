extends Node3D
@export var car: RigidBody3D

var force_path: Vector3
var force: Vector3

var cyl_mesh: CylinderMesh

func _ready() -> void:
	# Create a brand new mesh resource unique to this instance
	cyl_mesh = CylinderMesh.new()
	cyl_mesh.top_radius = 0.05      # adjust to taste
	cyl_mesh.bottom_radius = 0.05
	cyl_mesh.height = 1.0

	$Center/Arrow.mesh = cyl_mesh

func _process(_delta: float) -> void:
	if car.DEBUG == true:
		visible = true
		force = force_path
		update_force_arrow()
	else:
		visible = false

func update_force_arrow():
	var magnitude = force.length()

	if magnitude < 0.0001:
		$Center/Arrow.visible = false
		return

	$Center/Arrow.visible = true
	var direction = force / magnitude
	var length = magnitude * 0.001

	cyl_mesh.height = length

	$Center/Arrow.basis = Basis(Quaternion(Vector3.UP, direction))
	$Center/Arrow.position = direction * (length / 2.0)
