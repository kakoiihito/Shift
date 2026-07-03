extends Node3D
@export var car: RigidBody3D
@export var Display_Force_Magntiude: bool

var force_path: Vector3
var force: Vector3



var cyl_mesh: CylinderMesh

func _ready() -> void:
	# Create a brand new mesh resource unique to this instance
	cyl_mesh = CylinderMesh.new()
	cyl_mesh.top_radius = 0.01
	cyl_mesh.bottom_radius = 0.05
	cyl_mesh.height = 1.0

	$Center/Arrow.mesh = cyl_mesh

func _process(_delta: float) -> void:
	if car.DEBUG == true:
		visible = true
		force = force_path
		update_force_arrow()
		update_force_label()
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
	
func update_force_label():
	if Display_Force_Magntiude == true:
		var magnitude = force.length()
		$Center/Force_Magnitude.text = "%.1f" % magnitude
