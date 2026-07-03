extends Camera3D

# Adjust this to change how fast the camera spins
const ROTATION_SPEED = 2.0 

func _process(delta: float) -> void:
	var input_dir = Input.get_vector("ui_left", "ui_right", "ui_up", "ui_down")
	

	if input_dir.x != 0.0:
		rotate_y(-input_dir.x * ROTATION_SPEED * delta)
	if input_dir.y != 0.0:
		rotate_x(-input_dir.y * ROTATION_SPEED * delta)
