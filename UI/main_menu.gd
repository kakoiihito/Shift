extends Control

var save_manager

func _on_button_pressed() -> void:
	var scene_path = "res://node_3d.tscn"
	var scene_resource = load(scene_path)
	if scene_resource:
		var new_scene = scene_resource.instantiate()
		get_tree().current_scene.queue_free()  # Free the current scene
		get_tree().root.add_child(new_scene)  # Add the new scene to the root
		get_tree().current_scene = new_scene   # Set the new scene as current_scene
	else:
		push_error("Failed to load scene: " + scene_path)

func _on_button_2_pressed() -> void:
	var scene_path = "res://UI/editor.tscn"
	var scene_resource = load(scene_path)
	if scene_resource:
		var new_scene = scene_resource.instantiate()
		get_tree().current_scene.queue_free()  # Free the current scene
		get_tree().root.add_child(new_scene)  # Add the new scene to the root
		get_tree().current_scene = new_scene   # Set the new scene as current_scene
	else:
		push_error("Failed to load scene: " + scene_path)
