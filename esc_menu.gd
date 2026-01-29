extends Control

func _ready() -> void:
	hide()

func _on_button_pressed() -> void:
	get_tree().change_scene_to_file("res://UI/main_menu.tscn")

func _input(event: InputEvent) -> void:
	if event.is_action_pressed("Esc"): # default is Esc
		show()
		Input.set_mouse_mode(Input.MOUSE_MODE_VISIBLE)


func _on_button_2_pressed() -> void:
	hide()
