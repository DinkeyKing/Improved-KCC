extends Menu



func _ready() -> void:
	super._ready()
	
	visible = false
	
	GAME.exited_to_menu.connect(_on_exited_to_menu)
	GAME.level_completed.connect(_on_level_complete)


func _on_exited_to_menu() -> void:
	queue_free()


func _on_level_complete() -> void:
	queue_free()



func _process(_delta : float) -> void:
	if Input.is_action_just_pressed("pause"):
		if get_tree().paused == false:
			Input.mouse_mode = Input.MOUSE_MODE_VISIBLE
			visible = true
			selected_page_name = home_page_name
			get_tree().paused = true
		else :
			Input.mouse_mode = Input.MOUSE_MODE_CAPTURED
			visible = false
			selected_page_name = home_page_name
			get_tree().paused = false
