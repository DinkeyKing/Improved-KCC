extends Menu


# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	super._ready()
	
	visible = false
	
	GAME.level_completed.connect(_on_level_completed)
	GAME.exited_to_menu.connect(_on_exited_to_menu)




func _on_level_completed() -> void:
	visible = true
	Input.mouse_mode = Input.MOUSE_MODE_VISIBLE



func _on_exited_to_menu() -> void:
	queue_free()
