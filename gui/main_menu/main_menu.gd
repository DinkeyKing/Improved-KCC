extends Menu


# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	super._ready()
	
	GAME.game_world_entered.connect(_on_game_world_entered)


func _input(event: InputEvent) -> void:
	if event.is_action_pressed("load_sandbox"):
		GAME.load_level("res://maps/sandbox/sandbox.tscn")
		
		# Enable all debug settings
		SETTINGS.player_flying_enabled = true
		SETTINGS.player_display_speed_enabled = true
		SETTINGS.player_draw_velocity_enabled = true
		SETTINGS.player_collision_shape_swap_enabled = true



func _on_game_world_entered() -> void:
	queue_free()
