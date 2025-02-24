extends Page

@export var fullscreen_button: OptionButton
@export var mouse_sensitivity_slider: HSlider
@export var camera_fov_slider: HSlider
@export var player_flying_enabled_checkbutton: CheckButton
@export var player_display_speed_enabled_checkbutton: CheckButton
@export var player_draw_velocity_enabled_checkbutton: CheckButton
@export var player_collision_shape_swap_enabled_checkbutton: CheckButton


func _ready() -> void:
	super._ready()
	
	SETTINGS.settings_changed.connect(_on_settings_changed)
	
	# Window mode
	match DisplayServer.window_get_mode():
		DisplayServer.WINDOW_MODE_WINDOWED:
			fullscreen_button.select(0)
		DisplayServer.WINDOW_MODE_EXCLUSIVE_FULLSCREEN:
			fullscreen_button.select(1)
	fullscreen_button.item_selected.connect(set_window_mode)
	
	# Mouse sensitivity
	mouse_sensitivity_slider.value = SETTINGS.mouse_sensivity
	mouse_sensitivity_slider.drag_ended.connect(_on_mouse_sensitivity_slider_drag_ended)
	
	# Camera FOV
	camera_fov_slider.value = SETTINGS.camera_fov
	camera_fov_slider.value_changed.connect(_on_camera_fov_slider_value_changed)
	
	# Player flying enabled
	player_flying_enabled_checkbutton.button_pressed = SETTINGS.player_flying_enabled
	player_flying_enabled_checkbutton.toggled.connect(
		func (toggled_on : bool) -> void : SETTINGS.player_flying_enabled = toggled_on
	)
	
	# Player display speed enabled
	player_display_speed_enabled_checkbutton.button_pressed = SETTINGS.player_display_speed_enabled
	player_display_speed_enabled_checkbutton.toggled.connect(
		func (toggled_on : bool) -> void : SETTINGS.player_display_speed_enabled = toggled_on
	)
	
	# Player draw velocity enabled
	player_draw_velocity_enabled_checkbutton.button_pressed = SETTINGS.player_draw_velocity_enabled
	player_draw_velocity_enabled_checkbutton.toggled.connect(
		func (toggled_on : bool) -> void : SETTINGS.player_draw_velocity_enabled = toggled_on
	)
	
	# Player swap collision shape enabled
	player_collision_shape_swap_enabled_checkbutton.button_pressed = SETTINGS.player_collision_shape_swap_enabled
	player_collision_shape_swap_enabled_checkbutton.toggled.connect(
		func (toggled_on : bool) -> void : SETTINGS.player_collision_shape_swap_enabled = toggled_on
	)


func _on_settings_changed(_setting: GameSettings.Setting) -> void:
	player_flying_enabled_checkbutton.button_pressed = SETTINGS.player_flying_enabled
	player_display_speed_enabled_checkbutton.button_pressed = SETTINGS.player_display_speed_enabled
	player_draw_velocity_enabled_checkbutton.button_pressed = SETTINGS.player_draw_velocity_enabled
	player_collision_shape_swap_enabled_checkbutton.button_pressed = SETTINGS.player_collision_shape_swap_enabled
	


func set_window_mode(item_index: int) -> void:
	if item_index == 0:
		DisplayServer.window_set_mode(DisplayServer.WINDOW_MODE_WINDOWED)
	if item_index == 1:
		DisplayServer.window_set_mode(DisplayServer.WINDOW_MODE_EXCLUSIVE_FULLSCREEN)


func _on_mouse_sensitivity_slider_drag_ended(value_changed: bool) -> void:
	if value_changed:
		SETTINGS.mouse_sensivity = mouse_sensitivity_slider.value


func _on_camera_fov_slider_value_changed(value: float) -> void:
	SETTINGS.camera_fov = value
