
extends Node
class_name  GameSettings


enum Setting {
	MOUSE_SENSITIVITY,
	CAMERA_FOV,
	PLAYER_FLYING_ENABLED,
	PLAYER_DISPLAY_SPEED_ENABLED,
	PLAYER_DRAW_VELOCITY_ENABLED,
	PLAYER_COLLISION_SHAPE_SWAP_ENABLED
}


const DEFAULT_MOUSE_SENSITIVITY: float = 1.0
const DEFAULT_CAMERAA_FOV: float = 90.0



signal settings_changed(setting: Setting)



var mouse_sensivity : float = DEFAULT_MOUSE_SENSITIVITY :
	set(value):
		mouse_sensivity = value
		settings_changed.emit(Setting.MOUSE_SENSITIVITY)

var camera_fov : float = DEFAULT_CAMERAA_FOV :
	set(value):
		camera_fov = value
		settings_changed.emit(Setting.CAMERA_FOV)

var player_flying_enabled : bool = false :
	set(value):
		player_flying_enabled = value
		settings_changed.emit(Setting.PLAYER_FLYING_ENABLED)

var player_display_speed_enabled : bool = false :
	set(value):
		player_display_speed_enabled = value
		settings_changed.emit(Setting.PLAYER_DISPLAY_SPEED_ENABLED)

var player_draw_velocity_enabled : bool = false :
	set(value):
		player_draw_velocity_enabled = value
		settings_changed.emit(Setting.PLAYER_DRAW_VELOCITY_ENABLED)

var player_collision_shape_swap_enabled : bool = false :
	set(value):
		player_collision_shape_swap_enabled = value
		settings_changed.emit(Setting.PLAYER_COLLISION_SHAPE_SWAP_ENABLED)
