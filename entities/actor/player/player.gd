@tool
extends Node3D
class_name Player



@export var func_godot_properties: Dictionary = {}
@export var death_barrier_y_pos : float = -10.0
@export var player_body : PlayerBody
@export var speed_label : Label
@export var flying_enabled : bool = false

@export_group("Debug")
@export var display_speed_enabled : bool = false
@export var draw_velocity_enabled : bool = false

@onready var capsule_shape : CapsuleShape3D = preload("res://entities/actor/player/player_capsule_shape.tres")
@onready var cylinder_shape : CylinderShape3D = preload("res://entities/actor/player/player_cylinder_shape.tres")
@onready var box_shape : BoxShape3D = preload("res://entities/actor/player/player_box_shape.tres")

var active_checkpoint: Checkpoint
var spawn_rotation_degrees: Vector3
var restart_debounce_timer: SceneTreeTimer
var dead: bool = false


func _ready() -> void:
	if Engine.is_editor_hint(): return
	
	SETTINGS.settings_changed.connect(_on_settings_changed)
	GAME.level_completed.connect(_on_level_completed)
	
	flying_enabled = SETTINGS.player_flying_enabled
	speed_label.visible = SETTINGS.player_display_speed_enabled
	display_speed_enabled = SETTINGS.player_display_speed_enabled
	draw_velocity_enabled = SETTINGS.player_draw_velocity_enabled
	
	spawn_rotation_degrees = player_body.head.rotation_degrees



func _on_settings_changed(setting: GameSettings.Setting) -> void:
	match setting :
		GameSettings.Setting.PLAYER_FLYING_ENABLED:
			flying_enabled = SETTINGS.player_flying_enabled
			if not flying_enabled :
				player_body.motion_mode = PlayerBody.MotionMode.GROUNDED
		GameSettings.Setting.PLAYER_DISPLAY_SPEED_ENABLED:
			display_speed_enabled = SETTINGS.player_display_speed_enabled
			speed_label.visible = display_speed_enabled
		GameSettings.Setting.PLAYER_DRAW_VELOCITY_ENABLED:
			draw_velocity_enabled = SETTINGS.player_draw_velocity_enabled



func _on_level_completed() -> void:
	player_body.movement_disabled = true




func _func_godot_apply_properties(props: Dictionary) -> void:
	get_parent().set_editable_instance(self, true);
	
	var angles: Vector3 = GameManager.calculate_rotation_degrees_from_properties(self, props)
	
	player_body.head.apply_rotation_degrees(angles)




func _process(_delta: float) -> void:
	if display_speed_enabled:
		speed_label.text = "Player speed: %.2f m/s" % player_body.velocity.length()
	
	if draw_velocity_enabled:
		# Debug draw velocity
		DebugDraw3D.draw_arrow(
			player_body.global_position,
			player_body.global_position + (player_body.velocity * 0.2),
			Color.GREEN,
			0.2,
		)




func _physics_process(_delta: float) -> void:
	if Engine.is_editor_hint(): return
	
	if player_body.global_position.y < death_barrier_y_pos:
		die()
	
	for collider_data in player_body.collider_datas:
		var collider_object: Object = instance_from_id(collider_data.collider_id)
		var geo_material: GeoMaterial = collider_object.get("geo_material")
		if geo_material and geo_material.hazardous:
			die()
	
	if dead:
		respawn()
		return
	
	if player_body.current_floor_collider_encoded:
		var collider_object: Object = instance_from_id(player_body.current_floor_collider_encoded.object_id)
		if collider_object is FuncMover:
			var mover := collider_object as FuncMover
			
			if not mover.is_active:
				mover.is_active = true
	
	if flying_enabled:
		if Input.is_action_just_pressed("switch_move_mode"):
			if player_body.motion_mode == PlayerBody.MotionMode.GROUNDED :
				player_body.motion_mode = PlayerBody.MotionMode.FLOATING
			else :
				player_body.motion_mode = PlayerBody.MotionMode.GROUNDED




func _unhandled_key_input(event: InputEvent) -> void:
	if event.is_action_pressed("restart") and not restart_debounce_timer :
		restart_debounce_timer = get_tree().create_timer(1.0)
		restart_debounce_timer.timeout.connect(func(): restart_debounce_timer = null)
		die()
	
	if SETTINGS.player_collision_shape_swap_enabled :
		if event.is_action_pressed("shape1") :
			player_body.collider.shape = capsule_shape
		if event.is_action_pressed("shape2") :
			player_body.collider.shape = cylinder_shape
		if event.is_action_pressed("shape3") :
			player_body.collider.shape = box_shape




func respawn() -> void:
	if active_checkpoint:
		player_body.global_position = active_checkpoint.global_position
		player_body.head.apply_rotation_degrees(active_checkpoint.rotation_degrees)
	else:
		player_body.global_position = global_position
		player_body.head.apply_rotation_degrees(spawn_rotation_degrees)
		
	player_body.velocity = Vector3.ZERO
	
	GAME.level_reset.emit()
	
	dead = false





func die() -> void:
	dead = true
