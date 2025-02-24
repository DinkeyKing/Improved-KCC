
@tool
extends AnimatableBody3D
class_name FuncMover


enum MoveDirection {
	FORWARD,
	BACKWARD
}

enum MoveState {
	MOVE_BACK_AND_FORTH,
	MOVE_TO_END,
	MOVE_TO_GOAL,
	STOP
}


@export var func_godot_properties: Dictionary = {}
@export var geo_material: GeoMaterial

@export var path_follow: PathFollow3D
@export var target_path: String
@export var move_direction := MoveDirection.FORWARD
@export var move_state := MoveState.MOVE_BACK_AND_FORTH :
	set(value):
		move_state = value
		if progress_tween:
			progress_tween.kill()
@export var loop: bool = false
@export var move_speed: float = 3.0
@export var move_duration: float = 3.0
@export var use_move_duration: bool = false
@export var path: Path3D
@export var remote_transform: RemoteTransform3D
@export var path_length: float
@export var goal_progress: float = 0.0 :
	set(value):
		goal_progress = value
		if progress_tween:
			progress_tween.kill()
@export var active_move_state := MoveState.MOVE_BACK_AND_FORTH
@export var unactive_move_state := MoveState.STOP
@export var is_active: bool = true :
	set(value):
		is_active = value
		move_state = active_move_state if is_active else unactive_move_state
@export var angular_velocity: Vector3


var progress_tween: Tween
var init_transform: Transform3D




func _func_godot_apply_properties(props: Dictionary) -> void:
	if path_follow:
		path_follow.queue_free()
	path_follow = PathFollow3D.new()
	path_follow.rotation_mode = PathFollow3D.ROTATION_NONE
	
	if props.has("target"):
		target_path = props["target"]
	
	if props.has("move_direction"):
		move_direction = props["move_direction"]
	
	if props.has("active_move_state"):
		active_move_state = props["active_move_state"]
	
	if props.has("unactive_move_state"):
		unactive_move_state = props["unactive_move_state"]
	
	if props.has("is_active"):
		is_active = props["is_active"]
	
	if props.has("loop"):
		path_follow.loop = props["loop"]
	else:
		path_follow.loop = false
	
	if props.has("move_speed"):
		move_speed = props["move_speed"]
	
	if props.has("move_duration"):
		move_duration = props["move_duration"]
	
	if props.has("use_move_duration"):
		use_move_duration = props["use_move_duration"]
	
	if props.has("starting_progress"):
		path_follow.progress = props["starting_progress"]
	
	if props.has("goal_progress"):
		goal_progress = props["goal_progress"]
	
	if props.has("angular_velocity"):
		angular_velocity = props["angular_velocity"]
	
	move_state = active_move_state if is_active else unactive_move_state
	
	sync_to_physics = true
	
	# Connect build complete signal
	var map := GameManager.get_edited_map_node(self)
	if map:
		map.build_complete.connect(_on_build_complete)
	
	# Set geo material
	if geo_material:
		geo_material.free()
	geo_material = GeoMaterial.new()
	
	if props.has("override_default_friction"):
		geo_material.override_default_friction = props["override_default_friction"]
	if props.has("slippery") :
		geo_material.slippery = props["slippery"]
	if props.has("friction_speed") :
		geo_material.friction_speed = props["friction_speed"]
	if props.has("hazardous"):
		geo_material.hazardous = props["hazardous"]



func _func_godot_build_complete() -> void:
	if remote_transform:
		remote_transform.queue_free()
	remote_transform = RemoteTransform3D.new()
	
	var target_node : Node = get_tree().get_first_node_in_group(target_path)
	if target_node is Path3D:
		
		if path:
			path.queue_free()
		path = target_node as Path3D
		
		# Add PathFollow node as children of path
		path.add_child(path_follow)
		path_follow.owner = get_tree().edited_scene_root
		
		# Add RemoteTransform3D node as children of Pathfollow
		path_follow.add_child(remote_transform)
	else:
		# Add PathFollow node as a sibling
		add_sibling(remote_transform)
		remote_transform.global_transform = global_transform
	
	remote_transform.owner = get_tree().edited_scene_root
		
	# Set remote path after adjusting tree
	remote_transform.remote_path = remote_transform.get_path_to(self)
	
	init_transform = global_transform



func _on_build_complete() -> void:
	if path:
		path_length = path.curve.get_baked_length()




func _ready() -> void:
	if Engine.is_editor_hint(): return
	
	GAME.level_reset.connect(reset)




func _physics_process(delta: float) -> void:
	if Engine.is_editor_hint():
		return
	
	if path_follow:
		if move_state == MoveState.MOVE_BACK_AND_FORTH or move_state == MoveState.MOVE_TO_END:
			if not progress_tween or not progress_tween.is_valid():
				if move_state == MoveState.MOVE_BACK_AND_FORTH:
					if is_equal_approx(path_follow.progress, 0.0):
						move_direction = MoveDirection.FORWARD
					elif is_equal_approx(path_follow.progress, path_length):
						move_direction = MoveDirection.BACKWARD
				var end_goal: float = path_length if move_direction == MoveDirection.FORWARD else 0.0
				if not is_equal_approx(path_follow.progress, end_goal):
					create_tween_to_progress(end_goal)
						
		elif move_state == MoveState.MOVE_TO_GOAL:
			if not progress_tween or not progress_tween.is_valid() and not is_equal_approx(path_follow.progress, goal_progress):
				create_tween_to_progress(goal_progress)
	
	if move_state != MoveState.STOP:
		var angular_motion: Vector3 = angular_velocity * delta
		remote_transform.rotate_x(angular_motion.x)
		remote_transform.rotate_y(angular_motion.y)
		remote_transform.rotate_z(angular_motion.z)




func create_tween_to_progress(p_progress: float) -> void:
	if progress_tween and progress_tween.is_valid():
		progress_tween.kill()
		progress_tween.free()
	
	progress_tween = self.create_tween()
	progress_tween.set_ease(Tween.EASE_IN_OUT)
	progress_tween.set_trans(Tween.TRANS_SINE)
	progress_tween.set_process_mode(Tween.TWEEN_PROCESS_PHYSICS)
	
	var duration: float = absf(p_progress - path_follow.progress) / move_speed if not use_move_duration else move_duration
	progress_tween.tween_property(path_follow, "progress", p_progress, duration)





func reset() -> void:
	if progress_tween:
		progress_tween.kill()
	
	if func_godot_properties.has("target"):
		target_path = func_godot_properties["target"]
	
	if func_godot_properties.has("move_direction"):
		move_direction = func_godot_properties["move_direction"]
	
	if func_godot_properties.has("is_active"):
		is_active = func_godot_properties["is_active"]
	
	if func_godot_properties.has("starting_progress") and path_follow:
		path_follow.progress = func_godot_properties["starting_progress"]
	
	move_state = active_move_state if is_active else unactive_move_state
	
	remote_transform.global_basis = init_transform.basis  # Reset basis
