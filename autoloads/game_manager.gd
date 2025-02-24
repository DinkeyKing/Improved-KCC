
extends Node
class_name GameManager


signal exited_to_menu
signal game_world_entered

signal level_completed
signal level_reset



static var game_world : Node 


func _ready() -> void:
	if Engine.is_editor_hint(): return 
	
	Input.mouse_mode = Input.MOUSE_MODE_VISIBLE


func exit_to_menu() -> void:
	# Unload game world
	game_world.queue_free()
	
	# Load main menu
	var main_menu_scene: PackedScene = preload("res://gui/main_menu/main_menu.tscn")
	var main_menu_instance: Node = main_menu_scene.instantiate()
	get_tree().root.add_child(main_menu_instance)
	
	# Unpause
	get_tree().paused = false
	
	# Emit signal
	exited_to_menu.emit()




func load_level(path: String) -> void:
	# Load map
	var map_scene : PackedScene = load(path) as PackedScene
	var map_instance : Node = map_scene.instantiate()
	get_tree().root.add_child(map_instance)
	
	GAME.game_world = map_instance
	Input.mouse_mode = Input.MOUSE_MODE_CAPTURED
	
	# Load pause screen
	var pause_menu_scene: PackedScene = preload("res://gui/pause_menu/pause_menu.tscn") as PackedScene
	var pause_menu_instance: Node = pause_menu_scene.instantiate()
	get_tree().root.add_child(pause_menu_instance)
	
	# Load win screen
	var win_menu_scene: PackedScene = preload("res://gui/win_menu/win_menu.tscn") as PackedScene
	var win_menu_instance: Node = win_menu_scene.instantiate()
	get_tree().root.add_child(win_menu_instance)
	
	game_world_entered.emit()



# Set the targetnames for the entity. We can specify multiple targetnames using comma delimiting.
static func set_targetname(ent: Node, targetname: String)->void:
	if ent != null and targetname != "":
		for t in targetname.split(","):
			if t != "":
				ent.add_to_group(t)


static func apply_rotation(node: Node3D, properties: Dictionary, is_light: bool = false, is_info_intermission: bool = false) -> void:
	node.rotation_degrees = calculate_rotation_degrees_from_properties(node, properties, is_light, is_info_intermission)


static func calculate_rotation_degrees_from_properties(node: Node3D, properties: Dictionary, is_light: bool = false, is_info_intermission: bool = false) -> Vector3:
	var angles := Vector3.ZERO
	if 'angles' in properties or 'mangle' in properties:
		var key := 'angles' if 'angles' in properties else 'mangle'
		var angles_raw = properties[key]
		if not angles_raw is Vector3:
			angles_raw = angles_raw.split_floats(' ')
		if angles_raw.size() > 2:
			angles = Vector3(-angles_raw[0], angles_raw[1], -angles_raw[2])
			if key == 'mangle':
				if is_light:
					angles = Vector3(angles_raw[1], angles_raw[0], -angles_raw[2])
				elif is_info_intermission:
					angles = Vector3(angles_raw[0], angles_raw[1], -angles_raw[2])
		else:
			push_error("Invalid vector format for \'" + key + "\' in entity \'" + node.get_class() + "\'")
	elif 'angle' in properties:
		var angle = properties['angle']
		if not angle is float:
			angle = float(angle)
		angles.y += angle
	angles.y += 180
	
	return angles


static func get_edited_map_node(n: Node) -> FuncGodotMap:
	var node := n.get_tree().edited_scene_root.get_child(1)
	if node is FuncGodotMap:
		return node as FuncGodotMap
	printerr("\"get_edited_map_node\" - Found node is not FuncGodotMap.")
	return null
