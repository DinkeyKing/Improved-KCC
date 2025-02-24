@tool
extends RigidBody3D

@export var debug_impulse_enabled : bool = false

var spawn_transform : Transform3D

func _func_godot_apply_properties(props: Dictionary) -> void:
	if props.has("debug_impulse_enabled"):
		debug_impulse_enabled = props["debug_impulse_enabled"]
	

func _ready() -> void:
	if Engine.is_editor_hint() :
		return
	
	spawn_transform = global_transform
	
	GAME.level_reset.connect(
		func () -> void :
			var body_state : PhysicsDirectBodyState3D = PhysicsServer3D.body_get_direct_state(self)
			body_state.transform = spawn_transform
			body_state.linear_velocity = Vector3.ZERO
			body_state.angular_velocity = Vector3.ZERO
	)

func _physics_process(_delta: float) -> void :
	if Engine.is_editor_hint() :
		return
	
	if debug_impulse_enabled :
		if Input.is_action_just_pressed("box_impulse_up") :
			apply_central_impulse(Vector3.UP * 15.0)
