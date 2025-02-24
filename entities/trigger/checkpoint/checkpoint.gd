extends Area3D
class_name Checkpoint


@export var mesh_instance : MeshInstance3D
@export var active_material : StandardMaterial3D

func _on_body_entered(body: Node3D) -> void:
	if body is PlayerBody:
		var player_body : PlayerBody = body as PlayerBody
		var player := player_body.get_parent() as Player
		player.active_checkpoint = self
		
		mesh_instance.set_surface_override_material(0, active_material)
	
	disconnect("body_entered", _on_body_entered)
	set_deferred("monitoring", false)


func _physics_process(delta: float) -> void:
	mesh_instance.rotate_y(1.5 * delta)
	mesh_instance.rotate_z(1.5 * delta)
