
extends Area3D
class_name Goalpoint



@export var mesh_instance : MeshInstance3D
@export var active_material : StandardMaterial3D

func _on_body_entered(body: Node3D) -> void:
	if body is PlayerBody:
		GAME.level_completed.emit()
		
		mesh_instance.set_surface_override_material(0, active_material)
	
	disconnect("body_entered", _on_body_entered)
	set_deferred("monitoring", false)


func _physics_process(delta: float) -> void:
	if Engine.is_editor_hint():
		return
	rotate_y(1.5 * delta)
