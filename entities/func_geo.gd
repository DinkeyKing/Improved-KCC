@tool
extends StaticBody3D
class_name FuncGeo

const INVERSE_SCALE: float = 1.0 / 32.0

@export var properties: Dictionary = {} :
	set(value):
		properties = value;
		
		if !Engine.is_editor_hint():
			return
		
		for child in get_children():
			if child.get_class() == "MeshInstance3D":
				var m: MeshInstance3D = child
				m.set_gi_mode(GeometryInstance3D.GI_MODE_STATIC)
				#m.set_cast_shadows_setting((properties["cast_shadow"] as GeometryInstance3D.ShadowCastingSetting))
				#if get_parent() is QodotMap and m.mesh.get_class() == "ArrayMesh":
					#(m.mesh as ArrayMesh).lightmap_unwrap(Transform3D(), (get_parent() as QodotMap).inverse_scale_factor)
	get:
		return properties

func _ready() -> void:
	if !Engine.is_editor_hint():
			return
