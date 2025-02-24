
@tool
extends StaticBody3D
class_name FuncGeo

@export var func_godot_properties: Dictionary = {}
@export var geo_material: GeoMaterial

func _func_godot_apply_properties(props: Dictionary) -> void:
	if geo_material:
		geo_material.free()
	geo_material = GeoMaterial.new()
	
	if props.has("slippery") :
		geo_material.slippery = props["slippery"]
	if props.has("friction_speed") :
		geo_material.friction_speed = props["friction_speed"]
	if props.has("hazardous"):
		geo_material.hazardous = props["hazardous"]
	if props.has("override_default_friction") :
		geo_material.override_default_friction = props["override_default_friction"]
