@tool
extends WorldEnvironment
class_name Worldspawn

const LIGHT_LAYER_MASK: int = 31

@export var func_godot_properties: Dictionary = {}

func _func_godot_apply_properties(props: Dictionary) -> void:
	# WORLD ENVIRONMENT
	var env: Environment = Environment.new()
	# Base settings
	env.set_fog_enabled(false);
	env.set_tonemapper(Environment.TONE_MAPPER_FILMIC)
	env.set_glow_enabled(false)
	# Background
	env.set_background(Environment.BG_COLOR);
	if props.has("bg_color"):
		env.set_bg_color(props["bg_color"]);
	else:
		env.set_bg_color(Color());
	# Ambient light
	env.set_ambient_source(Environment.AMBIENT_SOURCE_COLOR);
	if props.has("ambient_color"):
		env.set_ambient_light_color(props["ambient_color"]);
	else:
		env.set_ambient_light_color(Color.hex(0xFFFFFFFF));
	if props.has("ambient_light"):
		env.set_ambient_light_energy(props["ambient_light"]);
	else:
		env.set_ambient_light_energy(0.0);
	env.set_ambient_light_sky_contribution(0.0)
	# Brightness setup
	env.set_adjustment_enabled(true)
	env.set_adjustment_brightness(1.0)
	set_environment(env)
	# Sky setup
	if props.has("sky"):
		var sky_name : String = props["sky"]
		var sky_resource_path : String = "res://skyboxes/" + sky_name + "/" + sky_name + ".tres"
		if ResourceLoader.exists(sky_resource_path, "Sky") :
			env.sky = load(sky_resource_path)
			env.background_mode = Environment.BG_SKY
	if props.has("sky_rotation") :
		var rotation_degree : Vector3 = props["sky_rotation"]
		var rotation_radian : Vector3
		for i in range(3) :
			rotation_radian[i] = deg_to_rad(rotation_degree[i])
		env.sky_rotation = rotation_radian


func _func_godot_build_complete() -> void:
	# Find existing lightmap, else build a new one
	var lit: LightmapGI
	if get_owner().has_node("lightmap"):
		lit = get_owner().get_node("lightmap")
	else:
		lit = LightmapGI.new()
		lit.set_name("lightmap")
		get_owner().add_child(lit)
		lit.set_owner(get_owner())
	lit.get_parent().call_deferred("move_child", lit, 0);
	lit.set_layer_mask(LIGHT_LAYER_MASK)
	# Bake Quality
	if func_godot_properties.has("lit_quality"):
		lit.set_bake_quality(func_godot_properties["lit_quality"] as LightmapGI.BakeQuality)
	else:
		lit.set_bake_quality(LightmapGI.BakeQuality.BAKE_QUALITY_MEDIUM);
	# Bounces
	if func_godot_properties.has("lit_bounces"):
		lit.set_bounces(func_godot_properties["lit_bounces"] as int)
	else:
		lit.set_bounces(3);
	# Lightmapper Probes Subdivision
	if func_godot_properties.has("lit_probes_subdiv"):
		lit.set_generate_probes(func_godot_properties["lit_probes_subdiv"] as LightmapGI.GenerateProbes)
	else:
		lit.set_generate_probes(LightmapGI.GenerateProbes.GENERATE_PROBES_SUBDIV_8);
	# Use Denoiser
	if func_godot_properties.has("lit_denoiser"):
		lit.set_use_denoiser(func_godot_properties["lit_denoiser"] as bool);
	else:
		lit.set_use_denoiser(true);
