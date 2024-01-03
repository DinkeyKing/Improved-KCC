extends Node3D
class_name PlayerCamera

##############
# PROPERTIES #
##############

@export_group("General")
@export var sensitivity : float = 0.005
@export var max_rotation_up : float = 90.0
@export var max_rotation_down : float = -90.0
@export_group("Height effect")
@export var stand_height : float = 0.5

#########
# NODES #
#########

@onready var player : Player = self.get_parent() as Player
@onready var pivot_x : Node3D = self.get_child(0) as Node3D
@onready var pivot_z : Node3D = self.get_child(0).get_child(0) as Node3D

#############
# FUNCTIONS #
#############

func _ready():
	position.y = stand_height

func _unhandled_input(event : InputEvent) -> void:
	if Input.mouse_mode != Input.MOUSE_MODE_CAPTURED :
		return
	if event is InputEventMouseMotion :
		rotate_y(-event.relative.x * sensitivity)
		pivot_x.rotate_x(-event.relative.y * sensitivity)
		pivot_x.rotation.x = clamp(pivot_x.rotation.x, deg_to_rad(max_rotation_down), deg_to_rad(max_rotation_up))
