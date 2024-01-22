extends Node3D

##############
# PROPERTIES #
##############

@export var animation_name : String = "moving_platform/move"


#########
# NODES #
#########

@onready var animation_player := $AnimationPlayer as AnimationPlayer


#############
# FUNCTIONS #
#############

func _ready() -> void :
	animation_player.play(animation_name)
