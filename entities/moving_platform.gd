extends Node3D

#########
# NODES #
#########

@onready var animation_player := $AnimationPlayer as AnimationPlayer


#############
# FUNCTIONS #
#############

func _ready() -> void :
	animation_player.play("move")
