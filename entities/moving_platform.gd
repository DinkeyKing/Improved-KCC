extends Node3D

#########
# NODES #
#########

@onready var animation_player := $AnimationPlayer as AnimationPlayer


#############
# FUNCTIONS #
#############

func _ready():
	animation_player.play("move")
