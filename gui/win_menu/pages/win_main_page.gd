extends Page


@export var exit_button: Button



# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	super._ready()
	
	exit_button.pressed.connect(_on_exit_button_pressed)


func _on_exit_button_pressed() -> void:
	GAME.exit_to_menu()
