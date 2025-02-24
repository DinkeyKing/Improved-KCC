extends Page


@export var level_button_container : Node


func _ready() -> void:
	super._ready()
	
	var level_buttons : Array[Node] = level_button_container.get_children()
	
	for button : DataButton in level_buttons:
		if button.data.has("map_path"):
			button.pressed.connect(load_level.bind(button.data["map_path"]))



func load_level(path: String) -> void:
	GAME.load_level(path)
