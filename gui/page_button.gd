extends Button
class_name PageButton

@export var target_page_name: String
@export var page: Page


# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	pressed.connect(_on_button_pressed)



func _on_button_pressed() -> void:
	page.parent_menu.selected_page_name = target_page_name
