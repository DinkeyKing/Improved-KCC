extends Control
class_name Page


@export var page_name: String
var parent_menu: Menu


func _ready() -> void:
	parent_menu = get_parent() as Menu
	
	if parent_menu.selected_page_name != page_name:
		visible = false
	
	parent_menu.selected_page_changed.connect(_on_selected_page_changed)


func _on_selected_page_changed(selected_page_name: String):
	if selected_page_name == page_name:
		visible = true
	else: 
		visible = false
