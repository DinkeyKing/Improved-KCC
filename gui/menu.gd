extends Control
class_name Menu


signal selected_page_changed(selected_page_name: String)


@export var home_page_name: String = "main_page"


@export var selected_page_name: String :
	set(value):
		selected_page_name = value
		selected_page_changed.emit(selected_page_name)


func _ready() -> void:
	selected_page_name = home_page_name
