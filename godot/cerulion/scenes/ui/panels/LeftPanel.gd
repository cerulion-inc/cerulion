extends PanelContainer

@onready var import_window = $ImportWindow
@onready var add_robot_button = $HBoxContainer/ButtonContainer/MarginContainer/VBoxContainer/AddRobotButton
@onready var panel_content_window = $HBoxContainer/PanelContent

var is_showing_content = false

func _ready() -> void:
	import_window.hide()
	panel_content_window.hide()

func _on_add_robot_button_pressed() -> void:
	import_window.show()

func _on_import_window_close_requested() -> void:
	import_window.hide()

func _input(event):
	if event.is_action_pressed("toggle_content_panel"):
		is_showing_content = !is_showing_content
		if is_showing_content:
			panel_content_window.show()
		else:
			panel_content_window.hide()


func _on_button_pressed() -> void:
	# Resize content panel
	var test = 0
