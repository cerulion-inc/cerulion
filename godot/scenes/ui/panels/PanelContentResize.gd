extends HBoxContainer

@onready var status = $PanelContainer/Status
@onready var plots = $PanelContainer/Plots
@onready var controls = $PanelContainer/Control

var resizing_panel:bool = false
var pos_mouse_init:Vector2
var pos_rect_init:Rect2
var panel_content:Array

func _ready() -> void:
	self.hide()
	panel_content.append(status)
	panel_content.append(plots)
	panel_content.append(controls)
	for content in panel_content:
		content.hide()

func _input(event):
	if resizing_panel:
		var new_width = get_size().x
		new_width = pos_rect_init.size.x - (pos_mouse_init.x - event.position.x)
		set_size(Vector2(new_width, pos_rect_init.size.y))
	elif event.is_action_pressed("toggle_content_panel"):
		if is_visible_in_tree():
			self.hide()
		else:
			self.show()

func _on_resize_panel_content_button_gui_input(event: InputEvent) -> void:
	if (event.is_action("left_click")):
		resizing_panel = true
		pos_mouse_init = get_global_mouse_position()
		pos_rect_init = get_global_rect()
		print(pos_mouse_init)
		print(pos_rect_init)

func _on_resize_panel_content_button_button_up() -> void:
	resizing_panel = false

func _on_show_statuses_button_pressed() -> void:
	if (is_visible_in_tree()):
		if (status.is_visible_in_tree()):
			self.hide()
		else:
			for content in panel_content:
				content.hide()
			status.show()
	else:
		for content in panel_content:
			content.hide()
		self.show()
		status.show()


func _on_show_plots_button_pressed() -> void:
	if (is_visible_in_tree()):
		if (plots.is_visible_in_tree()):
			self.hide()
		else:
			for content in panel_content:
				content.hide()
			plots.show()
	else:
		for content in panel_content:
			content.hide()
		self.show()
		plots.show()


func _on_show_controls_button_pressed() -> void:
	if (is_visible_in_tree()):
		if (controls.is_visible_in_tree()):
			self.hide()
		else:
			for content in panel_content:
				content.hide()
			controls.show()
	else:
		for content in panel_content:
			content.hide()
		self.show()
		controls.show()
