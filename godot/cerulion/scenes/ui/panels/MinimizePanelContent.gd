extends Panel

var show_panel

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	show_panel = false
	self.hide()

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	pass


func _on_minimize_button_toggled(_toggled_on: bool) -> void:
	show_panel = !show_panel
	if show_panel:
		self.show()
	else:
		self.hide()