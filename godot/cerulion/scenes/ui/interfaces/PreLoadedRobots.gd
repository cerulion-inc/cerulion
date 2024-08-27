extends GridContainer

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	pass # Replace with function body.

func _on_unitree_button_pressed():
	Signals.emit_signal("LoadUnitree")
