extends Window

@onready var import_window = $"."

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	import_window.hide()

func _on_close_requested() -> void:
	import_window.hide()

func _on_import_button_pressed() -> void:
	import_window.show()
