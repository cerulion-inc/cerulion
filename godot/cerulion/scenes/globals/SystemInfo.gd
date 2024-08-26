extends Node

var os_name:String = ""

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	os_name = OS.get_name()
