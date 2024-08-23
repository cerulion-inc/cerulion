@tool
extends Control

var points_px := PackedVector2Array([])
var color: Color = Color.WHITE
var width: float = 1.0


func _draw() -> void:
	if points_px.size() > 1:
		draw_polyline(points_px, color, width, true)
