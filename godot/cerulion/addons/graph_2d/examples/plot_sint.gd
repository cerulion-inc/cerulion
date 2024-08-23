@tool
extends Control

var plot_sin
var x = 0.0
var draw_enabled = false:
	set(value):
		draw_enabled = value
#		if is_instance_valid($Graph2D):
#			$Graph2D.background_color = Color.SLATE_GRAY if draw_enabled else Color.BLACK

func _ready():
	plot_sin = $Graph2D.add_plot_item("Sin(x)", Color.RED, 0.5)

func _process(_delta):
	if draw_enabled:
		var y: float = sin(x)
		plot_sin.add_point(Vector2(x,y))
		x += 0.1
	
	if draw_enabled and x > $Graph2D.x_max:
		draw_enabled = false

func _on_draw_button_pressed() -> void:
	draw_enabled = true
	plot_sin.remove_all()
	x = 0.0


func _on_clear_button_pressed() -> void:
	draw_enabled = false
	plot_sin.remove_all()
	x = 0.0
