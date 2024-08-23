@tool
extends Control

var vert_grid: Array
var hor_grid: Array
var grid_vertical_color: Color = Color(1,1,1,0.3)
var grid_horizontal_color: Color = Color(1,1,1,0.3)

func _ready():
	name = "Grid"

func _draw() -> void:
	for line in hor_grid:
		draw_line(line[0], line[1], grid_horizontal_color)
		
	for line in vert_grid:
		draw_line(line[0], line[1], grid_vertical_color)

