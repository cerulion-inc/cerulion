@tool
extends Control

var default_font: Font

enum {
	POINT = 0,
	LABEL,
}

var vert_grad: Array # [Vector2, String]
var hor_grad: Array
var x_label: String
var y_label: String
var show_x_ticks: bool
var show_y_ticks: bool
var show_x_numbers: bool
var show_y_numbers: bool
var show_vertical_line: bool
var show_horizontal_line: bool


func _ready():
	name = "Axis"
	default_font = ThemeDB.fallback_font
	var x_label_node = Label.new()
	x_label_node.name = "XLabel"
	add_child(x_label_node)
	var y_label_node = Label.new()
	y_label_node.name = "YLabel"
	y_label_node.rotation = -PI/2
	add_child(y_label_node)
	

func _draw() -> void:
	if vert_grad.is_empty() or hor_grad.is_empty(): return
	
	var topleft: Vector2 = vert_grad.front()[POINT]
	var topright: Vector2 = Vector2(hor_grad.back()[POINT].x, vert_grad.front()[POINT].y)
	var bottomright: Vector2 = hor_grad.back()[POINT]
	
	if show_x_ticks:
		for grad in hor_grad: 
			draw_line(grad[POINT], grad[POINT] + Vector2(0, 10), Color.WHITE)
	
	if show_x_numbers:
		for grad in hor_grad:
			draw_string(default_font, grad[POINT] + Vector2(0, 20), grad[LABEL])
	
	if show_horizontal_line == true:
		draw_line(hor_grad.front()[POINT], hor_grad.back()[POINT], Color.WHITE)

	if show_y_ticks == true:
		for grad in vert_grad:
			draw_line(grad[POINT], grad[POINT] - Vector2(10, 0), Color.WHITE)
	
	if show_y_numbers == true:
		for grad in vert_grad:
			draw_string(default_font, grad[0] + Vector2(-35, -5), grad[1])
		
	if show_vertical_line == true:
		draw_line(topleft, vert_grad.back()[POINT], Color.WHITE)

	get_node("XLabel").text = x_label
	get_node("YLabel").text = y_label 
	get_node("XLabel").position = Vector2((bottomright.x + topleft.x)/2, bottomright.y + 20)
	get_node("YLabel").position = Vector2(5, (bottomright.y + topleft.y)/2)
