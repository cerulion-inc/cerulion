@tool
class_name Graph2D
extends Control
## A graph for representing data on a two-dimensional axis system as plots
##
## Graph2D allows you to create and display several plots simultaneously.[br][br]
## To add a new plot, call the [method add_plot_item]. It returns an object of type [PlotItem].[br]
## To add points to the plot, call the [method PlotItem.add_point].

#region Export variables

@export_group("X Axis")
## Minimun value on X-axis
@export var x_min: float = 0.0:
	set(value):
		if value < x_max:
			_x_step = _get_min_step(value, x_max)
			x_min = _get_min_value(value, x_max, _x_step)
			_update_graph()
			_update_plots()
## Maximum value on X-axis
@export var x_max: float = 10.0:
	set(value):
		if value > x_min:
			_x_step = _get_min_step(x_min, value)
			x_max = _get_max_value(x_min, value, _x_step)
			_update_graph()
			_update_plots()
## Label of X-axis
@export var x_label: String = "":
	set(value):
		x_label = value
		_update_graph()
## Shows ticks on the X-axis
@export var show_x_ticks: bool = true:
	set(value):
		show_x_ticks = value
		_update_graph()
## Shows numbers on the X-axis
@export var show_x_numbers: bool = true:
	set(value):
		show_x_numbers = value
		_update_graph()
## Shows line on the X-axis
@export var show_horizontal_line: bool = true:
	set(value):
		show_horizontal_line = value
		_update_graph()

@export_group("Y Axis")
## Minimun value on Y-axis
@export var y_min = 0.0:
	set(value):
		if value < y_max:
			_y_step = _get_min_step(value, y_max)
#			print_debug("y step: ", y_step)
			y_min = _get_min_value(value, y_max, _y_step)
#			print_debug("y_min: ", y_min)
			_update_graph()
			_update_plots()
## Maximum value on Y-axis
@export var y_max = 1.0:
	set(value):
		if value > y_min:
			_y_step = _get_min_step(y_min, value)
			y_max = _get_max_value(y_min, value, _y_step)
#			print_debug("y_max: ", y_max)
			_update_graph()
			_update_plots()

## Lable of Y-axis
@export var y_label: String = "":
	set(value):
		y_label = value
		_update_graph()
## Shows ticks on the Y-axis
@export var show_y_ticks: bool = true:
	set(value):
		show_y_ticks = value
		_update_graph()
## Shows numbers on the Y-axis
@export var show_y_numbers: bool = true:
	set(value):
		show_y_numbers = value
		_update_graph()
## Shows line of the Y-axis
@export var show_vertical_line: bool = true:
	set(value):
		show_vertical_line = value
		_update_graph()
		
@export_group("Background")
## Background color of graph
@export var background_color = Color.BLACK:
	set(value):
		background_color = value
		if get_node_or_null("Background"):
			get_node("Background").color = background_color
## Shows horizontal grid
@export var grid_horizontal_visible = false:
	set(value):
		grid_horizontal_visible = value
		_update_graph()
## Horizontal grid color
@export var grid_horizontal_color: Color = Color(1,1,1,0.3):
	set(value):
		grid_horizontal_color = value
		_update_graph()
## Shows vertical grid
@export var grid_vertical_visible = false:
	set(value):
		grid_vertical_visible = value
		_update_graph()
## Vertical grid color
@export var grid_vertical_color: Color = Color(1,1,1,0.3):
	set(value):
		grid_vertical_color = value
		_update_graph()

#endregion

#region Private variables

const _MARGIN_TOP = 30
const _MARGIN_BOTTOM = 30
const _MARGIN_LEFT = 45
const _MARGIN_RIGHT = 30

const _Graph2DAxis = preload("res://addons/graph_2d/custom_nodes/axis.gd")
const _Graph2DCoord = preload("res://addons/graph_2d/custom_nodes/coordinate.gd")
const _Graph2DGrid = preload("res://addons/graph_2d/custom_nodes/grid.gd")
const _Graph2DLegend = preload("res://addons/graph_2d/custom_nodes/legend.gd")

var _plots: Array

var _x_step: float
var _y_step: float

#endregion

func _ready():
	var background = ColorRect.new()
	background.name = "Background"
	background.color = background_color
	background.anchor_right = 1.0
	background.anchor_bottom = 1.0
	add_child(background)
	
	var plot_area = Control.new()
	
	plot_area.name = "PlotArea"
	plot_area.anchor_right = 1.0
	plot_area.anchor_bottom = 1.0
	plot_area.offset_left = _MARGIN_LEFT
	plot_area.offset_top = _MARGIN_TOP
	plot_area.offset_right = -_MARGIN_RIGHT
	plot_area.offset_bottom = -_MARGIN_BOTTOM
	add_child(plot_area)
	
	var axis = _Graph2DAxis.new()
	add_child(axis)
	
	var grid = _Graph2DGrid.new()
	add_child(grid)
	
	var legend = _Graph2DLegend.new()
	plot_area.add_child(legend)
	
	var coordinate = _Graph2DCoord.new()
	plot_area.add_child(coordinate)
	
	resized.connect(_on_Graph_resized)
	plot_area.resized.connect(_on_plot_area_resized)

	move_child(grid, 0)
	move_child(axis, 0)
	move_child(plot_area, 0)
	move_child(background, 0)
	
	_update_graph()

func _input(event: InputEvent) -> void:

	if event is InputEventMouseMotion:
		var plot_rect: Rect2 = Rect2(Vector2.ZERO, get_node("PlotArea").size)
		
		if plot_rect.has_point(get_node("PlotArea").get_local_mouse_position()):
			var pos: Vector2i = get_node("PlotArea").get_local_mouse_position()
			var point = _pixel_to_coordinate(pos)
			get_node("PlotArea/Coordinate").text = "(%.3f, %.3f)" % [point.x, point.y]

## Add plot to the graph and return an instance of plot.
func add_plot_item(label = "", color = Color.WHITE, width = 1.0) -> PlotItem:
	var plot = PlotItem.new(self, label, color, width)
	_plots.append(plot)
	_update_legend()
	return plot

## Remove plot from the graph.
func remove_plot_item(plot: PlotItem):
	# remove from plot_list
	var new_plot_list = _plots.filter(func(p): return p!=plot)
	_plots = new_plot_list

	plot.delete()
	_update_legend()

## Remove all plots inside graph.
func remove_all() -> void:
	for p:PlotItem in _plots:
		remove_plot_item(p)

## Return number of plots
func count() -> int:
	return _plots.size()


func _pixel_to_coordinate(px: Vector2i) -> Vector2:
	var point: Vector2
	point.x = remap(px.x, 0, get_node("PlotArea").size.x, x_min, x_max)
	point.y = remap(px.y, 0, get_node("PlotArea").size.y, y_max, y_min)
	return point

func _update_graph() -> void:
	if get_node_or_null("Axis") == null: return
	if get_node_or_null("Grid") == null: return
	if get_node_or_null("PlotArea") == null: return
	
	# Update margins depend of axis labels
	get_node("Axis").x_label = x_label
	get_node("Axis").y_label = y_label
	get_node("Axis").show_x_ticks = show_x_ticks
	get_node("Axis").show_x_numbers = show_x_numbers
	get_node("Axis").show_horizontal_line = show_horizontal_line
	get_node("Axis").show_y_ticks = show_y_ticks
	get_node("Axis").show_y_numbers = show_y_numbers
	get_node("Axis").show_vertical_line = show_vertical_line
	get_node("Grid").grid_horizontal_color = grid_horizontal_color
	get_node("Grid").grid_vertical_color= grid_vertical_color
	var margin_left: float = _MARGIN_LEFT if get_node("Axis").y_label == "" else _MARGIN_LEFT + 20
	var margin_bottom: float = _MARGIN_BOTTOM if get_node("Axis").x_label == "" else _MARGIN_BOTTOM + 20
	
	get_node("PlotArea").offset_left = margin_left
	get_node("PlotArea").offset_bottom = -margin_bottom
	
	# Vertical Graduation
	var y_step = _get_min_step(y_min, y_max)
	assert(not is_inf(y_step), "y_step is infinite!")

	var y_axis_range: float = y_max - y_min
	var vert_grad_number = _get_graduation_num(y_min, y_max, y_step, "vert")
	
	# Horizontal Graduation
	var x_step = _get_min_step(x_min, x_max)
	assert(not is_inf(x_step), "y_step is infinite!")
	
	var x_axis_range: float = x_max - x_min
	var hor_grad_number = _get_graduation_num(x_min, x_max, x_step, "hor")
	
	# Plot area height in pixel
	var area_height = size.y - _MARGIN_TOP - margin_bottom
	var vert_grad_step_px = area_height / (vert_grad_number - 1)
	# Plot area width in pixel
	var area_width = size.x - margin_left - _MARGIN_RIGHT
	var hor_grad_step_px = area_width / (hor_grad_number -1)
	
	var vert_grad: Array
	var hor_grid: Array
	var grad_px: Vector2
	grad_px.x = margin_left
	# Draw grid number
	for n in range(vert_grad_number):
		var grad: Array = []
		grad_px.y = _MARGIN_TOP + n * vert_grad_step_px
		grad.append(grad_px)
		var grad_text = "%0.1f" % (float(y_max) - n * float(y_axis_range)/(vert_grad_number-1))
		grad.append(grad_text)
		vert_grad.append(grad)
		
		# Horizontal grid
		if grid_horizontal_visible:
			var grid_px: PackedVector2Array
			grid_px.append(grad_px)
			grid_px.append(Vector2(grad_px.x + area_width, grad_px.y))
			hor_grid.append(grid_px)
			
	get_node("Axis").vert_grad = vert_grad
	
	if grid_horizontal_visible:
		get_node("Grid").hor_grid = hor_grid
	else:
		get_node("Grid").hor_grid = []
		
	var hor_grad: Array
	var vert_grid: Array
	grad_px = Vector2()
	grad_px.y = _MARGIN_TOP + area_height
	
	for n in range(hor_grad_number):
		var grad: Array = []
		grad_px.x = margin_left + n * hor_grad_step_px
		grad.append(grad_px)
		var grad_text = "%0.1f" % (float(x_min) + n * float(x_axis_range)/(hor_grad_number-1))
		grad.append(grad_text)
		hor_grad.append(grad)
		
		# Vertical grid
		if grid_vertical_visible:
			var grid_px: PackedVector2Array
			grid_px.append(grad_px)
			grid_px.append(Vector2(grad_px.x, grad_px.y - area_height))
			vert_grid.append(grid_px)
		
	get_node("Axis").hor_grad = hor_grad
	
	if grid_vertical_visible:
		get_node("Grid").vert_grid = vert_grid
	else:
		get_node("Grid").vert_grid = []
	
	get_node("Axis").queue_redraw()
	get_node("Grid").queue_redraw()
	
func _update_plots():
	for plot in _plots:
		plot._redraw()

func _update_legend() -> void:
	# Add labels to the legend
	var labels = Array()
	for p in _plots:
		labels.append({
			name = p.label,
			color = p.color,
		})
	get_node("PlotArea/Legend").update(labels)

func _on_Graph_resized() -> void:
	_update_graph()
	
func _on_plot_area_resized() -> void:
	_update_plots()

# This function return the minimal step
func _get_min_step(value_min, value_max):
	var range_log: int = int(_log10(value_max - value_min))
	var step: float = 10.0**(range_log-1)
#	print("min step: %f " % [step])
	return step

func _get_graduation_num(value_min, value_max, step, orientation) -> int:
	var diff = value_max - value_min
	var nb_grad: int = roundi(diff/step)
	var max_grad_num: int
	match orientation:
		"vert":
			if size.y < 250: max_grad_num = 5
			else: max_grad_num = 10
		"hor":
			if size.x < 450: max_grad_num = 5
			else: max_grad_num = 10
	
	while nb_grad > max_grad_num:
#		print("->", nb_grad)
		if not nb_grad % 2:
			nb_grad /= 2
			continue
		elif not nb_grad % 3:
			nb_grad /= 3
			continue
		elif not nb_grad % 5:
			nb_grad /= 5
			continue
		elif not nb_grad % 7:
			nb_grad /= 7
			continue
		elif not nb_grad % 9:
			nb_grad /= 9
			continue
		else:
			# not divided
			break
#			return nb_grad + 1
			
#	print("diff: %f , nb_grad: %d" % [diff, nb_grad])
	return nb_grad + 1
	
func _get_min_value(min_value, max_value, step):
	var min_token = roundf(min_value/step) * step
	
	while true:
		var diff = max_value - min_token
		var nb_grad: int = roundi(diff/step)
		
		while nb_grad > 10:
	#		print("->", nb_grad)
			if not nb_grad % 2:
				nb_grad /= 2
				continue
			elif not nb_grad % 3:
				nb_grad /= 3
				continue
			elif not nb_grad % 5:
				nb_grad /= 5
				continue
			elif not nb_grad % 7:
				nb_grad /= 7
				continue
			elif not nb_grad % 9:
				nb_grad /= 9
				continue
			else:
				# not divided
				break
		if nb_grad <= 10:
			return min_token
		min_token -= step
	
func _get_max_value(min_value, max_value, step):
	var max_token = roundf(max_value/step) * step
	
	while true:
		var diff = max_token - min_value
		var nb_grad: int = roundi(diff/step)

		while nb_grad > 10:
	#		print("->", nb_grad)
			if not nb_grad % 2:
				nb_grad /= 2
				continue
			elif not nb_grad % 3:
				nb_grad /= 3
				continue
			elif not nb_grad % 5:
				nb_grad /= 5
				continue
			elif not nb_grad % 7:
				nb_grad /= 7
				continue
			elif not nb_grad % 9:
				nb_grad /= 9
				continue
			else:
				# not divided
				break
		if nb_grad <= 10:
			return max_token
		max_token += step
		
func _log10(value: float) -> float:
	return log(value)/log(10)
