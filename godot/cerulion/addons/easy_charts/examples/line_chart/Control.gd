extends Control

@onready var chart: Chart = $VBoxContainer/Chart

# This Chart will plot 3 different functions
var f1: Function
var f2: Function
var cp: ChartProperties

func _ready():
	var x: PackedFloat32Array = ArrayOperations.multiply_float(range(-10, 11, 1), 0.5)
	var y: Array
	y.resize(x.size())
	y.fill(0)

	cp = ChartProperties.new()
	cp.colors.frame = Color("#161a1d")
	cp.colors.background = Color.TRANSPARENT
	cp.colors.grid = Color("#283442")
	cp.colors.ticks = Color("#283442")
	cp.colors.text = Color.WHITE_SMOKE
	cp.draw_bounding_box = false
	cp.title = ""
	cp.x_label = "Time"
	cp.y_label = "Sensor values"
	cp.x_scale = 5
	cp.y_scale = 10
	cp.interactive = true # false by default, it allows the chart to create a tooltip to show point values
	# and interecept clicks on the plot
	
	# Let's add values to our functions
	f1 = Function.new(
		x, y, "data",
		{ 
			color = Color("#36a2eb"), 		# The color associated to this function
			marker = Function.Marker.CIRCLE, 	# The marker that will be displayed for each drawn point (x,y)
											# since it is `NONE`, no marker will be shown.
			type = Function.Type.LINE, 		# This defines what kind of plotting will be used, 
											# in this case it will be a Linear Chart.
			interpolation = Function.Interpolation.SPLINE	
		}
	)
	f2 = Function.new(
		x, y, "data",
		{ 
			color = Color("#36a2eb"), 		# The color associated to this function
			marker = Function.Marker.CIRCLE, 	# The marker that will be displayed for each drawn point (x,y)
											# since it is `NONE`, no marker will be shown.
			type = Function.Type.LINE, 		# This defines what kind of plotting will be used, 
											# in this case it will be a Linear Chart.
			interpolation = Function.Interpolation.SPLINE	
		}
	)
	chart.plot([f1, f2], cp)
	#set_process(false)

var new_val: float = 4.5

func _process(delta: float):
	# This function updates the values of a function and then updates the plot
	new_val += 3*delta
	cp.x_label = "eee"
	# we can use the `Function.add_point(x, y)` method to update a function
	f1.add_point(new_val, cos(new_val) * 20)
	f1.remove_point(0)
	f2.add_point(new_val, sin(new_val) * 20)
	f2.remove_point(0)
	chart.queue_redraw() # This will force the Chart to be updated


func _on_CheckButton_pressed():
	set_process(not is_processing())
