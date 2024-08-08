extends Control

@onready var chart: Chart = $VBoxContainer/Chart
@onready var plot_timer : Timer = $"../PlotTimer"

# This Chart will plot 3 different functions
var f1: Function

func _ready():
	# Let's create our @x values
	var x: PackedFloat32Array = ArrayOperations.multiply_float(range(0, 11, 1), 0.)
	
	# And our y values. It can be an n-size array of arrays.
	# NOTE: `x.size() == y.size()` or `x.size() == y[n].size()`
	var y: Array = ArrayOperations.multiply_int(ArrayOperations.cos(x), 0)
	
	# Let's customize the chart properties, which specify how the chart
	# should look, plus some additional elements like labels, the scale, etc...
	var cp: ChartProperties = ChartProperties.new()
	cp.colors.frame = Color("#161a1d")
	cp.colors.background = Color.TRANSPARENT
	cp.colors.grid = Color("#283442")
	cp.colors.ticks = Color("#283442")
	cp.colors.text = Color.WHITE_SMOKE
	cp.draw_bounding_box = false
	cp.title = ""
	cp.x_label = "Time (s)"
	cp.y_label = "Motor torques (Nm)"
	cp.x_scale = 5
	cp.y_scale = 10
	cp.interactive = true # false by default, it allows the chart to create a tooltip to show point values
	# and interecept clicks on the plot
	
	# Let's add values to our functions
	f1 = Function.new(
		x, y, "Pressure", # This will create a function with x and y values taken by the Arrays 
						# we have created previously. This function will also be named "Pressure"
						# as it contains 'pressure' values.
						# If set, the name of a function will be used both in the Legend
						# (if enabled thourgh ChartProperties) and on the Tooltip (if enabled).
		# Let's also provide a dictionary of configuration parameters for this specific function.
		{ 
			color = Color("#36a2eb"), 		# The color associated to this function
			marker = Function.Marker.CIRCLE, 	# The marker that will be displayed for each drawn point (x,y)
											# since it is `NONE`, no marker will be shown.
			type = Function.Type.LINE, 		# This defines what kind of plotting will be used, 
											# in this case it will be a Linear Chart.
			interpolation = Function.Interpolation.SPLINE	# Interpolation mode, only used for 
															# Line Charts and Area Charts.
		}
	)
	
	# Now let's plot our data
	chart.plot([f1], cp)
	
	# Uncommenting this line will show how real time data plotting works
	#set_process(false)
	plot_timer.autostart = true
	plot_timer.one_shot = false
	plot_timer.wait_time = 0.05
	plot_timer.start()


var new_val: float = 4.5
var rng = RandomNumberGenerator.new()
var can_plot = true

func _process(delta: float):
	# This function updates the values of a function and then updates the plot
	new_val += delta
	
	# we can use the `Function.add_point(x, y)` method to update a function
	if can_plot:
		f1.add_point(new_val, rng.randf_range(-10.0, 10.0))
		f1.remove_point(0)
		chart.queue_redraw() # This will force the Chart to be updated
		can_plot = false
	


#func _on_CheckButton_pressed():
	#set_process(not is_processing())


func _on_plot_timer_timeout():
	can_plot = true
