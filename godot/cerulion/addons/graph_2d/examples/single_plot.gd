extends Control

func _on_add_plot_pressed() -> void:
	var my_plot = $Graph2D.add_plot_item(
			"Plot %d" % [$Graph2D.count()],
			[Color.RED, Color.GREEN, Color.BLUE][$Graph2D.count() % 3],
			[1.0, 3.0, 5.0].pick_random()
			)

	for x in range(0, 11, 1):
		var y = randf_range(-50, 50)
		my_plot.add_point(Vector2(x, y))


func _on_remove_all_plots_pressed() -> void:
	$Graph2D.remove_all()
