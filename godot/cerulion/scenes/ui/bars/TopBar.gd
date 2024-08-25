extends PanelContainer

@onready var date_label:Label = $MarginContainer/HBoxContainer/HBoxContainer/DateLabel
@onready var time_label:Label = $MarginContainer/HBoxContainer/HBoxContainer/TimeLabel
@onready var meridian_label:Label = $MarginContainer/HBoxContainer/HBoxContainer/MeridianLabel

const month_dict:Dictionary = {
	1: "Jan", 2: "Feb", 3: "Mar", 4: "Apr", 5: "May", 6: "Jun",
	7: "Jul", 8: "Aug", 9: "Sep", 10: "Oct", 11: "Nov", 12: "Dec"
}
const weekday_dict:Dictionary = {
	0: "Sun", 1: "Mon", 2: "Tue", 3: "Wed", 4: "Thu", 5: "Fri", 6: "Sat"
}

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	pass # Replace with function body.


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(_delta: float) -> void:
	setClockLabel()

func setClockLabel() -> void:
	var time_data = Time.get_datetime_dict_from_system()
	date_label.text = weekday_dict[time_data.weekday] + " " \
		+ month_dict[time_data.month] + " " \
		+ str(time_data.day) + " " \
		+ str(time_data.year) + "   "
	time_label.text = str(time_data.hour % 12) + ":"
	if time_data.minute < 10:
		time_label.text += "0"
	time_label.text += str(time_data.minute) + ":"
	if time_data.second < 10:
		time_label.text += "0"
	time_label.text += str(time_data.second) + "  "
	if time_data.hour/12 < 1:
		meridian_label.text = "AM"
	else:
		meridian_label.text = "PM"
	
