extends MarginContainer

@onready var channel_data_entry = $VBoxContainer/ChannelData

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	pass # Replace with function body.
	
	
# Data with five fields:
# 	channel_label: String
# 	status: bool
#	hz: float
# 	period: float
# 	bandwidth: float
func fillDataFields(channel_data_node, data) -> void:
	channel_data_node.get_child("ChannelLabel").label = data.channel_label
	channel_data_node.get_child("HzLabel").label = str(data.hz)
	channel_data_node.get_child("PeriodLabel").label = str(data.period) + " ms"
	channel_data_node.get_child("BandwidthLabel").label = str(data.bandwidth) + " KB/s"
	if (data.status):
		channel_data_node.get_child("Status").texture = "res://assets/textures/ui/dark/status/"
	else:
		channel_data_node.get_child("Status").texture = "res://assets/textures/ui/dark/status/"
