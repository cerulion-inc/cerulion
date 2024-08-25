extends PanelContainer

@onready var robot_name:Label = $HBoxContainer/MarginContainer/HBoxContainer/HBoxContainer2/RobotName
@onready var network_name:Label = $HBoxContainer/MarginContainer/HBoxContainer/HBoxContainer/NetworkName
@onready var os_name:Label = $HBoxContainer/MarginContainer/HBoxContainer/HBoxContainer3/OSName
var ip_addresses:Array

func _ready() -> void:
	Signals.connect("RobotLoaded", updateRobotText)
	for address in IP.get_local_addresses():
		if (address.split('.').size() == 4):
			ip_addresses.append(address)
	network_name.text = ip_addresses[0]
	os_name.text = SystemInfo.os_name
	
	
func updateRobotText():
	robot_name.text = RobotParameters.robot_name

func _on_screenshot_button_pressed() -> void:
	var filepath:String = OS.get_system_dir(OS.SYSTEM_DIR_DESKTOP) \
		+ "/cerulion_screenshot.png"
	print(filepath)
	var capture = get_viewport().get_texture().get_image()
	var _time = Time.get_datetime_string_from_system()
	capture.save_png(filepath)

func _on_info_button_pressed() -> void:
	OS.shell_open("https://www.cerulion.com")

func _on_help_button_pressed() -> void:
	OS.shell_open("https://github.com/cerulion-inc/cerulion/wiki")
