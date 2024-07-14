extends Control

var start : Vector2
var initialPosition : Vector2
var isResizing : bool
var resizeX : bool
var initialSize : Vector2
@export var ResizeThreshold := 20

var panelStatus = true
var lastPanelWidth : float
var minimizedPanelWidth : int = 50

var isRightPanel : bool
@onready var panelNode : Node = $Margin/ContentContainer/Windows
@onready var resizeButtonNode : Node = $Margin/ContentContainer/ResizeButton

# SIGNAL FOR CURSOR ICON NECESSARY
# func _adjustCursorIcon():
# 	var rect = get_global_rect()
# 	var localMousePos = get_global_mouse_position() - get_global_position()
# 	if abs(localMousePos.x - rect.size.x) < ResizeThreshold:
# 		Input.set_default_cursor_shape(Input.CURSOR_HSIZE)
# 	else:
# 		Input.set_default_cursor_shape(Input.CURSOR_ARROW)

func _ready():
	if name.contains("Right"):
		isRightPanel = true

func _input(event):
	var rect = get_global_rect()
	var newWidith = get_size().x
	var newHeight = get_size().y
	if Input.is_action_just_pressed("LeftMouseDown"):
		var localMousePos = event.position - get_global_position()
		if abs(localMousePos.x - rect.size.x) < ResizeThreshold:
			start.x = event.position.x
			initialSize.x = get_size().x
			resizeX = true
			isResizing = true
			
		
	if Input.is_action_pressed("LeftMouseDown"):
		if isResizing:
			if resizeX:
				newWidith = initialSize.x - (start.x - event.position.x)
			if initialPosition.x != 0:
				newWidith = initialSize.x + (start.x - event.position.x)
				set_position(Vector2(initialPosition.x - (newWidith - initialSize.x), get_position().y))
			set_size(Vector2(newWidith, newHeight))
			
		
	if Input.is_action_just_released("LeftMouseDown"):
		initialPosition = Vector2(0,0)
		resizeX = false
		isResizing = false

func _on_resize_button_pressed():
	pass # Replace with function body.

func _on_minimize_button_pressed():
	panelStatus = not panelStatus
	if panelStatus:
		panelNode.set_visible(panelStatus)
		resizeButtonNode.set_visible(panelStatus)
		set_size(Vector2(lastPanelWidth, get_size().y))
		if isRightPanel:
			set_position(Vector2(get_position().x - lastPanelWidth + minimizedPanelWidth, get_position().y))
	else:
		lastPanelWidth = get_size().x
		panelNode.set_visible(panelStatus)
		resizeButtonNode.set_visible(panelStatus)
		set_size(Vector2(minimizedPanelWidth, get_size().y))
		if isRightPanel:
			set_position(Vector2(get_position().x + lastPanelWidth - minimizedPanelWidth, get_position().y))