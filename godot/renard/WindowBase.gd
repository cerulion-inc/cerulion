extends Control

var start : Vector2
var initialPosition : Vector2
var isMoving : bool
var isResizing : bool
var resizeX : bool
var resizeY : bool
var initialSize : Vector2
@export var GrabThreshold := 20
@export var ResizeThreshold := 5

func _input(event):
	if Input.is_action_just_pressed("LeftMouseDown"):
		var rect = get_global_rect()
		var localMousePos = event.position - get_global_position()
		if localMousePos.y < GrabThreshold:
			start = event.position
			initialPosition = get_global_position()
			isMoving = true
		else:
			if abs(localMousePos.x - rect.size.x) < ResizeThreshold:
				start.x = event.position.x
				initialSize.x = get_size().x
				resizeX = true
				isResizing = true
			
			if abs(localMousePos.y - rect.size.y) < ResizeThreshold:
				start.y = event.position.y
				initialSize.y = get_size().y
				resizeY = true
				isResizing = true
			
			if localMousePos.x < ResizeThreshold &&  localMousePos.x > -ResizeThreshold:
				start.x = event.position.x
				initialPosition.x = get_global_position().x
				initialSize.x = get_size().x
				isResizing = true
				resizeX = true
				
			if localMousePos.y < ResizeThreshold &&  localMousePos.y > -ResizeThreshold:
				start.y = event.position.y
				initialPosition.y = get_global_position().y
				initialSize.y = get_size().y
				isResizing = true
				resizeY = true
		
	if Input.is_action_pressed("LeftMouseDown"):
		if isMoving:
			set_position(initialPosition + (event.position - start))
		
		if isResizing:
			var newWidith = get_size().x
			var newHeight = get_size().y
			
			if resizeX:
				newWidith = initialSize.x - (start.x - event.position.x)
			if resizeY:
				newHeight = initialSize.y - (start.y - event.position.y)
				
			if initialPosition.x != 0:
				newWidith = initialSize.x + (start.x - event.position.x)
				set_position(Vector2(initialPosition.x - (newWidith - initialSize.x), get_position().y))
			
			if initialPosition.y != 0:
				newHeight = initialSize.y + (start.y - event.position.y)
				set_position(Vector2(get_position().x, initialPosition.y - (newHeight - initialSize.y)))
			
			set_size(Vector2(newWidith, newHeight))
			
		
	if Input.is_action_just_released("LeftMouseDown"):
		isMoving = false
		initialPosition = Vector2(0,0)
		resizeX = false
		resizeY = false
		isResizing = false
