extends Node

# Size of the circular data
var data_size: int = 1000
var data: PackedVector3Array
var head: int = 0
var tail: int = 0
var is_full: bool = false

func _ready():
	# Initialize the data with default Vector3 values
	data.resize(data_size)
	for i in range(data_size):
		data[i] = Vector3.ZERO

func enqueue(value: Vector3):
	data[tail] = value
	tail = (tail + 1) % data_size
	
	if is_full:
		head = (head + 1) % data_size
	else:
		if tail == head:
			is_full = true

func dequeue() -> Vector3:
	if is_empty():
		print("Buffer is empty")
		return Vector3.ZERO
	
	var value: Vector3 = data[head]
	head = (head + 1) % data_size
	is_full = false
	return value

func is_empty() -> bool:
	return not is_full and (head == tail)
