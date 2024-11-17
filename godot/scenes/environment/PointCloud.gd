extends Node3D

var test 

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	test = PointCloudData.data[0]

func _process(_delta: float) -> void:
	test.x += _delta
	# PointCloudData.data is a PackedVector4Array, which is meant to contain location
	# + intensity values from the sensors
	# I'm not too sure how the values map, so I'll leave this to you.

func addSphere(location, size=0.05):
	var sphere = MeshInstance3D.new()
	sphere.mesh = SphereMesh.new()
	sphere.mesh.radial_segments = 8
	sphere.mesh.rings = 8
	sphere.mesh.radius = size
	sphere.mesh.height = size * 2
	var material = StandardMaterial3D.new()
	material.albedo_color = Color(1, 0, 0)
	sphere.material_override = material
	PointCloudData.data.append(sphere)
	add_child(sphere)

func drawSphere(sphere_node, location, color):
	var material = StandardMaterial3D.new()
	material.albedo_color = Color(1, 0, 0)
	sphere_node.material_override = material
	sphere_node.transform.origin = location
