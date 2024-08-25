extends Node3D

var robot_dir:DirAccess
var links:Array

var mesh_directory:String
var mesh_filepaths:Array
var meshes:Array

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	Signals.connect("RobotLoaded", loadRobot)

func loadRobot():
	robot_dir = DirAccess.open("user://robots")
	links = RobotParameters.links.keys()
	for l in links:
		if (RobotParameters.links[l].visual.geometry.mesh):
			var link_node = Node3D.new()
			var link_mesh = MeshInstance3D.new()
			link_mesh.mesh = RobotParameters.links[l].visual.geometry.obj
			link_node.add_child(link_mesh)
			self.add_child(link_node)
	

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	pass
