extends Node3D

var joints:Array
var links:Array
var time:float = 0
var robot_loaded:bool = false

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	Signals.connect("RobotLoaded", loadRobot)

func loadRobot():
	print(RobotParameters.joints)
	print(RobotParameters.links)
	joints = RobotParameters.joints.keys()
	links = RobotParameters.links.keys()
	for l in links:
		if (RobotParameters.links[l].visual.geometry.mesh):
			var link_node = Node3D.new()
			var link_mesh = MeshInstance3D.new()
			var link_transforms = LinkTransform.new()
			link_mesh.mesh = RobotParameters.links[l].visual.geometry.obj
			link_node.add_child(link_mesh)
			link_transforms.link_pos_rel_joint = RobotParameters.links[l].visual.origin.xyz
			link_transforms.mesh_rpy_offset = RobotParameters.links[l].visual.origin.rpy
			# link_transforms.mesh_pos_offset = RobotParameters.links[l].visual.origin.xyz
			link_transforms.R_mesh = Basis(
				Vector3(0, 1, 0),
				Vector3(1, 0, 0),
				Vector3(0, 0, -1))
			RobotState.link_nodes[l] = link_node
			RobotState.link_transforms[l] = link_transforms
			self.add_child(link_node)
	for j in joints:
		if (RobotParameters.joints[j].type != "fixed"):
			var child = RobotParameters.joints[j].child
			RobotState.link_transforms[child].joint_axis = RobotParameters.joints[j].axis
			RobotState.link_transforms[child].joint_pos_rel_parent = RobotParameters.joints[j].origin.xyz
			RobotState.link_transforms[child].joint_rpy_rel_parent = RobotParameters.joints[j].origin.rpy
			RobotState.link_transforms[child].is_root = false
			if (RobotParameters.joints[j].has("parent")):
				RobotState.link_transforms[child].parent_node \
					= RobotState.link_nodes[RobotParameters.joints[j].parent]
	initializeTransforms()
	robot_loaded = true

func initializeTransforms() -> void:
	for key in RobotState.link_transforms.keys():
		(RobotState.link_nodes[key].get_child(0)).transform = \
			RobotState.link_transforms[key].initializeMeshTransform()
		RobotState.link_nodes[key].transform = \
			RobotState.link_transforms[key].initializeLinkTransform()
		if RobotState.link_transforms[key].is_root:
			RobotState.joints[key] = PackedFloat64Array([0, 0, 0, 0, 0, 0, 0])
			RobotState.link_nodes[key].transform = \
				RobotState.link_transforms[key].getBaseTransformT(LinkTransform.T_default)
			RobotState.link_nodes[key].transform.origin = Vector3(0, 1, 0)
		else:
			RobotState.joints[key] = 0.0
			RobotState.link_nodes[key].transform = \
				RobotState.link_transforms[key].getLinkTransform(0.0)

func _physics_process(delta: float) -> void:
	if robot_loaded:
		for key in RobotState.link_transforms.keys():
			if RobotState.link_transforms[key].is_root:
				var rpy = Vector3(sin(5*time), sin(5*time), sin(5*time)) * 0.5
				var p_base = Vector3(sin(time), cos(time), 1)
				RobotState.link_nodes[key].transform = \
					RobotState.link_transforms[key].getBaseTransformRPY(rpy, p_base)
			else:
				RobotState.link_nodes[key].transform = \
					RobotState.link_transforms[key].getLinkTransform(sin(time))
		time += delta
