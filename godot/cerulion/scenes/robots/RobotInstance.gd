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
			
			# ! TODO: R_mesh is hardcoded. I'm not sure this is always true for all exported meshes.
			link_transforms.R_mesh = Basis(
				Vector3(0, 1, 0),
				Vector3(1, 0, 0),
				Vector3(0, 0, -1))
			RobotState.link_nodes[l] = link_node
			RobotState.link_transforms[l] = link_transforms
			self.add_child(link_node)
	for j in joints:
		print(j)
		var child = RobotParameters.joints[j].child
		if (RobotState.link_transforms.has(child)):
			RobotState.link_transforms[child].is_root = false
			RobotState.link_transforms[child].parent_node = RobotState.link_nodes[RobotParameters.joints[j].parent]
			if (RobotParameters.joints[j].type != "fixed"):
				RobotState.link_transforms[child].joint_axis = RobotParameters.joints[j].axis
				RobotState.link_transforms[child].joint_pos_rel_parent = RobotParameters.joints[j].origin.xyz
				RobotState.link_transforms[child].joint_rpy_rel_parent = RobotParameters.joints[j].origin.rpy
			else:
				RobotState.link_transforms[child].joint_axis = Vector3(1, 0, 0)
			
	initializeTransforms()
	robot_loaded = true

# ! TODO: CHANGE "is_root" EXCEPTION HANDLING TO "is_floating". 
# ! FIXED BASE ROBOTS SHOULD HAVE NO ISSUES
func initializeTransforms() -> void:
	for key in RobotState.link_transforms.keys():
		(RobotState.link_nodes[key].get_child(0)).transform = \
			RobotState.link_transforms[key].initializeMeshTransform()
		RobotState.link_nodes[key].transform = \
			RobotState.link_transforms[key].initializeLinkTransform()
		if RobotState.link_transforms[key].is_root:
			RobotState.joints[key] = PackedFloat32Array([0, 0, 0, 1, 0, 0, 0])
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
				var q_base:PackedFloat32Array = RobotState.joints[key]
				var base_quat:Quaternion = Quaternion(q_base[0], q_base[1], q_base[2], q_base[3]).normalized();
				var base_pos:Vector3 = Vector3(q_base[4], q_base[5], q_base[6]);
				RobotState.link_nodes[key].transform = \
					RobotState.link_transforms[key].getBaseTransformQuat(base_quat, base_pos)
			else:
				RobotState.link_nodes[key].transform = \
					RobotState.link_transforms[key].getLinkTransform(RobotState.joints[key]);
		time += delta
