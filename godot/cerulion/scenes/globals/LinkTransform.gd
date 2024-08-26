extends Node3D
class_name LinkTransform

# Transform constants
const godot_R_world:Basis = Basis(
	Vector3.BACK,
	Vector3.RIGHT,
	Vector3.UP)
const world_R_godot:Basis = Basis(
	Vector3(0, 1, 0),
	Vector3(0, 0, 1),
	Vector3(1, 0, 0))
const I:Basis = Basis.IDENTITY
const T_default:Transform3D = Transform3D(
	Basis.IDENTITY,
	Vector3.ZERO)

# Body link variables
var is_root:bool = true
var R_mesh:Basis = Basis.IDENTITY
var link_T_godot:Transform3D = T_default
var link_T_world:Transform3D = T_default
var mesh_T_world:Transform3D = T_default
var mesh_T_godot:Transform3D = T_default
var world_R_parent:Basis = Basis.IDENTITY
var parent_R_offset:Basis = Basis.IDENTITY
var offset_R_joint:Basis = Basis.IDENTITY
var joint_R_link:Basis = Basis.IDENTITY
var world_p_parent:Vector3 = Vector3.ZERO
var world_p_joint:Vector3 = Vector3.ZERO
var world_p_link:Vector3 = Vector3.ZERO

# Parent link variables
var q:float = 0
var parent_node:Node3D
var joint_axis:Vector3 = Vector3.ZERO
var joint_pos_rel_parent:Vector3 = Vector3.ZERO
var joint_rpy_rel_parent:Vector3 = Vector3.ZERO
var joint_ZYX_rel_parent:Vector3 = Vector3.ZERO
var link_pos_rel_joint:Vector3 = Vector3.ZERO
var link_rpy_rel_joint:Vector3 = Vector3.ZERO
var link_ZYX_rel_joint:Vector3 = Vector3.ZERO
var mesh_pos_offset:Vector3 = Vector3.ZERO
var mesh_rpy_offset:Vector3 = Vector3.ZERO

# Functions
func initializeMeshTransform() -> Transform3D:
	mesh_T_world.origin = mesh_pos_offset
	var mesh_ZYX_offset:Vector3 = Vector3(
		mesh_rpy_offset.y,
		mesh_rpy_offset.z,
		mesh_rpy_offset.x)
	mesh_T_world = mesh_T_world.rotated(Vector3(0, 0, 1), mesh_ZYX_offset.z)
	mesh_T_world = mesh_T_world.rotated(Vector3(0, 1, 0), mesh_ZYX_offset.y)
	mesh_T_world = mesh_T_world.rotated(Vector3(1, 0, 0), mesh_ZYX_offset.x)
	mesh_T_world.basis = mesh_T_world.basis * R_mesh
	mesh_T_godot = convertWorldTransformToGodot(mesh_T_world)
	return mesh_T_godot

func initializeLinkTransform() -> Transform3D:
	link_T_world.origin = Vector3.ZERO
	link_T_world.basis = Basis.IDENTITY
	link_T_godot = convertWorldTransformToGodot(link_T_world)
	return link_T_godot

func getLinkTransform(q_joint:float) -> Transform3D:
	# Rotation
	world_R_parent = world_R_godot * parent_node.transform.basis
	parent_R_offset = Basis.from_euler(joint_ZYX_rel_parent, EULER_ORDER_ZYX)
	if (joint_axis.dot(joint_axis) < 1e-6):
		offset_R_joint = Basis.IDENTITY
	else:
		offset_R_joint = Basis(joint_axis, q_joint)
	joint_R_link = Basis.from_euler(link_ZYX_rel_joint, EULER_ORDER_ZYX)
	var world_R_link:Basis = world_R_parent * parent_R_offset \
		* offset_R_joint * joint_R_link;
	# Translation
	world_p_parent = world_R_godot * parent_node.transform.origin
	world_p_joint = world_p_parent + world_R_parent * joint_pos_rel_parent
	world_p_link = world_p_joint + world_R_parent * parent_R_offset \
		* offset_R_joint * link_pos_rel_joint
	
	link_T_world.basis = world_R_link
	link_T_world.origin = world_p_link
	link_T_godot = convertWorldTransformToGodot(link_T_world)
	return link_T_godot

func getBaseTransformT(T_base:Transform3D) -> Transform3D:
	link_T_world = T_base
	link_T_godot = convertWorldTransformToGodot(link_T_world)
	return link_T_godot

func getBaseTransformR(R:Basis, p_base:Vector3) -> Transform3D:
	link_T_world.basis = R
	link_T_world.origin = p_base
	link_T_godot = convertWorldTransformToGodot(link_T_world)
	return link_T_godot

func getBaseTransformAxisAngle(axis:Vector3, angle:float, p_base:Vector3) -> Transform3D:
	link_T_world.basis = Basis(axis, angle)
	link_T_world.origin = p_base
	link_T_godot = convertWorldTransformToGodot(link_T_world)
	return link_T_godot

func getBaseTransformRPY(rpy:Vector3, p_base:Vector3) -> Transform3D:
	var euler_ZYX:Vector3 = Vector3(rpy.z, rpy.y, rpy.x)
	link_T_world.basis = Basis.from_euler(euler_ZYX, EULER_ORDER_ZYX)
	link_T_world.origin = p_base
	link_T_godot = convertWorldTransformToGodot(link_T_world)
	return link_T_godot

func getBaseTransformQuat(q_base:Quaternion, p_base:Vector3) -> Transform3D:
	link_T_world.basis = Basis(q_base)
	link_T_world.origin = p_base
	link_T_godot = convertWorldTransformToGodot(link_T_world)
	return link_T_godot

static func convertWorldTransformToGodot(T_world:Transform3D) -> Transform3D:
		var T_godot:Transform3D
		T_godot.origin.x = T_world.origin.y
		T_godot.origin.y = T_world.origin.z
		T_godot.origin.z = T_world.origin.x
		T_godot.basis = godot_R_world * T_world.basis
		return T_godot
	
