using System.Reflection.Metadata;
using Godot;

public partial class Link : Node3D
{
	public Link() {}

	// Transformation from "world" coordinate frame (x forward, y left, z up) to
	// Godot's coordinate frame (x right, y up, z forward)
	public static Basis R_godot = new(Vector3.Back,
									  Vector3.Right,
									  Vector3.Up);
	public static Basis R_world = R_godot.Transposed();
	public Basis R_mesh = Basis.Identity;
	public static Basis I = Basis.Identity;
	public static Transform3D T_default = new(I, Vector3.Zero);

    // ! URDF parameters
	// Body link variables
	public bool is_root = false;
    public string link_name = "";
	public Transform3D link_T_godot = T_default;
	public Transform3D link_T_world = T_default;
	public Transform3D mesh_T_world = T_default;
	public Transform3D mesh_T_godot = T_default;
	private Basis world_R_parent = I;
	private Basis parent_R_offset = I;
	private Basis offset_R_joint = I;
	private Basis joint_R_link = I;
	private Vector3 world_p_parent = Vector3.Zero;
	private Vector3 world_p_joint = Vector3.Zero;
	private Vector3 world_p_link = Vector3.Zero;
	public string mesh_name = "";
	public MeshInstance3D mesh_instance;

	// Parent link
	public string parent_name = "";
	public Link parent_node;
	public Transform3D parent_T_world = T_default;
	public Transform3D parent_T_godot = T_default;

	// Transform parameters (in world frame, from URDF)
	public float q;	// Generalized coordinate
	public Vector3 joint_axis = Vector3.Zero;
	public Vector3 joint_pos_rel_parent = Vector3.Zero; // p_P_jf/p
	public Vector3 joint_rpy_rel_parent = Vector3.Zero; // p_R_jf/p
	public Vector3 joint_ZYX_rel_parent = Vector3.Zero; // p_R_jf/p
	public Vector3 link_pos_rel_joint = Vector3.Zero;	// jf_P_l/jf
	public Vector3 link_rpy_rel_joint = Vector3.Zero;	// jf_R_l/jf
	public Vector3 link_ZYX_rel_joint = Vector3.Zero;	// jf_R_l/jf
	public Vector3 mesh_pos_offset = Vector3.Zero;
	public Vector3 mesh_rpy_offset = Vector3.Zero;
	
	public Transform3D InitializeMeshTransform() {
		mesh_T_world.Origin = mesh_pos_offset;
		Vector3 mesh_ZYX_offset = new Vector3(mesh_rpy_offset.Z,
									  mesh_rpy_offset.Y,
									  mesh_rpy_offset.X);
		mesh_T_world = mesh_T_world.Rotated(new Vector3(0, 0, 1), mesh_rpy_offset.Z);
		mesh_T_world = mesh_T_world.Rotated(new Vector3(0, 1, 0), mesh_rpy_offset.Y);
		mesh_T_world = mesh_T_world.Rotated(new Vector3(1, 0, 0), mesh_rpy_offset.X);
		mesh_T_world.Basis = mesh_T_world.Basis * R_mesh;
		ConvertWorldTransformToGodot(ref mesh_T_godot, in mesh_T_world);

		// TODO: This code  below works for the humanoid, without the converstion to the Godot frame
		// TODO: This is a bug that needs to be fixed for the humanoid specifically.
		// mesh_T_world.Basis = R_mesh * Basis.FromEuler(mesh_ZYX_offset, EulerOrder.Zyx);
		return mesh_T_godot;
	}

	public Transform3D InitializeLinkTransform() {
		link_T_world.Origin = Vector3.Zero;
		link_T_world.Basis = I;
        ConvertWorldTransformToGodot(ref link_T_godot, in link_T_world);
		return link_T_godot;
	}
	public Transform3D GetLinkTransform() {
		// Rotation
		world_R_parent = R_world * parent_node.Transform.Basis;
		parent_R_offset = Basis.FromEuler(joint_ZYX_rel_parent, EulerOrder.Zyx);
		offset_R_joint = new(joint_axis, q); // This should depend on the joint type
		joint_R_link = Basis.FromEuler(link_ZYX_rel_joint, EulerOrder.Zyx);
		Basis world_R_link = world_R_parent * parent_R_offset * offset_R_joint * joint_R_link;
		
		// Translation
		world_p_parent = R_world * parent_node.Transform.Origin;	// w_P_p/w
		world_p_joint = world_p_parent + world_R_parent * joint_pos_rel_parent; // w_P_j/w = w_P_j/p + w_P_p/w
		world_p_link = world_p_joint + world_R_parent * parent_R_offset * offset_R_joint * link_pos_rel_joint; // w_P_l/j + w_P_j/w

		link_T_world.Basis = world_R_link;
		link_T_world.Origin = world_p_link;
        ConvertWorldTransformToGodot(ref link_T_godot, in link_T_world);
		return link_T_godot;
	}

	public Transform3D GetBaseTransform(Transform3D T_base) {
        ConvertWorldTransformToGodot(ref link_T_godot, in T_base);
		return link_T_godot;
	}

	public Transform3D GetBaseTransform(Basis R,
										Vector3 p_base) {
		link_T_world.Basis = R;
		link_T_world.Origin = p_base;
        ConvertWorldTransformToGodot(ref link_T_godot, in link_T_world);
		return link_T_godot;
	}

	public Transform3D GetBaseTransform(Vector3 axis, float angle,
										Vector3 p_base) {
		link_T_world.Basis = new(axis, angle);
		link_T_world.Origin = p_base;
        ConvertWorldTransformToGodot(ref link_T_godot, in link_T_world);
		return link_T_godot;
	}

	public Transform3D GetBaseTransform(Vector3 rpy,
										Vector3 p_base) {
		Vector3 euler_ZYX = new(rpy.Z, rpy.Y, rpy.X);
		link_T_world.Basis = Basis.FromEuler(euler_ZYX, EulerOrder.Zyx);
		link_T_world.Origin = p_base;
        ConvertWorldTransformToGodot(ref link_T_godot, in link_T_world);
		return link_T_godot;
	}

	public Transform3D GetBaseTransform(Quaternion q_base,
										Vector3 p_base) {
		link_T_world.Basis = new(q_base);
		link_T_world.Origin = p_base;
        ConvertWorldTransformToGodot(ref link_T_godot, in link_T_world);
		return link_T_godot;
	}

	public static void ConvertWorldTransformToGodot(ref Transform3D T_godot,
													in Transform3D T_world) {
		T_godot.Origin = R_godot * T_world.Origin;
		T_godot.Basis = R_godot * T_world.Basis;
	}

	// Called when the node enters the scene tree for the first time.
	public override void _Ready() {
	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta) {
	}
}
