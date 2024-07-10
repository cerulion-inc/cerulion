using Godot;
using System;

public partial class Right_Upper_Leg : Link {
	private void SetupParameters() {
		is_root = false;
		link_name = "Right_Upper_Leg";
		parent_name = "Right_Hip_Abad";
		joint_axis = new Vector3(0.0f, 1.0f, 0.0f);
		joint_pos_rel_parent = new Vector3(0.07133f, 0.002f, 0.0f);
		joint_rpy_rel_parent = new Vector3(0.0f, -0.261799f, 0.0f);
		joint_ZYX_rel_parent = new Vector3(joint_rpy_rel_parent.Z,
										   joint_rpy_rel_parent.Y,
										   joint_rpy_rel_parent.X);
		link_pos_rel_joint = Vector3.Zero;
		link_rpy_rel_joint = Vector3.Zero;
		link_ZYX_rel_joint = new Vector3(link_rpy_rel_joint.Z,
										 link_rpy_rel_joint.Y,
										 link_rpy_rel_joint.X);
		mesh_pos_offset = new Vector3(0.01283f, 0.0f, -0.0174f);
		mesh_rpy_offset = Vector3.Zero;
		parent_node = GetNode<Link>("../" + parent_name);
	}

	// Called when the node enters the scene tree for the first time.
	public override void _Ready() {
		SetupParameters();
		mesh_instance = GetNode<MeshInstance3D>(link_name);
		mesh_instance.Transform = InitializeMeshTransform();
		Transform = InitializeLinkTransform();
	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta) {
		Transform = GetLinkTransform();
	}
}
