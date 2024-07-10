using Godot;
using System;

public partial class Left_Shoulder_2 : Link {
	private void SetupParameters() {
		is_root = false;
		link_name = "Left_Shoulder_2";
		parent_name = "Left_Shoulder_1";
		joint_axis = new Vector3(-1.0f, 0.0f, 0.0f);
		joint_pos_rel_parent = new Vector3(0.0f, 0.05760f, 0.0f);
		joint_rpy_rel_parent = Vector3.Zero;
		joint_ZYX_rel_parent = new Vector3(joint_rpy_rel_parent.Z,
										   joint_rpy_rel_parent.Y,
										   joint_rpy_rel_parent.X);
		link_pos_rel_joint = Vector3.Zero;
		link_rpy_rel_joint = Vector3.Zero;
		link_ZYX_rel_joint = new Vector3(link_rpy_rel_joint.Z,
										 link_rpy_rel_joint.Y,
										 link_rpy_rel_joint.X);
		mesh_pos_offset = Vector3.Zero;
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
