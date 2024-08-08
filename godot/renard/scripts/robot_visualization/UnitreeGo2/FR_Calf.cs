using Godot;
using System;

namespace UnitreeGo2_Links{
public partial class FR_Calf : Link {
	private void SetupParameters() {
		is_root = false;
		link_name = "FR_Calf";
		parent_name = "FR_Thigh";
		joint_axis = new Vector3(0.0f, 1.0f, 0.0f);
		joint_pos_rel_parent = new Vector3(0, 0, -0.213f);
		parent_node = GetNode<Link>("../" + parent_name);
		R_mesh = new Basis(new Vector3(0, 1, 0),
						   new Vector3(1, 0, 0),
						   new Vector3(0, 0, -1));
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

}