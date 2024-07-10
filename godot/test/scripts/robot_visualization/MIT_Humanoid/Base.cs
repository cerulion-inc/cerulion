using Godot;

public partial class Base : Link
{
    private void SetupParameters() {
		is_root = true;
		link_name = "Base";
		parent_name = "";
		joint_axis = new Vector3(0.0f, 0.0f, 0.0f);
		joint_pos_rel_parent = new Vector3(0.0f, 0.0f, 0.0f);
		joint_rpy_rel_parent = new Vector3(0.0f, 0.0f, 0.0f);
		joint_ZYX_rel_parent = new Vector3(0.0f, 0.0f, 0.0f);
		link_pos_rel_joint = new Vector3(0.0f, 0.0f, 0.0f);
		link_rpy_rel_joint = new Vector3(0.0f, 0.0f, 0.0f);
		link_ZYX_rel_joint = new Vector3(0.0f, 0.0f, 0.0f);
		mesh_pos_offset = new Vector3(-0.00565f, 0.0f, -0.05735f);
		mesh_rpy_offset = new Vector3(0.0f, 0.0f, 0.0f);
	}

	// Called when the node enters the scene tree for the first time.
	public override void _Ready() {
		SetupParameters();
		mesh_instance = GetNode<MeshInstance3D>(link_name);
		mesh_instance.Transform = InitializeMeshTransform();
		Transform = InitializeLinkTransform();
		parent_node = this;
		GD.Print("Base initialized.");
	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta) {
		Transform = GetBaseTransform(link_T_world);
	}
}
