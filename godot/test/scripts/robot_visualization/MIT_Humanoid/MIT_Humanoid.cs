using Godot;
using System;
using System.Collections.Generic;

public partial class MIT_Humanoid : Node3D
{
	// There will be some layer to receive and parse ROS messages
	// We should similarly assume our own backend will format messages the same way

	float time = 0.0f;

	// Coordinates of base in world frame
	public Quaternion q_base = new(0, 0, 0, 1);
	public Vector3 p_base = new(0, 0, 0);

	// Generalized coordinates
	private readonly Dictionary<string, int> _link_idx = new() {
		{"Base", 0},
		{"Right_Hip_Yaw", 1}, {"Right_Hip_Abad", 2}, {"Right_Upper_Leg", 3}, {"Right_Lower_Leg", 4}, {"Right_Foot", 5},
		{"Left_Hip_Yaw", 6}, {"Left_Hip_Abad", 7}, {"Left_Upper_Leg", 8}, {"Left_Lower_Leg", 9}, {"Left_Foot", 10},
		{"Right_Shoulder_1", 11}, {"Right_Shoulder_2", 12}, {"Right_Shoulder_3", 13}, {"Right_Forearm", 14},
		{"Left_Shoulder_1", 15}, {"Left_Shoulder_2", 16}, {"Left_Shoulder_3", 17}, {"Left_Forearm", 18}
	};
	private readonly Dictionary<string, float> _q_joints = new() {
		{"Right_Hip_Yaw", 0.0f}, {"Right_Hip_Abad", 0.0f}, {"Right_Upper_Leg", 0.0f}, {"Right_Lower_Leg", 0.0f}, {"Right_Foot", 0.0f},
		{"Left_Hip_Yaw", 0.0f}, {"Left_Hip_Abad", 0.0f}, {"Left_Upper_Leg", 0.0f}, {"Left_Lower_Leg", 0.0f}, {"Left_Foot", 0.0f},
		{"Right_Shoulder_1", 0.0f}, {"Right_Shoulder_2", 0.0f}, {"Right_Shoulder_3", 0.0f}, {"Right_Forearm", 0.0f},
		{"Left_Shoulder_1", 0.0f}, {"Left_Shoulder_2", 0.0f}, {"Left_Shoulder_3", 0.0f}, {"Left_Forearm", 0.0f}
	};
	
	// Child links
	Node3D link_container;
	private Godot.Collections.Array<Link> links = new();

	// Update information from ros-like message here, from backend (directly from robot/sim)
	public void UpdateSystemState(Array q_ROS) {
	}

	public void UpdateEnvironmentState(double delta) {
		time += (float) delta;
	}

	// Called when the node enters the scene tree for the first time.
	public override void _Ready() {
		link_container = GetNode<Node3D>("Link");
		for (int i = 0; i < _link_idx.Count; i++) {
            links.Add(link_container.GetChild<Link>(i));
		}
	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta) {
		// Temporary random movement for visualization
		p_base.X = 0.5f * MathF.Sin(time);
		p_base.Y = 0.5f * MathF.Cos(time);
		p_base.Z = 0.75f;
		q_base = new(MathF.Sin(time), MathF.Sin(time), MathF.Cos(time), MathF.Cos(time));
		q_base = q_base.Normalized();

		// UpdateSystemState(); // Update q_base, p_base, q_joints from backend
		links[_link_idx["Base"]].link_T_world.Basis = new(q_base);
		links[_link_idx["Base"]].link_T_world.Origin = p_base;
		foreach (KeyValuePair<string, float> kvp in _q_joints) {
			_q_joints[kvp.Key] = MathF.Sin(time);
			links[_link_idx[kvp.Key]].q = kvp.Value;
		}
		UpdateEnvironmentState(delta);
	}
}
