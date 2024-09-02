using Godot;
using System;
using LCM;

using JointDict = Godot.Collections.Dictionary<string, float>;
using NodeDict = Godot.Collections.Dictionary<string, Godot.Node3D>;

public partial class Subscriber : Control
{
	private LCM.LCM.LCM lcm_handler = null;
	float time = 0;
	private Node robot_state = null;
	
	// Called when the node enters the scene tree for the first time.
	public override void _Ready() {
		robot_state = GetNode("/root/RobotState");
		lcm_handler = new LCM.LCM.LCM();
		lcm_handler.SubscribeAll(new GodotSubscriber());
	}

// On signal received that the robot is loaded, set instances
// All other logic should be inside LCM subscriber


	public override void _PhysicsProcess(double delta) {
		//GD.Print("test");
		JointDict joints = robot_state.Get("joints").As<JointDict>();
		//joints[]
	}
	
	internal class GodotSubscriber : LCM.LCM.LCMSubscriber
  	{
			public void MessageReceived(LCM.LCM.LCM lcm, string channel, LCM.LCM.LCMDataInputStream dins)
			{
	  			if (channel == "JOINT_STATES") {
					unitree_lcm.joint_states_t msg = new unitree_lcm.joint_states_t(dins);
	  			} else if (channel == "") {}
			}
  	}
}


	//// Called every frame. 'delta' is the elapsed time since the previous frame.
	//public override void _Process(double delta)
	//{
		////Vector3 posn = new(0, MathF.Sin(time), 0);
		////GD.Print(robot_state.Get("link_nodes"));
		////NodeDict test2 = 
			////robot_state.Get("link_nodes").As<NodeDict>();
		////Transform3D T_test;
		////T_test.Basis = Basis.Identity;
		////T_test.Origin = new(0, MathF.Sin(time), 0);
		////test2["FL_thigh"].Transform = T_test;
		////time += (float) delta;
		////float state = (float) MathF.Sin(time);
		////GD.Print(state);
		////robotStateNode.Set("link_transforms[\"FL_thigh\"].q", state);
		//
		//
		////GD.Print(robotStateNode.Get("joints"));
		////robotStateNode.Set("joints["FL_hip""]")
	//}
