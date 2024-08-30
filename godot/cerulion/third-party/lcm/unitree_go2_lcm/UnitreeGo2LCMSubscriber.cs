using Godot;
using System;
using LCM;

public partial class UnitreeGo2LCMSubscriber : Node3D
{
	
	private LCM.LCM.LCM myLCM = null;
	//float time = 0;
	
	private GDScript RobotState = null;
	private static GodotObject robotStateNode = null;
	
	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
		myLCM = LCM.LCM.LCM.Singleton;
		myLCM.SubscribeAll(new SimpleSubscriber());
		
		RobotState = GD.Load<GDScript>("res://scenes/globals/RobotState.gd");
		robotStateNode = (GodotObject)RobotState.New();
	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta)
	{
		//time += (float) delta;
		//float state = (float) MathF.Sin(time);
		//GD.Print(state);
		//robotStateNode.Set("link_transforms[\"FL_thigh\"].q", state);
		
		
		//GD.Print(robotStateNode.Get("joints"));
		//robotStateNode.Set("joints["FL_hip""]")
	}
	
	internal class SimpleSubscriber : LCM.LCM.LCMSubscriber
  	{
			public void MessageReceived(LCM.LCM.LCM lcm, string channel, LCM.LCM.LCMDataInputStream dins)
			{
	  			if (channel == "JOINT_STATES")
	  			{
					unitree_lcm.joint_states_t msg = new unitree_lcm.joint_states_t(dins);
					
					robotStateNode.Set("link_transforms[\"FL_hip\"].q", msg.q_front_left[0]);
					robotStateNode.Set("link_transforms[\"FL_thigh\"].q", msg.q_front_left[1]);
					robotStateNode.Set("link_transforms[\"FL_calf\"].q", msg.q_front_left[2]);
					
					robotStateNode.Set("link_transforms[\"FR_hip\"].q", msg.q_front_right[0]);
					robotStateNode.Set("link_transforms[\"FR_thigh\"].q", msg.q_front_right[1]);
					robotStateNode.Set("link_transforms[\"FR_calf\"].q", msg.q_front_right[2]);
					
					robotStateNode.Set("link_transforms[\"RL_hip\"].q", msg.q_rear_left[0]);
					robotStateNode.Set("link_transforms[\"RL_thigh\"].q", msg.q_rear_left[1]);
					robotStateNode.Set("link_transforms[\"RL_calf\"].q", msg.q_rear_left[2]);
					
					robotStateNode.Set("link_transforms[\"RR_hip\"].q", msg.q_rear_right[0]);
					robotStateNode.Set("link_transforms[\"RR_thigh\"].q", msg.q_rear_right[1]);
					robotStateNode.Set("link_transforms[\"RR_calf\"].q", msg.q_rear_right[2]);
	  			}
			}
  	}
}
