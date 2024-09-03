using Godot;
using System;
using LCM;

using JointDict = Godot.Collections.Dictionary<string, Godot.Variant>;

public partial class Subscriber : Control
{
	private double time = 0;
	private LCM.LCM.LCM lcm_handler = null;
	private Node signals = null;
	private Node robot_state = null;
	private static JointDict joint_dict = null;
	private bool robot_loaded = false;
	
	// Called when the node enters the scene tree for the first time.
	public override void _Ready() {
		signals = GetNode("/root/Signals");
		robot_state = GetNode("/root/RobotState");
		lcm_handler = new LCM.LCM.LCM();
		lcm_handler.SubscribeAll(new GodotSubscriber());
		signals.Connect("RobotLoaded", Callable.From(SetupSubscriber));
	}

	// ! TODO: THIS SHOULD HAVE IS_FLOATING CHECK
	private void SetupSubscriber() {
		joint_dict = robot_state.Get("joints").As<JointDict>();
		joint_dict["base"] = (joint_dict["base"]).As<float []>();
		robot_loaded = true;
	}

	// THIS FUNCTION IS ONLY TO VALIDATE THAT MODIFYING "joint_dict" IN THIS SCRIPT
	// DIRECTLY MOVES THE ROBOT IN "RobotInstance.gd". 
	// THE CODE IN THE LCM FUNCTION BELOW SHOULD WORK AS IS
	public override void _PhysicsProcess(double delta) {
		if (robot_loaded) {
			float [] test = {0, 0, 0, 1, (float)Mathf.Sin(time), (float)Mathf.Cos(time), 1};
			joint_dict["base"] = test;
			joint_dict["FL_hip"] = (float) Mathf.Sin(time);
			joint_dict["FL_thigh"] = Mathf.Sin(time);
			time += delta;
			//GD.Print(joint_dict);
		}
	}
	
	internal class GodotSubscriber : LCM.LCM.LCMSubscriber {
		public void MessageReceived(LCM.LCM.LCM lcm, string channel, LCM.LCM.LCMDataInputStream dins) {
			if (channel == "JOINT_STATES") {
				GD.Print("Joint state message received from LCM.");
				unitree_lcm.joint_states_t msg = new unitree_lcm.joint_states_t(dins);
				//! TODO: MODIFY THIS FOR MSG FORMAT FOR FLOATING BASE
				//! TODO: PACK QUATERNION AS X Y Z W
				
				// float[] q_base = new float[5];
				// q_base[0] = msg.quat[0];
				// q_base[1] = msg.quat[1];
				// ...
				// q_base[4] = msg.pos[0];
				// q_base[5] = msg.pos[1];
				// ...
				// joint_dict["base"] = q_base;
				
				joint_dict["FL_hip"] = (float) msg.q_front_left[0];
				joint_dict["FL_thigh"] = (float) msg.q_front_left[1];
				joint_dict["FL_calf"] = (float) msg.q_front_left[2];
				
				joint_dict["FR_hip"] = (float) msg.q_front_right[0];
				joint_dict["FR_thigh"] = (float) msg.q_front_right[1];
				joint_dict["FR_calf"] = (float) msg.q_front_right[2];
				
				joint_dict["RL_hip"] = (float) msg.q_rear_left[0];
				joint_dict["RL_thigh"] = (float) msg.q_rear_left[1];
				joint_dict["RL_calf"] = (float) msg.q_rear_left[2];
				
				joint_dict["RR_hip"] = (float) msg.q_rear_right[0];
				joint_dict["RR_thigh"] = (float) msg.q_rear_right[1];
				joint_dict["RR_calf"] = (float) msg.q_rear_right[2];
			}
		}
	}
}
