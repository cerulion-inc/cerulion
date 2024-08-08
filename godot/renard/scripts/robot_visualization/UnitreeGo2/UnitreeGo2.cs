using Godot;
using LCM;
using System;
using System.Threading;
using System.Collections;
using System.Collections.Generic;

// TODO: Efficient threading and async parallelization of LCM message subscriptions

public partial class UnitreeGo2 : Node3D
{
	// There will be some layer to receive and parse ROS messages
	// We should similarly assume our own backend will format messages the same way
	bool running = false;
    private LCM.LCM.LCM unitree_LCM = null;

	float time = 0.0f;

	// Coordinates of base in world frame
	public Quaternion q_base = new(0, 0, 0, 1);
	public Vector3 p_base = new(0, 0, 0);

	// Generalized coordinates
	private readonly Dictionary<string, int> _link_idx = new() {
		{"Base", 0},
		{"FL_Hip", 1}, {"FL_thigh", 2}, {"FL_calf", 3},
		{"FR_Hip", 4}, {"FR_thigh", 5}, {"FR_calf", 6},
		{"RL_Hip", 7}, {"RL_thigh", 8}, {"RL_calf", 9},
		{"RR_Hip", 10}, {"RR_thigh", 11}, {"RR_calf", 12},
	};
	private readonly Dictionary<string, float> _q_joints = new() {
		{"FL_Hip", 0.0f}, {"FL_thigh", 0.0f}, {"FL_calf", 0.0f},
		{"FR_Hip", 0.0f}, {"FR_thigh", 0.0f}, {"FR_calf", 0.0f},
		{"RL_Hip", 0.0f}, {"RL_thigh", 0.0f}, {"RL_calf", 0.0f},
		{"RR_Hip", 0.0f}, {"RR_thigh", 0.0f}, {"RR_calf", 0.0f},
	};
	
	// Child links
	Node3D link_container;
	Link link_instance;
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
			link_instance = link_container.GetChild<Link>(i);
            links.Add(link_instance);
		}
    unitree_LCM = new LCM.LCM.LCM();
    unitree_LCM.SubscribeAll(new SimpleSubscriber());
	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta) {
		// Temporary random movement for visualization
		// p_base.X = 0.5f * MathF.Sin(time);
		// p_base.X = 0.5f * (time);
		// p_base.Y = 0.5f * MathF.Cos(time);
		p_base.Z = 0.75f;
		// q_base = new(MathF.Sin(time), MathF.Sin(time), MathF.Cos(time), MathF.Cos(time));
		// q_base = q_base.Normalized();

		// UpdateSystemState(); // Update q_base, p_base, q_joints from backend
		links[_link_idx["Base"]].link_T_world.Basis = new(q_base);
		links[_link_idx["Base"]].link_T_world.Origin = p_base;
		foreach (KeyValuePair<string, float> kvp in _q_joints) {
			_q_joints[kvp.Key] = MathF.Sin(time);
			links[_link_idx[kvp.Key]].q = kvp.Value;
		}
		UpdateEnvironmentState(delta);
	}


  internal class SimpleSubscriber : LCM.LCM.LCMSubscriber
  {
	int iter = 0;
    public void MessageReceived(LCM.LCM.LCM lcm, string channel, LCM.LCM.LCMDataInputStream dins)
    {
      if (channel == "TEST_CHANNEL")
      {
		GD.Print("LCM MESSGE RECEIVED!!!!!!");
		GD.Print(iter);
		iter += 1;
      }
    }
  }


}

//   protected override void Start()
//   {
//     base.Start();
//     StartCoroutine("Listener");
//   }

//   // Update is called once per frame
//   void Update()
//   {
//     updateState(body_pos, body_ori_quat, jpos);
//   }

//   public IEnumerator Listener()
//   {
//     unitree_LCM = new LCM.LCM.LCM();

//     unitree_LCM.SubscribeAll(new SimpleSubscriber());
//     // may want to subscribe only to one channel: unitree_LCM.Subscribe("quadruped_visualization_info", new SimpleSubscriber());
//     running = true;
//     while (running){
//       yield return null;
//     }
//   }