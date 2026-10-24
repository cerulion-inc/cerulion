//! The simulated arm: a pure, deterministic function from a tool call to the
//! next plant state.
//!
//! Six revolute joints (`JOINT_NAMES`), a parallel gripper normalized to
//! 0 (open) ..= 1 (closed), and one cube on a table. One `arm.step` call
//! advances the plant exactly one control period of [`DT_S`] seconds. Every
//! quantity is an `f64` computed from the call sequence alone: no clock, no
//! randomness beyond the seeded [`splitmix64`], no iteration over hashed
//! containers. Replaying the same calls yields the same bytes.

use cerulion_core::state::CerulionState;
use serde::Serialize;
use serde_json::Value;

/// Control period, seconds (20 Hz).
pub const DT_S: f64 = 0.05;
/// Joint names, base to tool.
pub const JOINT_NAMES: [&str; 6] = [
    "base_yaw",
    "shoulder_pitch",
    "elbow_pitch",
    "wrist_pitch",
    "wrist_roll",
    "tool_roll",
];
/// Symmetric joint limit, radians.
pub const JOINT_LIMIT_RAD: f64 = 3.0;
/// Largest joint change in one period, radians (1.0 rad/s).
pub const MAX_JOINT_STEP_RAD: f64 = 0.05;
/// Largest gripper change in one period, normalized units.
pub const MAX_GRIPPER_STEP: f64 = 0.25;
/// Shoulder height above the table, metres.
pub const SHOULDER_HEIGHT_M: f64 = 0.20;
/// Upper arm length, metres.
pub const L1_M: f64 = 0.30;
/// Forearm length, metres.
pub const L2_M: f64 = 0.25;
/// Wrist to tool tip, metres.
pub const L3_M: f64 = 0.10;
/// Cube half edge, metres; a cube resting on the table has its centre here.
pub const CUBE_HALF_M: f64 = 0.025;
/// The tool tip must be this close to the cube centre to grasp it.
pub const GRASP_RADIUS_M: f64 = 0.03;
/// A gripper at or above this closure grasps.
pub const GRIPPER_CLOSED: f64 = 0.9;
/// A gripper below this closure releases.
pub const GRIPPER_RELEASE: f64 = 0.5;
/// The task succeeds while the cube is held with its centre at or above this.
pub const LIFT_HEIGHT_M: f64 = 0.15;
/// The arm's configuration after `arm.reset`: tool tip near (0.406, 0, 0.215),
/// pointing down.
pub const HOME_Q: [f64; 6] = [0.0, 0.9, -1.4, -1.07, 0.0, 0.0];

/// The plant state a result carries.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct ArmState {
    /// Control periods executed since the last reset.
    pub step: u64,
    /// Control period, seconds.
    pub dt_s: f64,
    /// Joint positions, radians.
    pub q: [f64; 6],
    /// Joint velocities over the last period, radians per second.
    pub qd: [f64; 6],
    /// Gripper closure, 0 open ..= 1 closed.
    pub gripper: f64,
    /// Tool tip position in the base frame, metres.
    pub ee_position: [f64; 3],
    /// Tool orientation in the base frame, unit quaternion (w, x, y, z).
    pub ee_quaternion_wxyz: [f64; 4],
    /// Cube centre in the base frame, metres.
    pub cube_position: [f64; 3],
    /// Whether the gripper holds the cube.
    pub grasped: bool,
    /// Whether the emergency stop is latched.
    pub estopped: bool,
    /// Whether the cube is held at or above [`LIFT_HEIGHT_M`].
    pub task_success: bool,
}

/// A refusal carried in a result.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct CallError {
    /// A stable machine-readable code.
    pub code: &'static str,
    /// What was wrong.
    pub message: String,
}

/// The `/rcp/results` payload for one call.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct CallResult {
    /// The `call_id` of the call this answers; 0 when the call had none.
    pub call_id: u64,
    /// The tool the call named.
    pub tool: String,
    /// Whether the call took effect.
    pub ok: bool,
    /// Why it did not, when `ok` is false.
    pub error: Option<CallError>,
    /// The plant state after the call.
    pub state: ArmState,
}

/// SplitMix64: the seeded generator `arm.reset` places the cube with.
pub fn splitmix64(state: &mut u64) -> u64 {
    *state = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
    let mut z = *state;
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

fn unit(state: &mut u64) -> f64 {
    // The top 53 bits as a float in [0, 1).
    (splitmix64(state) >> 11) as f64 / (1u64 << 53) as f64
}

/// Tool tip position and orientation for joint configuration `q`.
pub fn forward_kinematics(q: &[f64; 6]) -> ([f64; 3], [f64; 4]) {
    let a1 = q[1];
    let a2 = q[1] + q[2];
    let a3 = q[1] + q[2] + q[3];
    let r = L1_M * a1.cos() + L2_M * a2.cos() + L3_M * a3.cos();
    let z = SHOULDER_HEIGHT_M + L1_M * a1.sin() + L2_M * a2.sin() + L3_M * a3.sin();
    let position = [r * q[0].cos(), r * q[0].sin(), z];
    // R = Rz(yaw) * Ry(-a3) * Rx(roll), composed as quaternions.
    let yaw = quat_axis(2, q[0]);
    let pitch = quat_axis(1, -a3);
    let roll = quat_axis(0, q[4] + q[5]);
    (position, quat_mul(quat_mul(yaw, pitch), roll))
}

fn quat_axis(axis: usize, angle: f64) -> [f64; 4] {
    let half = angle * 0.5;
    let mut out = [half.cos(), 0.0, 0.0, 0.0];
    out[axis + 1] = half.sin();
    out
}

fn quat_mul(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    [
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ]
}

fn distance(a: &[f64; 3], b: &[f64; 3]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

/// The arm, gripper and cube. `CerulionState` lets a Flashback capture or a
/// resim restart save and restore it with the node.
#[derive(Debug, Clone, CerulionState)]
pub struct Plant {
    step: u64,
    q: [f64; 6],
    qd: [f64; 6],
    gripper: f64,
    cube: [f64; 3],
    /// Cube centre minus tool tip, fixed at grasp time; `Some` while held.
    grasp_offset: Option<[f64; 3]>,
    estopped: bool,
}

impl Default for Plant {
    fn default() -> Self {
        Self::reset(0)
    }
}

impl Plant {
    /// The plant after `arm.reset { seed }`.
    pub fn reset(seed: u64) -> Self {
        let mut rng = seed;
        let x = 0.30 + 0.12 * unit(&mut rng);
        let y = -0.12 + 0.24 * unit(&mut rng);
        Self {
            step: 0,
            q: HOME_Q,
            qd: [0.0; 6],
            gripper: 0.0,
            cube: [x, y, CUBE_HALF_M],
            grasp_offset: None,
            estopped: false,
        }
    }

    /// The state a result reports.
    pub fn state(&self) -> ArmState {
        let (ee_position, ee_quaternion_wxyz) = forward_kinematics(&self.q);
        let grasped = self.grasp_offset.is_some();
        ArmState {
            step: self.step,
            dt_s: DT_S,
            q: self.q,
            qd: self.qd,
            gripper: self.gripper,
            ee_position,
            ee_quaternion_wxyz,
            cube_position: self.cube,
            grasped,
            estopped: self.estopped,
            task_success: grasped && self.cube[2] >= LIFT_HEIGHT_M,
        }
    }

    /// Advance one control period toward `q_target` and `gripper`.
    ///
    /// # Errors
    ///
    /// `estopped` while the stop is latched; `joint_limit` when a target lies
    /// outside [`JOINT_LIMIT_RAD`]; `invalid_args` for a non-finite value or a
    /// gripper outside 0..=1. A refused step leaves the plant unchanged.
    pub fn step(&mut self, q_target: &[f64; 6], gripper: f64) -> Result<(), CallError> {
        if self.estopped {
            return Err(CallError {
                code: "estopped",
                message: String::from("the emergency stop is latched; arm.reset clears it"),
            });
        }
        if q_target.iter().any(|v| !v.is_finite()) || !gripper.is_finite() {
            return Err(invalid("q_target and gripper must be finite"));
        }
        if !(0.0..=1.0).contains(&gripper) {
            return Err(invalid("gripper must lie in 0..=1"));
        }
        if let Some(i) = q_target.iter().position(|v| v.abs() > JOINT_LIMIT_RAD) {
            return Err(CallError {
                code: "joint_limit",
                message: format!(
                    "{} target {} lies outside +-{JOINT_LIMIT_RAD} rad",
                    JOINT_NAMES[i], q_target[i]
                ),
            });
        }
        for (i, target) in q_target.iter().enumerate() {
            let delta = (target - self.q[i]).clamp(-MAX_JOINT_STEP_RAD, MAX_JOINT_STEP_RAD);
            self.q[i] += delta;
            self.qd[i] = delta / DT_S;
        }
        self.gripper += (gripper - self.gripper).clamp(-MAX_GRIPPER_STEP, MAX_GRIPPER_STEP);
        self.step += 1;

        let (tip, _) = forward_kinematics(&self.q);
        match self.grasp_offset {
            Some(offset) if self.gripper < GRIPPER_RELEASE => {
                // The cube falls straight down from where the tool now holds it.
                self.grasp_offset = None;
                self.cube = [tip[0] + offset[0], tip[1] + offset[1], CUBE_HALF_M];
            }
            Some(offset) => {
                self.cube = [tip[0] + offset[0], tip[1] + offset[1], tip[2] + offset[2]];
            }
            None if self.gripper >= GRIPPER_CLOSED
                && distance(&tip, &self.cube) <= GRASP_RADIUS_M =>
            {
                self.grasp_offset = Some([
                    self.cube[0] - tip[0],
                    self.cube[1] - tip[1],
                    self.cube[2] - tip[2],
                ]);
            }
            None => {}
        }
        Ok(())
    }

    /// Latch the emergency stop and zero the reported velocities.
    pub fn estop(&mut self) {
        self.estopped = true;
        self.qd = [0.0; 6];
    }

    /// Apply one `/rcp/calls` payload and return the `/rcp/results` payload.
    pub fn apply(&mut self, call_json: &str) -> CallResult {
        let parsed: Result<Value, _> = serde_json::from_str(call_json);
        let call = match parsed {
            Ok(Value::Object(map)) => map,
            Ok(_) | Err(_) => {
                return self.answer(0, "", Err(invalid("the call is not a JSON object")));
            }
        };
        let call_id = call.get("call_id").and_then(Value::as_u64).unwrap_or(0);
        let tool = call
            .get("tool")
            .and_then(Value::as_str)
            .unwrap_or("")
            .to_string();
        let empty = Value::Object(serde_json::Map::new());
        let args = call.get("args").unwrap_or(&empty);
        let outcome = match tool.as_str() {
            "arm.reset" => match parse_seed(args) {
                Some(seed) => {
                    *self = Self::reset(seed);
                    Ok(())
                }
                None => Err(invalid("arm.reset needs a non-negative integer seed")),
            },
            "arm.step" => match parse_step(args) {
                Ok((q_target, gripper)) => self.step(&q_target, gripper),
                Err(e) => Err(e),
            },
            "estop" => {
                self.estop();
                Ok(())
            }
            other => Err(CallError {
                code: "unknown_tool",
                message: format!("the plant has no tool {other:?}"),
            }),
        };
        self.answer(call_id, &tool, outcome)
    }

    fn answer(&self, call_id: u64, tool: &str, outcome: Result<(), CallError>) -> CallResult {
        let (ok, error) = match outcome {
            Ok(()) => (true, None),
            Err(e) => (false, Some(e)),
        };
        CallResult {
            call_id,
            tool: tool.to_string(),
            ok,
            error,
            state: self.state(),
        }
    }
}

fn invalid(message: &str) -> CallError {
    CallError {
        code: "invalid_args",
        message: message.to_string(),
    }
}

/// A non-negative integer seed; `3.0` counts, as JSON Schema's `integer` does.
fn parse_seed(args: &Value) -> Option<u64> {
    let seed = args.get("seed")?;
    seed.as_u64().or_else(|| {
        seed.as_f64()
            .filter(|v| *v >= 0.0 && v.fract() == 0.0 && *v < 9.007_199_254_740_992e15)
            .map(|v| v as u64)
    })
}

fn parse_step(args: &Value) -> Result<([f64; 6], f64), CallError> {
    let values = args
        .get("q_target")
        .and_then(Value::as_array)
        .ok_or_else(|| invalid("arm.step needs q_target, an array of 6 numbers"))?;
    if values.len() != 6 {
        return Err(invalid("q_target must hold exactly 6 numbers"));
    }
    let mut q_target = [0.0; 6];
    for (slot, value) in q_target.iter_mut().zip(values) {
        *slot = value
            .as_f64()
            .ok_or_else(|| invalid("q_target must hold numbers"))?;
    }
    let gripper = args
        .get("gripper")
        .and_then(Value::as_f64)
        .ok_or_else(|| invalid("arm.step needs gripper, a number in 0..=1"))?;
    Ok((q_target, gripper))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn step_until(plant: &mut Plant, q: [f64; 6], gripper: f64) {
        for _ in 0..400 {
            plant.step(&q, gripper).expect("step");
            let s = plant.state();
            let done = s.q.iter().zip(q.iter()).all(|(a, b)| (a - b).abs() < 1e-12)
                && (s.gripper - gripper).abs() < 1e-12;
            if done {
                return;
            }
        }
        panic!("did not converge");
    }

    /// Joint angles that put the tool tip at `p` pointing straight down.
    fn ik_down(p: [f64; 3]) -> [f64; 6] {
        let yaw = p[1].atan2(p[0]);
        let r = (p[0] * p[0] + p[1] * p[1]).sqrt();
        let (wr, wz) = (r, p[2] + L3_M - SHOULDER_HEIGHT_M);
        let c2 = (wr * wr + wz * wz - L1_M * L1_M - L2_M * L2_M) / (2.0 * L1_M * L2_M);
        let a2 = -c2.clamp(-1.0, 1.0).acos();
        let a1 = wz.atan2(wr) - (L2_M * a2.sin()).atan2(L1_M + L2_M * a2.cos());
        let a3 = -std::f64::consts::FRAC_PI_2 - a1 - a2;
        [yaw, a1, a2, a3, 0.0, 0.0]
    }

    #[test]
    fn home_tip_points_down_near_the_documented_spot() {
        let (p, quat) = forward_kinematics(&HOME_Q);
        assert!((p[0] - 0.406).abs() < 0.005 && p[1].abs() < 1e-12 && (p[2] - 0.215).abs() < 0.005);
        let norm: f64 = quat.iter().map(|v| v * v).sum();
        assert!((norm - 1.0).abs() < 1e-12);
    }

    #[test]
    fn reset_is_a_pure_function_of_the_seed() {
        let a = Plant::reset(7).state();
        let b = Plant::reset(7).state();
        assert_eq!(a, b);
        assert_ne!(a.cube_position, Plant::reset(8).state().cube_position);
        assert!((0.30..0.42).contains(&a.cube_position[0]));
        assert!((-0.12..0.12).contains(&a.cube_position[1]));
    }

    #[test]
    fn pick_and_lift_succeeds_and_release_drops_the_cube() {
        let mut plant = Plant::reset(3);
        let cube = plant.state().cube_position;
        step_until(&mut plant, ik_down([cube[0], cube[1], cube[2] + 0.08]), 0.0);
        step_until(&mut plant, ik_down(cube), 0.0);
        step_until(&mut plant, ik_down(cube), 1.0);
        assert!(plant.state().grasped);
        step_until(&mut plant, ik_down([cube[0], cube[1], 0.25]), 1.0);
        let lifted = plant.state();
        assert!(lifted.task_success, "{lifted:?}");
        step_until(&mut plant, ik_down([cube[0], cube[1], 0.25]), 0.0);
        let dropped = plant.state();
        assert!(!dropped.grasped && !dropped.task_success);
        assert_eq!(dropped.cube_position[2], CUBE_HALF_M);
    }

    #[test]
    fn closing_away_from_the_cube_grasps_nothing() {
        let mut plant = Plant::reset(3);
        step_until(&mut plant, HOME_Q, 1.0);
        assert!(!plant.state().grasped);
    }

    #[test]
    fn estop_latches_until_reset_and_refusals_leave_the_plant_unchanged() {
        let mut plant = Plant::reset(1);
        let r = plant.apply(r#"{"call_id":1,"tool":"estop","args":{"reason":"test"}}"#);
        assert!(r.ok && r.state.estopped);
        let before = plant.state();
        let r = plant.apply(
            r#"{"call_id":2,"tool":"arm.step","args":{"q_target":[0,0,0,0,0,0],"gripper":0}}"#,
        );
        assert!(!r.ok);
        assert_eq!(r.error.as_ref().map(|e| e.code), Some("estopped"));
        assert_eq!(plant.state(), before);
        let r = plant.apply(r#"{"call_id":3,"tool":"arm.reset","args":{"seed":1}}"#);
        assert!(r.ok && !r.state.estopped);
    }

    #[test]
    fn out_of_limit_and_malformed_calls_are_refused_with_codes() {
        let mut plant = Plant::reset(1);
        let r = plant.apply(
            r#"{"call_id":4,"tool":"arm.step","args":{"q_target":[0,0,0,3.5,0,0],"gripper":0}}"#,
        );
        assert_eq!(r.error.as_ref().map(|e| e.code), Some("joint_limit"));
        let r = plant.apply(r#"{"call_id":5,"tool":"arm.step","args":{"q_target":[0,0]}}"#);
        assert_eq!(r.error.as_ref().map(|e| e.code), Some("invalid_args"));
        let r = plant.apply("not json");
        assert_eq!(
            (r.call_id, r.error.as_ref().map(|e| e.code)),
            (0, Some("invalid_args"))
        );
        let r = plant.apply(r#"{"call_id":6,"tool":"arm.fly"}"#);
        assert_eq!(r.error.as_ref().map(|e| e.code), Some("unknown_tool"));
    }

    #[test]
    fn the_same_calls_serialize_to_the_same_bytes() {
        let calls = [
            r#"{"call_id":1,"tool":"arm.reset","args":{"seed":11}}"#,
            r#"{"call_id":2,"tool":"arm.step","args":{"q_target":[0.1,0.8,-1.3,-1.0,0.2,0],"gripper":0.6}}"#,
            r#"{"call_id":3,"tool":"arm.step","args":{"q_target":[0.1,0.8,-1.3,-1.0,0.2,0],"gripper":0.6}}"#,
        ];
        let run = || {
            let mut plant = Plant::default();
            calls
                .iter()
                .map(|c| serde_json::to_string(&plant.apply(c)).expect("json"))
                .collect::<Vec<_>>()
        };
        assert_eq!(run(), run());
    }
}
