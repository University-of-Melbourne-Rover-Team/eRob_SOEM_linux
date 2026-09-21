//! Same-host Unix datagram bridge. Wire layout is defined in demo/csp_command.h.
//! No socket thread: receive before input/IK, send after input/IK each Bevy frame.

use bevy::prelude::*;
use std::fs;
use std::io::ErrorKind;
use std::os::unix::net::UnixDatagram;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use crate::kinematics::{EncoderCalibration, KinematicsState};
use crate::model::RobotModel;
use crate::scene::JointState;

const MOTOR_COUNT: usize = 6;
const SERVER_PATH: &str = "/tmp/csp.sock";
const FEEDBACK_SIZE: usize = 48;
const FEEDBACK_TIMEOUT: Duration = Duration::from_millis(250);

#[derive(Resource)]
pub(crate) struct HardwareBridge {
    socket: UnixDatagram,
    client_path: PathBuf,
    calibration: [EncoderCalibration; MOTOR_COUNT],
    last_feedback: Option<Instant>,
    last_subscribe: Option<Instant>,
    last_sent: Option<[i32; MOTOR_COUNT]>,
    needs_seed: bool,
    pub(crate) reset_task_target: bool,
    startup_command: bool,
    ready: bool,
    pub(crate) statuswords: [u16; MOTOR_COUNT],
    pub(crate) last_error: Option<String>,
}

impl HardwareBridge {
    pub(crate) fn open(path: &Path, model: &RobotModel, startup_command: bool) -> Result<Self, String> {
        let text = fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
        let mut calibration = Vec::new();
        for (line_number, line) in text.lines().enumerate() {
            let fields = line.split('#').next().unwrap_or("").split_whitespace().collect::<Vec<_>>();
            if fields.is_empty() { continue; }
            let invalid = || format!("{}:{}: expected joint counts_per_turn zero_count direction",
                                      path.display(), line_number + 1);
            if fields.len() != 4 { return Err(invalid()); }
            let entry = EncoderCalibration {
                joint_name: fields[0].to_string(),
                counts_per_turn: fields[1].parse::<f64>().map_err(|_| invalid())?,
                zero_count: fields[2].parse::<i32>().map_err(|_| invalid())?,
                direction: fields[3].parse::<f64>().map_err(|_| invalid())?,
            };
            if !entry.counts_per_turn.is_finite() || entry.counts_per_turn <= 0.0
                || (entry.direction != -1.0 && entry.direction != 1.0)
            { return Err(invalid()); }
            if !model.joints.iter().any(|j| j.is_moving() && j.name == entry.joint_name)
                || calibration.iter().any(|j: &EncoderCalibration| j.joint_name == entry.joint_name)
            { return Err(format!("Unknown or duplicate joint: {}", entry.joint_name)); }
            calibration.push(entry);
        }
        if model.joints.iter().filter(|j| j.is_moving()).count() != MOTOR_COUNT {
            return Err("Hardware mode requires six movable URDF joints".to_string());
        }
        let calibration: [EncoderCalibration; MOTOR_COUNT] = calibration.try_into()
            .map_err(|_| "Hardware calibration requires exactly six rows".to_string())?;
        let nonce = SystemTime::now().duration_since(UNIX_EPOCH).map_err(|e| e.to_string())?.as_nanos();
        let client_path = PathBuf::from(format!("/tmp/csp-bevy-{}-{nonce}.sock", std::process::id()));
        let socket = UnixDatagram::bind(&client_path).map_err(|e| e.to_string())?;
        if let Err(error) = socket.set_nonblocking(true) {
            let _ = fs::remove_file(&client_path);
            return Err(error.to_string());
        }
        Ok(Self {
            socket, client_path, calibration, last_feedback: None,
            last_subscribe: None, last_sent: None, needs_seed: true, reset_task_target: false,
            startup_command, ready: false, statuswords: [0; MOTOR_COUNT], last_error: None,
        })
    }

    pub(crate) fn fresh(&self) -> bool {
        self.last_feedback.is_some_and(|at| at.elapsed() < FEEDBACK_TIMEOUT)
    }

    pub(crate) fn has_feedback(&self) -> bool { self.last_feedback.is_some() }

    pub(crate) fn can_command(&self) -> bool { self.fresh() && self.ready }

    pub(crate) fn status(&self) -> &'static str {
        if self.last_feedback.is_none() { "Waiting for CSP feedback" }
        else if !self.fresh() { "Feedback stale - command input paused" }
        else if !self.ready { "Actual feedback - drives not ready" }
        else { "Live actual feedback" }
    }

    fn poll(&mut self) -> Option<[f32; MOTOR_COUNT]> {
        let was_fresh = self.fresh();
        if !was_fresh && self.last_feedback.is_some() {
            self.needs_seed = true;
            self.startup_command = false; // Never replay an old startup target after a disconnect.
            self.last_sent = None;
        }
        // Subscription does not move the robot, and also reattaches after CSP restarts.
        if self.last_subscribe.is_none_or(|at| at.elapsed() >= Duration::from_millis(250)) {
            self.last_subscribe = Some(Instant::now());
            if let Err(error) = self.socket.send_to(b"CSU1", SERVER_PATH) {
                if error.kind() != ErrorKind::WouldBlock && error.kind() != ErrorKind::Interrupted {
                    self.last_error = Some(format!("CSP subscribe: {error}"));
                }
            }
        }
        let mut latest = None;
        for _ in 0..64 {
            // One extra byte rejects oversized datagrams even when recv_from truncates them.
            let mut bytes = [0u8; FEEDBACK_SIZE + 1];
            match self.socket.recv_from(&mut bytes) {
                Ok((size, sender)) => {
                    if sender.as_pathname() != Some(Path::new(SERVER_PATH))
                        || size != FEEDBACK_SIZE || &bytes[..4] != b"CSF1"
                    {
                        self.last_error = Some("Ignored malformed CSP feedback".to_string());
                        continue;
                    }
                    let mut counts = [0i32; MOTOR_COUNT];
                    for i in 0..MOTOR_COUNT {
                        let offset = 8 + 4 * i;
                        counts[i] = i32::from_ne_bytes(bytes[offset..offset + 4].try_into().unwrap());
                        let status_offset = 32 + 2 * i;
                        self.statuswords[i] = u16::from_ne_bytes(bytes[status_offset..status_offset + 2].try_into().unwrap());
                    }
                    let ready = u32::from_ne_bytes(bytes[44..48].try_into().unwrap()) & 1 != 0;
                    if self.ready && !ready {
                        self.needs_seed = true;
                        self.startup_command = false;
                        self.last_sent = None;
                    }
                    self.ready = ready;
                    self.last_feedback = Some(Instant::now());
                    self.last_error = None;
                    latest = Some(std::array::from_fn(|i|
                        self.calibration[i].counts_to_radians(counts[i] as f64)));
                }
                Err(error) if error.kind() == ErrorKind::WouldBlock => break,
                Err(error) if error.kind() == ErrorKind::Interrupted => continue,
                Err(error) => {
                    self.last_error = Some(format!("CSP feedback: {error}"));
                    break;
                }
            }
        }
        latest
    }
}

impl Drop for HardwareBridge {
    fn drop(&mut self) { let _ = fs::remove_file(&self.client_path); }
}

pub(crate) fn receive_hardware_feedback(
    bridge: Option<ResMut<HardwareBridge>>,
    mut joints: Query<(&mut JointState, &mut Transform)>,
    kinematics: ResMut<KinematicsState>,
) {
    let Some(mut bridge) = bridge else { return; };
    let Some(angles) = bridge.poll() else { return; };
    for (mut joint, mut transform) in &mut joints {
        if let Some(index) = bridge.calibration.iter().position(|c| c.joint_name == joint.name) {
            joint.value = angles[index]; // Never clamp measured positions to a desired joint limit.
            if bridge.needs_seed && !bridge.startup_command {
                joint.target_value = joint.value;
            }
            joint.update_transform(&mut transform);
        }
    }
    if bridge.needs_seed && !bridge.startup_command {
        // Seed the command comparison from feedback without transmitting a move.
        let seed = bridge.calibration.iter().zip(angles).map(|(calibration, angle)|
            calibration.radians_to_counts(angle as f64)).collect::<Result<Vec<_>, _>>();
        match seed {
            Ok(counts) => bridge.last_sent = counts.try_into().ok(),
            Err(error) => {
                bridge.last_error = Some(error);
                bridge.ready = false;
                return;
            }
        }
        bridge.reset_task_target = true;
    }
    // Follow actual positions throughout startup, then seed once more on the
    // first ready sample. Early SAFE_OP values must not become later targets.
    bridge.needs_seed = !bridge.ready;
    let positions = kinematics.joint_names.iter().map(|name| {
        bridge.calibration.iter().position(|c| &c.joint_name == name)
            .map(|index| angles[index] as f64).unwrap_or(0.0)
    }).collect::<Vec<_>>();
    if let Err(error) = kinematics.chain.set_joint_positions(&positions) {
        bridge.last_error = Some(format!("Measured kinematics: {error}"));
    } else {
        kinematics.chain.update_transforms();
    }
}

pub(crate) fn send_hardware_commands(
    bridge: Option<ResMut<HardwareBridge>>,
    joints: Query<&JointState>,
) {
    let Some(mut bridge) = bridge else { return; };
    if !bridge.can_command() { return; }
    let encoded = bridge.calibration.iter().map(|calibration| {
        let joint = joints.iter().find(|joint| joint.name == calibration.joint_name)
            .ok_or_else(|| format!("Missing hardware joint: {}", calibration.joint_name))?;
        calibration.radians_to_counts(joint.target_value as f64)
    }).collect::<Result<Vec<_>, String>>();
    let counts: [i32; MOTOR_COUNT] = match encoded {
        Ok(counts) => counts.try_into().expect("six calibrated joints"),
        Err(error) => { bridge.last_error = Some(error); return; }
    };
    if bridge.last_sent == Some(counts) { return; }
    let mut bytes = [0u8; MOTOR_COUNT * 4];
    for (i, count) in counts.iter().enumerate() {
        bytes[i * 4..i * 4 + 4].copy_from_slice(&count.to_ne_bytes());
    }
    match bridge.socket.send_to(&bytes, SERVER_PATH) {
        Ok(size) if size == bytes.len() => {
            bridge.last_sent = Some(counts);
            bridge.startup_command = false;
            bridge.last_error = None;
        }
        Err(error) if error.kind() == ErrorKind::WouldBlock || error.kind() == ErrorKind::Interrupted => {
            // Retry only the latest desired array on the next frame.
        }
        Err(error) => { bridge.last_error = Some(format!("CSP command: {error}")); }
        Ok(_) => { bridge.last_error = Some("Incomplete CSP command send".to_string()); }
    }
}
