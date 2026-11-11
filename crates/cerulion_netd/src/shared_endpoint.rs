// SPDX-License-Identifier: AGPL-3.0-only
//! Bounded same-user outbound mirrors on the remote server's owned endpoint.

use std::collections::HashMap;
use std::io::{Read, Write};
use std::net::{Shutdown, SocketAddr};
use std::os::unix::net::UnixStream;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, TryLockError};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use cerulion_core::transport::cerulion_q::{CatalogReply, SchemaReply};
use cerulion_core::TransportManager;
use cerulion_link::{Endpoint, EndpointId, RelayConfig};
use cerulion_pairing::client::DeviceIdentity;
use cerulion_pairing::verify::OwnerCertificatePresentationWire;
use rustix::event::{poll, PollFd, PollFlags, Timespec};
use serde::{Deserialize, Serialize};

use crate::iroh_plane::{IrohMirrorPlane, RobotProbeOutcome};
use crate::mirror::{MirrorPlane, MirrorRelease};
use crate::registry::TopicKey;
use crate::serving_hold::{HoldReason, ServingHold, ServingHolds};
use crate::wan::{WanRegistry, WanRobot};

const MAX_MESSAGE: usize = 256 * 1024;
pub(crate) const MAX_CLIENTS: usize = 32;
const MAX_ROBOTS: usize = 256;
pub(crate) const CONTROL_TIMEOUT: Duration = Duration::from_secs(20);

fn remaining(deadline: Instant) -> Result<Duration, String> {
    deadline
        .checked_duration_since(Instant::now())
        .filter(|duration| !duration.is_zero())
        .ok_or_else(|| "outbound shared operation deadline expired".to_owned())
}

fn wait_for_io(stream: &UnixStream, interest: PollFlags, deadline: Instant) -> Result<(), String> {
    loop {
        let timeout = Timespec::try_from(remaining(deadline)?)
            .map_err(|_| "outbound shared operation deadline is too distant")?;
        let mut descriptors = [PollFd::new(stream, interest)];
        match poll(&mut descriptors, Some(&timeout)) {
            Ok(0) => {
                remaining(deadline)?;
            }
            Ok(_) => return Ok(()),
            Err(rustix::io::Errno::INTR) => continue,
            Err(_) => return Err("outbound control readiness failed".into()),
        }
    }
}

fn read_exact_until(
    stream: &mut UnixStream,
    mut bytes: &mut [u8],
    deadline: Instant,
) -> Result<(), String> {
    while !bytes.is_empty() {
        remaining(deadline)?;
        match stream.read(bytes) {
            Ok(0) => return Err("outbound control connection ended".into()),
            Ok(count) => bytes = &mut bytes[count..],
            Err(error) if error.kind() == std::io::ErrorKind::Interrupted => continue,
            Err(error) if error.kind() == std::io::ErrorKind::WouldBlock => {
                wait_for_io(stream, PollFlags::IN, deadline)?
            }
            Err(_) => return Err("outbound control connection read failed".into()),
        }
    }
    Ok(())
}

fn write_all_until(
    stream: &mut UnixStream,
    mut bytes: &[u8],
    deadline: Instant,
) -> Result<(), String> {
    while !bytes.is_empty() {
        remaining(deadline)?;
        match stream.write(bytes) {
            Ok(0) => return Err("outbound control connection write failed".into()),
            Ok(count) => bytes = &bytes[count..],
            Err(error) if error.kind() == std::io::ErrorKind::Interrupted => continue,
            Err(error) if error.kind() == std::io::ErrorKind::WouldBlock => {
                wait_for_io(stream, PollFlags::OUT, deadline)?
            }
            Err(_) => return Err("outbound control connection write failed".into()),
        }
    }
    Ok(())
}

/// The owned server and consumer resolve this path from the same state root.
pub fn socket_path(root: &Path) -> PathBuf {
    root.join("remoted/outbound.sock")
}

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Peer {
    name: String,
    key: [u8; 32],
    addresses: Vec<SocketAddr>,
}

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Configuration {
    key: [u8; 32],
    account: Option<[u8; 32]>,
    epoch_dir: Option<PathBuf>,
    owner: Option<Vec<u8>>,
    namespace: String,
    peers: Vec<Peer>,
}

#[derive(Serialize, Deserialize)]
#[serde(tag = "op", rename_all = "snake_case", deny_unknown_fields)]
enum Operation {
    Probe {
        robot: String,
        budget_ms: u64,
    },
    Catalog {
        robot: String,
        budget_ms: u64,
    },
    Schema {
        robot: String,
        topic: String,
        budget_ms: u64,
    },
    Ensure {
        robot: String,
        topic: String,
        schema_hash: u64,
        observer: bool,
    },
    Release {
        robot: String,
        topic: String,
        observer: bool,
    },
    HasReader {
        robot: String,
        topic: String,
        observer: bool,
    },
    Retire {
        robot: String,
    },
    Sync,
    Invalidate,
}

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Request {
    configuration: Configuration,
    operation: Operation,
}

#[derive(Serialize, Deserialize)]
enum Answer {
    Done,
    Present(bool),
    Probe(Option<String>),
    Catalog(
        CatalogReply,
        crate::account_access::RobotPath,
        Option<SocketAddr>,
    ),
    Schema(SchemaReply),
    Released(bool),
    Retired(Vec<(String, String, bool)>),
}

enum RequestError {
    Transport(String),
    Refused(String),
}

impl RequestError {
    fn message(self) -> String {
        match self {
            Self::Transport(message) | Self::Refused(message) => message,
        }
    }
}

fn topic_key(robot: String, topic: String, observer: bool) -> TopicKey {
    if observer {
        TopicKey::observed(robot, topic)
    } else {
        TopicKey::new(robot, topic)
    }
}

fn encode_message(value: &impl Serialize) -> Result<Vec<u8>, String> {
    let bytes = serde_json::to_vec(value).map_err(|_| "outbound control encoding failed")?;
    if bytes.len() > MAX_MESSAGE {
        return Err("outbound control message exceeds its byte limit".into());
    }
    Ok(bytes)
}

fn write_encoded(stream: &mut UnixStream, bytes: &[u8]) -> Result<(), String> {
    stream
        .write_all(&(bytes.len() as u32).to_le_bytes())
        .and_then(|()| stream.write_all(bytes))
        .map_err(|_| "outbound control connection write failed".into())
}

fn write_message(stream: &mut UnixStream, value: &impl Serialize) -> Result<(), String> {
    write_encoded(stream, &encode_message(value)?)
}

fn read_message<T: for<'a> Deserialize<'a>>(
    stream: &mut UnixStream,
    idle: bool,
) -> Result<T, String> {
    let mut size = [0; 4];
    loop {
        match stream.read_exact(&mut size[..1]) {
            Ok(()) => break,
            Err(error)
                if idle
                    && matches!(
                        error.kind(),
                        std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
                    ) => {}
            Err(_) => return Err("outbound control connection ended".into()),
        }
    }
    stream
        .read_exact(&mut size[1..])
        .map_err(|_| "outbound control header was truncated")?;
    let size = u32::from_le_bytes(size) as usize;
    if size == 0 || size > MAX_MESSAGE {
        return Err("outbound control message exceeds its byte limit".into());
    }
    let mut bytes = vec![0; size];
    stream
        .read_exact(&mut bytes)
        .map_err(|_| "outbound control message was truncated")?;
    serde_json::from_slice(&bytes).map_err(|_| "outbound control message is invalid".into())
}

fn configure_stream(stream: &UnixStream) -> Result<(), String> {
    let runtime = tokio::runtime::Builder::new_current_thread()
        .enable_io()
        .build()
        .map_err(|_| "outbound peer credential runtime setup failed")?;
    let entered = runtime.enter();
    let copy = stream
        .try_clone()
        .map_err(|_| "outbound peer credential socket setup failed")?;
    copy.set_nonblocking(true)
        .map_err(|_| "outbound peer credential socket setup failed")?;
    let peer = tokio::net::UnixStream::from_std(copy).and_then(|socket| socket.peer_cred());
    stream
        .set_nonblocking(false)
        .map_err(|_| "outbound peer credential socket restoration failed")?;
    drop(entered);
    if peer
        .map_err(|_| "outbound peer identity is unavailable")?
        .uid()
        != cerulion_hygiene::euid()
    {
        return Err("outbound control requires the same local user".into());
    }
    stream
        .set_read_timeout(Some(CONTROL_TIMEOUT))
        .and_then(|()| stream.set_write_timeout(Some(CONTROL_TIMEOUT)))
        .map_err(|_| "outbound control deadline setup failed".into())
}

impl Configuration {
    fn owner_certificate(&self) -> Result<Option<Arc<OwnerCertificatePresentationWire>>, String> {
        // Preserve the certificate's byte-string signature encoding on this JSON seam.
        self.owner
            .as_deref()
            .map(|bytes| {
                OwnerCertificatePresentationWire::from_postcard(bytes)
                    .map(Arc::new)
                    .map_err(|error| error.to_string())
            })
            .transpose()
    }
    fn robots(&self) -> Result<HashMap<String, WanRobot>, String> {
        if self.peers.len() > MAX_ROBOTS {
            return Err("outbound robot membership exceeds its limit".into());
        }
        let mut robots = HashMap::new();
        for peer in &self.peers {
            if peer.name.is_empty()
                || peer.name.trim() != peer.name
                || peer.name.len() > 256
                || peer.addresses.len() > 16
            {
                return Err("outbound robot membership is invalid".into());
            }
            let eid =
                EndpointId::from_bytes(&peer.key).map_err(|_| "outbound robot key is invalid")?;
            if robots
                .insert(
                    peer.name.clone(),
                    WanRobot {
                        eid,
                        direct_addrs: peer.addresses.clone(),
                    },
                )
                .is_some()
            {
                return Err("outbound robot membership has duplicate names".into());
            }
        }
        Ok(robots)
    }
}

/// A local connection owns all its mirrors; close is crash-safe release.
pub(crate) struct SharedMirrorPlane {
    socket: PathBuf,
    manager: Arc<TransportManager>,
    registry: Arc<WanRegistry>,
    connection: Mutex<Option<UnixStream>>,
    invalidated: AtomicBool,
    transport_lost: AtomicBool,
}

impl SharedMirrorPlane {
    pub(crate) fn new(
        socket: PathBuf,
        manager: Arc<TransportManager>,
        registry: Arc<WanRegistry>,
    ) -> Self {
        Self {
            socket,
            manager,
            registry,
            connection: Mutex::new(None),
            invalidated: AtomicBool::new(false),
            transport_lost: AtomicBool::new(false),
        }
    }

    fn configuration(&self) -> Result<Configuration, String> {
        let mut peers: Vec<_> = self
            .registry
            .membership_snapshot()?
            .into_iter()
            .map(|(name, peer)| Peer {
                name,
                key: *peer.eid.as_bytes(),
                addresses: peer.direct_addrs,
            })
            .collect();
        peers.sort_by(|a, b| a.name.cmp(&b.name));
        Ok(Configuration {
            key: DeviceIdentity::from_seed(&self.registry.desk_seed())
                .public_key()
                .0,
            account: self.registry.account(),
            epoch_dir: self.registry.epoch_dir().map(Path::to_owned),
            owner: self
                .registry
                .owner_certificate()?
                .map(|proof| proof.to_postcard().map_err(|error| error.to_string()))
                .transpose()?,
            namespace: serde_json::to_string(&self.manager.iox_config())
                .map_err(|_| "outbound namespace encoding failed")?,
            peers,
        })
    }

    fn request(
        stream: &mut UnixStream,
        configuration: Configuration,
        operation: Operation,
    ) -> Result<Answer, RequestError> {
        let bytes = encode_message(&Request {
            configuration,
            operation,
        })
        .map_err(RequestError::Refused)?;
        write_encoded(stream, &bytes).map_err(RequestError::Transport)?;
        read_message::<Result<Answer, String>>(stream, false)
            .map_err(RequestError::Transport)?
            .map_err(RequestError::Refused)
    }

    fn request_until(
        stream: &mut UnixStream,
        configuration: Configuration,
        operation: Operation,
        deadline: Instant,
    ) -> Result<Answer, RequestError> {
        let bytes = encode_message(&Request {
            configuration,
            operation,
        })
        .map_err(RequestError::Refused)?;
        remaining(deadline).map_err(RequestError::Refused)?;
        stream.set_nonblocking(true).map_err(|_| {
            RequestError::Transport("outbound control nonblocking setup failed".into())
        })?;
        let result = (|| {
            write_all_until(stream, &(bytes.len() as u32).to_le_bytes(), deadline)
                .map_err(RequestError::Transport)?;
            write_all_until(stream, &bytes, deadline).map_err(RequestError::Transport)?;
            let mut header = [0; 4];
            read_exact_until(stream, &mut header, deadline).map_err(RequestError::Transport)?;
            let size = u32::from_le_bytes(header) as usize;
            if size == 0 || size > MAX_MESSAGE {
                return Err(RequestError::Transport(
                    "outbound control message exceeds its byte limit".into(),
                ));
            }
            let mut reply = vec![0; size];
            read_exact_until(stream, &mut reply, deadline).map_err(RequestError::Transport)?;
            remaining(deadline).map_err(RequestError::Transport)?;
            serde_json::from_slice::<Result<Answer, String>>(&reply)
                .map_err(|_| RequestError::Transport("outbound control message is invalid".into()))?
                .map_err(RequestError::Refused)
        })();
        if stream.set_nonblocking(false).is_err() {
            return Err(RequestError::Transport(
                "outbound control socket restoration failed".into(),
            ));
        }
        result
    }

    fn lock_connection(
        &self,
        deadline: Option<Instant>,
    ) -> Result<MutexGuard<'_, Option<UnixStream>>, String> {
        let Some(deadline) = deadline else {
            return self
                .connection
                .lock()
                .map_err(|_| "outbound control state is poisoned".into());
        };
        // Wait for the same mutation lock, without turning contention into a busy refusal.
        loop {
            let budget = remaining(deadline)?;
            match self.connection.try_lock() {
                Ok(connection) => return Ok(connection),
                Err(TryLockError::Poisoned(_)) => {
                    return Err("outbound control state is poisoned".into())
                }
                Err(TryLockError::WouldBlock) => {
                    std::thread::sleep(budget.min(Duration::from_millis(1)))
                }
            }
        }
    }

    fn call(&self, operation: Operation) -> Result<Answer, String> {
        self.call_until(operation, None)
    }

    fn call_until(
        &self,
        mut operation: Operation,
        deadline: Option<Instant>,
    ) -> Result<Answer, String> {
        let started = Instant::now();
        let budget = match &operation {
            Operation::Probe { budget_ms, .. }
            | Operation::Catalog { budget_ms, .. }
            | Operation::Schema { budget_ms, .. } => Some(Duration::from_millis(*budget_ms)),
            _ => None,
        };
        if self.invalidated.load(Ordering::Acquire) {
            return Err("outbound account changed; recreate the mirror plane".into());
        }
        if let Some(budget) = budget {
            // Discovery rows own no mirrors and cannot fence the persistent
            // connection on an offline peer, refusal or read timeout.
            let configuration = self.configuration()?;
            let mut stream = UnixStream::connect(&self.socket)
                .map_err(|_| "owned remote endpoint is not ready for outbound access")?;
            configure_stream(&stream)?;
            let remaining = budget.saturating_sub(started.elapsed());
            if remaining.is_zero() {
                return Err("outbound metadata operation exceeded its time budget".into());
            }
            let remote_budget = remaining.saturating_sub(Duration::from_millis(250));
            match &mut operation {
                Operation::Probe { budget_ms, .. }
                | Operation::Catalog { budget_ms, .. }
                | Operation::Schema { budget_ms, .. } => {
                    *budget_ms = remote_budget.as_millis() as u64;
                }
                _ => unreachable!("only metadata operations have a time budget"),
            }
            stream
                .set_read_timeout(Some(remaining))
                .and_then(|()| stream.set_write_timeout(Some(remaining)))
                .map_err(|_| "outbound control deadline setup failed")?;
            if self.invalidated.load(Ordering::Acquire) {
                return Err("outbound account changed; recreate the mirror plane".into());
            }
            let result =
                Self::request(&mut stream, configuration, operation).map_err(RequestError::message);
            let _closed = stream.shutdown(Shutdown::Both);
            return result;
        }
        let mut connection = self.lock_connection(deadline)?;
        if self.invalidated.load(Ordering::Acquire) {
            return Err("outbound account changed; recreate the mirror plane".into());
        }
        let configuration = self.configuration()?;
        if connection.is_none() {
            let stream = if let Some(deadline) = deadline {
                let address = socket2::SockAddr::unix(&self.socket)
                    .map_err(|_| "outbound control socket address is invalid")?;
                let socket =
                    socket2::Socket::new(socket2::Domain::UNIX, socket2::Type::STREAM, None)
                        .map_err(|_| "outbound control socket setup failed")?;
                socket
                    .connect_timeout(&address, remaining(deadline)?)
                    .and_then(|()| socket.peer_addr().map(|_| ()))
                    .map_err(|_| "owned remote endpoint is not ready for outbound access")?;
                socket.into()
            } else {
                UnixStream::connect(&self.socket)
                    .map_err(|_| "owned remote endpoint is not ready for outbound access")?
            };
            configure_stream(&stream)?;
            *connection = Some(stream);
        }
        let stream = connection
            .as_mut()
            .ok_or("outbound connection was not constructed")?;
        let result = if let Some(deadline) = deadline {
            Self::request_until(stream, configuration, operation, deadline)
        } else {
            let deadline_setup = stream
                .set_read_timeout(Some(CONTROL_TIMEOUT))
                .and_then(|()| stream.set_write_timeout(Some(CONTROL_TIMEOUT)));
            if deadline_setup.is_err() {
                // macOS can reject timeout setup on a disconnected socket before writing.
                connection.take();
                self.transport_lost.store(true, Ordering::Release);
                return Err("outbound control deadline setup failed".into());
            }
            Self::request(stream, configuration, operation)
        };
        if matches!(result, Err(RequestError::Transport(_))) {
            // A framing failure fences the whole connection; never retry an operation
            // whose result may have been committed on the old endpoint.
            connection.take();
            self.transport_lost.store(true, Ordering::Release);
        }
        result.map_err(RequestError::message)
    }

    pub(crate) fn take_transport_loss(&self) -> bool {
        self.transport_lost.swap(false, Ordering::AcqRel)
    }

    pub(crate) fn invalidate(&self) -> Result<(), String> {
        let mut connection = self
            .connection
            .lock()
            .map_err(|_| "outbound control state is poisoned")?;
        // Serialize the fence with requests, including demands that were waiting
        // for this lock before invalidation began.
        self.invalidated.store(true, Ordering::Release);
        let retired = match connection.as_mut() {
            Some(stream) => self.configuration().and_then(|configuration| {
                Self::request(stream, configuration, Operation::Invalidate)
                    .map(|_| ())
                    .map_err(RequestError::message)
            }),
            None => Ok(()),
        };
        if let Some(stream) = connection.take() {
            let _closed = stream.shutdown(Shutdown::Both);
        }
        retired
    }
    pub(crate) fn sync_until(&self, deadline: Instant) -> Result<Vec<TopicKey>, String> {
        match self.call_until(Operation::Sync, Some(deadline))? {
            Answer::Retired(keys) => Ok(keys
                .into_iter()
                .map(|(robot, topic, observer)| topic_key(robot, topic, observer))
                .collect()),
            _ => Err("outbound membership reply is invalid".into()),
        }
    }
    pub(crate) fn retire(&self, robot: &str) -> Result<(), String> {
        self.call(Operation::Retire {
            robot: robot.into(),
        })
        .map(|_| ())
    }
    pub(crate) fn has_reader(&self, key: &TopicKey) -> Result<bool, String> {
        match self.call(Operation::HasReader {
            robot: key.robot.clone(),
            topic: key.topic.clone(),
            observer: key.observer,
        })? {
            Answer::Present(present) => Ok(present),
            _ => Err("outbound reader reply is invalid".into()),
        }
    }
    pub(crate) fn ensure_until(
        &self,
        key: &TopicKey,
        schema_hash: u64,
        deadline: Instant,
    ) -> Result<(), String> {
        self.call_until(
            Operation::Ensure {
                robot: key.robot.clone(),
                topic: key.topic.clone(),
                observer: key.observer,
                schema_hash,
            },
            Some(deadline),
        )
        .map(|_| ())
    }
    pub(crate) fn release(&self, key: &TopicKey) -> MirrorRelease {
        match self.call(Operation::Release {
            robot: key.robot.clone(),
            topic: key.topic.clone(),
            observer: key.observer,
        }) {
            Ok(Answer::Released(true)) => MirrorRelease::Retired,
            _ => MirrorRelease::Lingering,
        }
    }
    pub(crate) fn probe(
        &self,
        robot: &str,
        timeout: Duration,
    ) -> Result<RobotProbeOutcome, String> {
        match self.call(Operation::Probe {
            robot: robot.into(),
            budget_ms: timeout.as_millis().min(15_000) as u64,
        })? {
            Answer::Probe(None) => Ok(RobotProbeOutcome::Online),
            Answer::Probe(Some(reason)) => Ok(RobotProbeOutcome::NotReached { reason }),
            _ => Err("outbound probe reply is invalid".into()),
        }
    }
    pub(crate) fn catalog(
        &self,
        robot: &str,
        timeout: Duration,
    ) -> Result<
        (
            CatalogReply,
            crate::account_access::RobotPath,
            Option<SocketAddr>,
        ),
        String,
    > {
        match self.call(Operation::Catalog {
            robot: robot.into(),
            budget_ms: timeout.as_millis().min(15_000) as u64,
        })? {
            Answer::Catalog(catalog, path, direct) => Ok((catalog, path, direct)),
            _ => Err("outbound catalog reply is invalid".into()),
        }
    }
    pub(crate) fn schema(
        &self,
        robot: &str,
        topic: &str,
        timeout: Duration,
    ) -> Result<SchemaReply, String> {
        match self.call(Operation::Schema {
            robot: robot.into(),
            topic: topic.into(),
            budget_ms: timeout.as_millis().min(15_000) as u64,
        })? {
            Answer::Schema(schema) => Ok(schema),
            _ => Err("outbound schema reply is invalid".into()),
        }
    }
}

/// Local control service hosted beside the machine's one serving endpoint.
/// Its transport manager is shared with serving; other namespaces are refused.
pub struct OutboundServer {
    running: Arc<AtomicBool>,
    clients: Arc<Mutex<HashMap<u64, UnixStream>>>,
    accept: Option<JoinHandle<()>>,
}

impl OutboundServer {
    /// Active local sessions, independent of log output and wire progress.
    pub fn active_clients(&self) -> Result<usize, String> {
        self.clients
            .lock()
            .map(|clients| clients.len())
            .map_err(|_| "outbound control state is poisoned".into())
    }

    /// Start bounded local control without binding another network endpoint.
    pub fn start(
        socket: PathBuf,
        endpoint: Endpoint,
        runtime: tokio::runtime::Handle,
        seed: [u8; 32],
        manager: Arc<TransportManager>,
    ) -> Result<Self, String> {
        Self::start_inner(socket, endpoint, runtime, seed, manager, None)
    }

    /// Start owned outbound control with atomic admission against idle teardown.
    pub fn start_with_holds(
        socket: PathBuf,
        endpoint: Endpoint,
        runtime: tokio::runtime::Handle,
        seed: [u8; 32],
        manager: Arc<TransportManager>,
        holds: ServingHolds,
    ) -> Result<Self, String> {
        Self::start_inner(socket, endpoint, runtime, seed, manager, Some(holds))
    }

    fn start_inner(
        socket: PathBuf,
        endpoint: Endpoint,
        runtime: tokio::runtime::Handle,
        seed: [u8; 32],
        manager: Arc<TransportManager>,
        holds: Option<ServingHolds>,
    ) -> Result<Self, String> {
        if endpoint.id().as_bytes() != &DeviceIdentity::from_seed(&seed).public_key().0 {
            return Err("owned endpoint does not match its device key".into());
        }
        let (listener, guard) = crate::hygiene::acquire_socket(socket)
            .map_err(|_| "outbound control socket acquisition failed")?;
        listener
            .set_nonblocking(true)
            .map_err(|_| "outbound control listener setup failed")?;
        let running = Arc::new(AtomicBool::new(true));
        let clients = Arc::new(Mutex::new(HashMap::new()));
        let live = running.clone();
        let active = clients.clone();
        let accept = std::thread::Builder::new()
            .name("remoted-outbound".into())
            .spawn(move || {
                let _guard = guard;
                let mut workers: Vec<JoinHandle<()>> = Vec::new();
                let mut next_id = 0u64;
                while live.load(Ordering::Acquire) {
                    let mut i = 0;
                    while i < workers.len() {
                        if workers[i].is_finished() {
                            let _joined = workers.swap_remove(i).join();
                        } else {
                            i += 1;
                        }
                    }
                    match listener.accept() {
                        Ok((stream, _)) if workers.len() < MAX_CLIENTS => {
                            if configure_stream(&stream).is_err() {
                                continue;
                            }
                            let Ok(copy) = stream.try_clone() else {
                                continue;
                            };
                            let id = next_id;
                            next_id = next_id.wrapping_add(1);
                            let Ok(mut sockets) = active.lock() else {
                                break;
                            };
                            if !live.load(Ordering::Acquire) {
                                break;
                            }
                            sockets.insert(id, copy);
                            drop(sockets);
                            let active = active.clone();
                            let endpoint = endpoint.clone();
                            let runtime = runtime.clone();
                            let manager = manager.clone();
                            let holds = holds.clone();
                            workers.push(std::thread::spawn(move || {
                                serve_connection(stream, endpoint, runtime, seed, manager, holds);
                                if let Ok(mut sockets) = active.lock() {
                                    sockets.remove(&id);
                                }
                            }));
                        }
                        Ok(_) => {}
                        Err(error) if error.kind() == std::io::ErrorKind::WouldBlock => {
                            std::thread::sleep(Duration::from_millis(10))
                        }
                        Err(_) => break,
                    }
                }
                // Also covers listener failure and acceptance racing shutdown.
                if let Ok(sockets) = active.lock() {
                    for stream in sockets.values() {
                        let _closed = stream.shutdown(Shutdown::Both);
                    }
                }
                for worker in workers {
                    let _joined = worker.join();
                }
            })
            .map_err(|_| "outbound control thread could not start")?;
        Ok(Self {
            running,
            clients,
            accept: Some(accept),
        })
    }
}

impl Drop for OutboundServer {
    fn drop(&mut self) {
        self.running.store(false, Ordering::Release);
        if let Ok(clients) = self.clients.lock() {
            for stream in clients.values() {
                let _closed = stream.shutdown(Shutdown::Both);
            }
        }
        if let Some(accept) = self.accept.take() {
            let _joined = accept.join();
        }
    }
}

fn serve_connection(
    mut stream: UnixStream,
    endpoint: Endpoint,
    runtime: tokio::runtime::Handle,
    seed: [u8; 32],
    manager: Arc<TransportManager>,
    holds: Option<ServingHolds>,
) {
    let mut reader_hold: Option<ServingHold> = None;
    let mut plane: Option<IrohMirrorPlane> = None;
    let mut readers = std::collections::BTreeSet::new();
    let mut identity = None;
    while let Ok(request) = read_message::<Request>(&mut stream, true) {
        let configuration = request.configuration;
        let mut operation_started = false;
        let answer = (|| {
            if configuration.key != *endpoint.id().as_bytes() || endpoint.is_closed() {
                return Err("shared endpoint does not match the current device key".into());
            }
            let namespace =
                cerulion_core::ix_config_shm_identity_from_json(&configuration.namespace)
                    .map_err(|_| "outbound namespace is invalid")?;
            if namespace != manager.iox_shm_identity() {
                return Err(
                    "outbound mirror namespace does not match the serving namespace".into(),
                );
            }
            let robots = configuration.robots()?;
            let owner = configuration.owner_certificate()?;
            let configured_identity = (configuration.account, configuration.epoch_dir.clone());
            let retired = if let Some(existing) = &plane {
                if identity.as_ref() != Some(&configured_identity) {
                    return Err("outbound account changed; release the previous connection".into());
                }
                let retired = existing.update_membership(robots)?;
                existing.registry_owner(owner)?;
                retired
            } else {
                let registry = Arc::new(
                    WanRegistry::new(robots, seed, RelayConfig::Disabled)
                        .with_account(configuration.account)
                        .with_epoch_dir(configuration.epoch_dir),
                );
                registry.replace_owner_certificate(owner)?;
                plane = Some(IrohMirrorPlane::on_owned_endpoint(
                    manager.clone(),
                    registry,
                    endpoint.clone(),
                    runtime.clone(),
                )?);
                identity = Some(configured_identity);
                Vec::new()
            };
            let mirror = plane.as_ref().ok_or("outbound plane was not constructed")?;
            for key in &retired {
                readers.remove(key);
            }
            if readers.is_empty() {
                reader_hold.take();
            }
            operation_started = true;
            match request.operation {
                Operation::Ensure {
                    robot,
                    topic,
                    schema_hash,
                    observer,
                } => {
                    let admission = if reader_hold.is_none() {
                        holds
                            .as_ref()
                            .map(|holds| {
                                holds.acquire(HoldReason::Session).map_err(|_| {
                                    "owned remote endpoint is stopping; retry outbound demand"
                                        .to_owned()
                                })
                            })
                            .transpose()?
                    } else {
                        None
                    };
                    let key = topic_key(robot, topic, observer);
                    mirror
                        .ensure_mirror(&key, schema_hash)
                        .map_err(|error| error.to_string())?;
                    readers.insert(key);
                    if reader_hold.is_none() {
                        reader_hold = admission;
                    }
                    Ok(Answer::Done)
                }
                Operation::Release {
                    robot,
                    topic,
                    observer,
                } => {
                    let key = topic_key(robot, topic, observer);
                    let released = mirror.release_mirror(&key);
                    if released == MirrorRelease::Retired {
                        readers.remove(&key);
                        if readers.is_empty() {
                            reader_hold.take();
                        }
                    }
                    Ok(Answer::Released(released == MirrorRelease::Retired))
                }
                Operation::HasReader {
                    robot,
                    topic,
                    observer,
                } => {
                    let key = topic_key(robot, topic, observer);
                    let present = mirror.has_reader(&key)?;
                    if !present {
                        readers.remove(&key);
                        if readers.is_empty() {
                            reader_hold.take();
                        }
                    }
                    Ok(Answer::Present(present))
                }
                Operation::Retire { robot } => {
                    mirror.retire_robot(&robot)?;
                    readers.retain(|key| key.robot != robot);
                    if readers.is_empty() {
                        reader_hold.take();
                    }
                    Ok(Answer::Done)
                }
                Operation::Probe { robot, budget_ms } => mirror
                    .probe_robot(&robot, Duration::from_millis(budget_ms.min(15_000)))
                    .map(|outcome| {
                        Answer::Probe(match outcome {
                            RobotProbeOutcome::Online => None,
                            RobotProbeOutcome::NotReached { reason } => Some(reason),
                        })
                    }),
                Operation::Catalog { robot, budget_ms } => mirror
                    .query_catalog_with_path(&robot, Duration::from_millis(budget_ms.min(15_000)))
                    .map(|(catalog, path, direct)| Answer::Catalog(catalog, path, direct)),
                Operation::Schema {
                    robot,
                    topic,
                    budget_ms,
                } => mirror
                    .query_schema(&robot, &topic, Duration::from_millis(budget_ms.min(15_000)))
                    .map(Answer::Schema),
                Operation::Sync => {
                    for key in &retired {
                        readers.remove(key);
                    }
                    if readers.is_empty() {
                        reader_hold.take();
                    }
                    Ok(Answer::Retired(
                        retired
                            .into_iter()
                            .map(|key| (key.robot, key.topic, key.observer))
                            .collect(),
                    ))
                }
                Operation::Invalidate => {
                    mirror.invalidate_account()?;
                    readers.clear();
                    reader_hold.take();
                    Ok(Answer::Done)
                }
            }
        })();
        // Refused operations keep the admitted connection and every other
        // reader alive. Only validation or framing failures retire its plane.
        if write_message(&mut stream, &answer).is_err() || !operation_started {
            break;
        }
    }
    if let Some(plane) = plane {
        let _retired = plane.invalidate_account();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stalled_shared_ensure_returns_at_the_callers_absolute_deadline() {
        let directory = tempfile::tempdir().unwrap();
        let socket = directory.path().join("deadline.sock");
        let listener = std::os::unix::net::UnixListener::bind(&socket).unwrap();
        let worker = std::thread::spawn(move || {
            let (mut stream, _) = listener.accept().unwrap();
            let request = read_message::<Request>(&mut stream, false).unwrap();
            assert!(matches!(request.operation, Operation::Ensure { .. }));
            std::thread::sleep(Duration::from_millis(350));
        });
        let manager = TransportManager::init_for_test(
            Default::default(),
            cerulion_core::testing::iceoryx_test_config(),
        )
        .unwrap();
        let control = SharedMirrorPlane::new(
            socket,
            manager,
            Arc::new(WanRegistry::new(
                HashMap::new(),
                [17; 32],
                RelayConfig::Disabled,
            )),
        );
        let started = Instant::now();
        let result = control.ensure_until(
            &TopicKey::new("robot", "/stalled"),
            11,
            started + Duration::from_millis(70),
        );
        let elapsed = started.elapsed();
        worker.join().unwrap();
        assert!(result.is_err());
        assert!(
            elapsed < Duration::from_millis(250),
            "stalled Ensure took {elapsed:?}"
        );
        assert!(control.take_transport_loss());
    }

    #[test]
    fn fragmented_sync_reply_cannot_renew_the_absolute_deadline() {
        let directory = tempfile::tempdir().unwrap();
        let socket = directory.path().join("fragmented.sock");
        let listener = std::os::unix::net::UnixListener::bind(&socket).unwrap();
        let worker = std::thread::spawn(move || {
            let (mut stream, _) = listener.accept().unwrap();
            let request = read_message::<Request>(&mut stream, false).unwrap();
            assert!(matches!(request.operation, Operation::Sync));
            let body = b"{\"Ok\":{\"Retired\":[]}}";
            assert_eq!(body.len(), 21);
            for byte in 21u32.to_le_bytes().into_iter().chain(body.iter().copied()) {
                if stream.write_all(&[byte]).is_err() {
                    break;
                }
                std::thread::sleep(Duration::from_millis(20));
            }
        });
        let manager = TransportManager::init_for_test(
            Default::default(),
            cerulion_core::testing::iceoryx_test_config(),
        )
        .unwrap();
        let control = SharedMirrorPlane::new(
            socket,
            manager,
            Arc::new(WanRegistry::new(
                HashMap::new(),
                [18; 32],
                RelayConfig::Disabled,
            )),
        );
        let started = Instant::now();
        assert!(control
            .sync_until(started + Duration::from_millis(70))
            .is_err());
        let elapsed = started.elapsed();
        worker.join().unwrap();
        assert!(
            elapsed < Duration::from_millis(250),
            "fragmented Sync took {elapsed:?}"
        );
        assert!(control.take_transport_loss());
    }

    #[test]
    fn an_expired_deadline_opens_no_connection_and_fences_no_reader() {
        let directory = tempfile::tempdir().unwrap();
        let socket = directory.path().join("expired.sock");
        let listener = std::os::unix::net::UnixListener::bind(&socket).unwrap();
        listener.set_nonblocking(true).unwrap();
        let manager = TransportManager::init_for_test(
            Default::default(),
            cerulion_core::testing::iceoryx_test_config(),
        )
        .unwrap();
        let control = SharedMirrorPlane::new(
            socket,
            manager,
            Arc::new(WanRegistry::new(
                HashMap::new(),
                [19; 32],
                RelayConfig::Disabled,
            )),
        );
        assert!(control
            .ensure_until(&TopicKey::new("robot", "/expired"), 11, Instant::now())
            .unwrap_err()
            .contains("deadline expired"));
        assert_eq!(
            listener.accept().unwrap_err().kind(),
            std::io::ErrorKind::WouldBlock
        );
        assert!(!control.take_transport_loss());
    }

    #[test]
    fn malformed_owner_postcard_is_refused_before_operation_admission() {
        for owner in [vec![0], vec![1], vec![255, 1]] {
            let configuration = Configuration {
                key: [1; 32],
                account: Some([2; 32]),
                epoch_dir: None,
                owner: Some(owner),
                namespace: String::new(),
                peers: Vec::new(),
            };
            assert!(configuration.owner_certificate().is_err());
        }
    }

    #[test]
    fn malformed_and_duplicate_membership_is_refused() {
        let key = DeviceIdentity::from_seed(&[4; 32]).public_key().0;
        for names in [vec![""], vec![" robot"], vec!["robot", "robot"]] {
            let configuration = Configuration {
                key,
                account: None,
                epoch_dir: None,
                owner: None,
                namespace: String::new(),
                peers: names
                    .into_iter()
                    .map(|name| Peer {
                        name: name.into(),
                        key,
                        addresses: Vec::new(),
                    })
                    .collect(),
            };
            assert!(configuration.robots().is_err());
        }
        let configuration = Configuration {
            key,
            account: None,
            epoch_dir: None,
            owner: None,
            namespace: String::new(),
            peers: vec![Peer {
                name: "robot".into(),
                key,
                addresses: Vec::new(),
            }],
        };
        assert_eq!(configuration.robots().unwrap().len(), 1);
    }

    #[test]
    fn untrusted_frame_lengths_and_truncated_messages_fail_closed() {
        for size in [0u32, MAX_MESSAGE as u32 + 1, u32::MAX] {
            let (mut reader, mut writer) = UnixStream::pair().unwrap();
            writer.write_all(&size.to_le_bytes()).unwrap();
            assert_eq!(
                read_message::<Request>(&mut reader, false).err().unwrap(),
                "outbound control message exceeds its byte limit"
            );
        }
        let (mut reader, mut writer) = UnixStream::pair().unwrap();
        writer.write_all(&10u32.to_le_bytes()).unwrap();
        writer.write_all(b"{}").unwrap();
        drop(writer);
        assert_eq!(
            read_message::<Request>(&mut reader, false).err().unwrap(),
            "outbound control message was truncated"
        );
    }
}
