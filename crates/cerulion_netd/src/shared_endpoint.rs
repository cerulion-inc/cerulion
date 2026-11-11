// SPDX-License-Identifier: AGPL-3.0-only
//! Bounded same-user outbound mirrors on the remote server's owned endpoint.

use std::collections::HashMap;
use std::io::{Read, Write};
use std::net::{Shutdown, SocketAddr};
use std::os::unix::net::UnixStream;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use cerulion_core::transport::cerulion_q::{CatalogReply, SchemaReply};
use cerulion_core::TransportManager;
use cerulion_link::{Endpoint, EndpointId, RelayConfig};
use cerulion_pairing::client::DeviceIdentity;
use cerulion_pairing::verify::OwnerCertificatePresentationWire;
use serde::{Deserialize, Serialize};

use crate::iroh_plane::{IrohMirrorPlane, RobotProbeOutcome};
use crate::mirror::{MirrorPlane, MirrorRelease};
use crate::registry::TopicKey;
use crate::wan::{WanRegistry, WanRobot};

const MAX_MESSAGE: usize = 256 * 1024;
const MAX_CLIENTS: usize = 32;
const MAX_ROBOTS: usize = 256;
const CONTROL_TIMEOUT: Duration = Duration::from_secs(20);

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
    owner: Option<OwnerCertificatePresentationWire>,
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
    },
    Release {
        robot: String,
        topic: String,
    },
    HasReader {
        robot: String,
        topic: String,
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
    Catalog(CatalogReply),
    Schema(SchemaReply),
    Released(bool),
    Retired(Vec<(String, String)>),
}

fn write_message(stream: &mut UnixStream, value: &impl Serialize) -> Result<(), String> {
    let bytes = serde_json::to_vec(value).map_err(|_| "outbound control encoding failed")?;
    if bytes.len() > MAX_MESSAGE {
        return Err("outbound control message exceeds its byte limit".into());
    }
    stream
        .write_all(&(bytes.len() as u32).to_le_bytes())
        .and_then(|()| stream.write_all(&bytes))
        .map_err(|_| "outbound control connection write failed".into())
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
                .map(|proof| (*proof).clone()),
            namespace: serde_json::to_string(&self.manager.iox_config())
                .map_err(|_| "outbound namespace encoding failed")?,
            peers,
        })
    }

    fn request(
        stream: &mut UnixStream,
        configuration: Configuration,
        operation: Operation,
    ) -> Result<Answer, String> {
        write_message(
            stream,
            &Request {
                configuration,
                operation,
            },
        )
        .and_then(|()| read_message::<Result<Answer, String>>(stream, false))
        .and_then(|answer| answer)
    }

    fn call(&self, operation: Operation) -> Result<Answer, String> {
        let started = Instant::now();
        let budget = match &operation {
            Operation::Probe { budget_ms, .. }
            | Operation::Catalog { budget_ms, .. }
            | Operation::Schema { budget_ms, .. } => Some(Duration::from_millis(*budget_ms)),
            _ => None,
        };
        let configuration = self.configuration()?;
        // Metadata never waits behind another control operation beyond its
        // caller's budget. The account adapter already retries this busy regime.
        let mut connection = if budget.is_some() {
            self.connection
                .try_lock()
                .map_err(|_| crate::account_access::CONTROLLER_BUSY)?
        } else {
            self.connection
                .lock()
                .map_err(|_| "outbound control state is poisoned")?
        };
        if self.invalidated.load(Ordering::Acquire) {
            return Err("outbound account changed; recreate the mirror plane".into());
        }
        if connection.is_none() {
            let stream = UnixStream::connect(&self.socket)
                .map_err(|_| "owned remote endpoint is not ready for outbound access")?;
            configure_stream(&stream)?;
            *connection = Some(stream);
        }
        let stream = connection
            .as_mut()
            .ok_or("outbound connection was not constructed")?;
        let timeout = budget.map_or(CONTROL_TIMEOUT, |budget| {
            budget.saturating_sub(started.elapsed())
        });
        if timeout.is_zero() {
            return Err("outbound metadata operation exceeded its time budget".into());
        }
        let deadline_setup = stream
            .set_read_timeout(Some(timeout))
            .and_then(|()| stream.set_write_timeout(Some(timeout)));
        if deadline_setup.is_err() {
            // macOS can reject timeout setup on a disconnected socket before
            // the first write. Retire that socket just like any framing error.
            connection.take();
            return Err("outbound control deadline setup failed".into());
        }
        let result = Self::request(stream, configuration, operation);
        if result.is_err() {
            // A framing failure fences the whole connection; never retry an operation
            // whose result may have been committed on the old endpoint.
            connection.take();
        }
        result
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
                Self::request(stream, configuration, Operation::Invalidate).map(|_| ())
            }),
            None => Ok(()),
        };
        if let Some(stream) = connection.take() {
            let _closed = stream.shutdown(Shutdown::Both);
        }
        retired
    }
    pub(crate) fn sync(&self) -> Result<Vec<TopicKey>, String> {
        match self.call(Operation::Sync)? {
            Answer::Retired(keys) => Ok(keys
                .into_iter()
                .map(|(robot, topic)| TopicKey::new(&robot, &topic))
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
        })? {
            Answer::Present(present) => Ok(present),
            _ => Err("outbound reader reply is invalid".into()),
        }
    }
    pub(crate) fn ensure(&self, key: &TopicKey, schema_hash: u64) -> Result<(), String> {
        self.call(Operation::Ensure {
            robot: key.robot.clone(),
            topic: key.topic.clone(),
            schema_hash,
        })
        .map(|_| ())
    }
    pub(crate) fn release(&self, key: &TopicKey) -> MirrorRelease {
        match self.call(Operation::Release {
            robot: key.robot.clone(),
            topic: key.topic.clone(),
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
    pub(crate) fn catalog(&self, robot: &str, timeout: Duration) -> Result<CatalogReply, String> {
        match self.call(Operation::Catalog {
            robot: robot.into(),
            budget_ms: timeout.as_millis().min(15_000) as u64,
        })? {
            Answer::Catalog(catalog) => Ok(catalog),
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
                            workers.push(std::thread::spawn(move || {
                                serve_connection(stream, endpoint, runtime, seed, manager);
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
) {
    let mut plane: Option<IrohMirrorPlane> = None;
    let mut identity = None;
    while let Ok(request) = read_message::<Request>(&mut stream, true) {
        let configuration = request.configuration;
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
            let configured_identity = (configuration.account, configuration.epoch_dir.clone());
            let retired = if let Some(existing) = &plane {
                if identity.as_ref() != Some(&configured_identity) {
                    return Err("outbound account changed; release the previous connection".into());
                }
                let retired = existing.update_membership(robots)?;
                existing.registry_owner(configuration.owner.map(Arc::new))?;
                retired
            } else {
                let registry = Arc::new(
                    WanRegistry::new(robots, seed, RelayConfig::Disabled)
                        .with_account(configuration.account)
                        .with_epoch_dir(configuration.epoch_dir),
                );
                registry.replace_owner_certificate(configuration.owner.map(Arc::new))?;
                plane = Some(
                    IrohMirrorPlane::new(manager.clone(), registry)?
                        .with_endpoint(endpoint.clone(), runtime.clone())?,
                );
                identity = Some(configured_identity);
                Vec::new()
            };
            let mirror = plane.as_ref().ok_or("outbound plane was not constructed")?;
            match request.operation {
                Operation::Ensure {
                    robot,
                    topic,
                    schema_hash,
                } => mirror
                    .ensure_mirror(&TopicKey::new(&robot, &topic), schema_hash)
                    .map(|()| Answer::Done)
                    .map_err(|error| error.to_string()),
                Operation::Release { robot, topic } => Ok(Answer::Released(
                    mirror.release_mirror(&TopicKey::new(&robot, &topic)) == MirrorRelease::Retired,
                )),
                Operation::HasReader { robot, topic } => mirror
                    .has_reader(&TopicKey::new(&robot, &topic))
                    .map(Answer::Present),
                Operation::Retire { robot } => mirror.retire_robot(&robot).map(|()| Answer::Done),
                Operation::Probe { robot, budget_ms } => mirror
                    .probe_robot(&robot, Duration::from_millis(budget_ms.min(15_000)))
                    .map(|outcome| {
                        Answer::Probe(match outcome {
                            RobotProbeOutcome::Online => None,
                            RobotProbeOutcome::NotReached { reason } => Some(reason),
                        })
                    }),
                Operation::Catalog { robot, budget_ms } => mirror
                    .query_catalog(&robot, Duration::from_millis(budget_ms.min(15_000)))
                    .map(Answer::Catalog),
                Operation::Schema {
                    robot,
                    topic,
                    budget_ms,
                } => mirror
                    .query_schema(&robot, &topic, Duration::from_millis(budget_ms.min(15_000)))
                    .map(Answer::Schema),
                Operation::Sync => Ok(Answer::Retired(
                    retired
                        .into_iter()
                        .map(|key| (key.robot, key.topic))
                        .collect(),
                )),
                Operation::Invalidate => mirror.invalidate_account().map(|()| Answer::Done),
            }
        })();
        let failed = answer.is_err();
        if write_message(&mut stream, &answer).is_err() || failed {
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
