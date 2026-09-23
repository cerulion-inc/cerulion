// SPDX-License-Identifier: AGPL-3.0-only
//! Deadline-bound cold control I/O. Progress never renews a request's deadline.
//!
//! Two observed macOS socket behaviors require nonblocking I/O and separate
//! shutdown halves: setting socket timeouts after peer close returns EINVAL
//! even while its complete reply remains readable; SHUT_RDWR after a peer's
//! write-half close returns ENOTCONN without closing our still-open write half.
//! Safe readiness polling avoids timeout setters on the reply path, and closing
//! Write then Read makes the peer observe EOF after a poisoned exchange.

use std::io::{self, BufRead, BufReader, Write};
use std::net::Shutdown;
use std::os::unix::net::UnixStream;
use std::path::PathBuf;
use std::time::Instant;

use super::{
    default_socket_path, validate_hello, verb_compat_error, ClientError, ConnectAttempt,
    NetdClient, ROUNDTRIP_TIMEOUT,
};
use rustix::event::{poll, PollFd, PollFlags, Timespec};

use crate::account_access::{AccountAccessReply, AccountAccessRequest};
use crate::protocol::{Request, Response, ServingSchemaSnapshot, MAX_REQUEST_LINE_BYTES};

const MAX_HELLO_BYTES: usize = 16 * 1024;
const MAX_RESPONSE_BYTES: usize = MAX_REQUEST_LINE_BYTES;

fn remaining(deadline: Instant) -> io::Result<std::time::Duration> {
    deadline
        .checked_duration_since(Instant::now())
        .filter(|duration| !duration.is_zero())
        .ok_or_else(|| io::Error::new(io::ErrorKind::TimedOut, "netd control deadline expired"))
}

/// Read one complete newline-terminated record without accepting partial EOF.
fn read_line(
    reader: &mut BufReader<UnixStream>,
    deadline: Instant,
    cap: usize,
) -> io::Result<String> {
    // On macOS, changing socket timeouts after peer close returns EINVAL even
    // when a complete reply is buffered. Nonblocking I/O can still drain it.
    reader.get_ref().set_nonblocking(true)?;
    // hot-path-alloc-ok: bounded cold control response, never a topic frame
    let mut bytes = Vec::new();
    loop {
        remaining(deadline)?;
        let available = match reader.fill_buf() {
            Ok([]) => {
                return Err(io::Error::new(
                    io::ErrorKind::UnexpectedEof,
                    "netd closed before completing a control line",
                ));
            }
            Ok(bytes) => bytes,
            Err(error) if error.kind() == io::ErrorKind::Interrupted => continue,
            Err(error) if error.kind() == io::ErrorKind::WouldBlock => {
                wait_for_io(reader.get_ref(), PollFlags::IN, deadline)?;
                continue;
            }
            Err(error) => return Err(error),
        };
        remaining(deadline)?;
        let newline = available.iter().position(|byte| *byte == b'\n');
        let count = newline.unwrap_or(available.len());
        if count > cap.saturating_sub(bytes.len()) {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "netd control line exceeds the byte limit",
            ));
        }
        bytes.extend_from_slice(&available[..count]);
        reader.consume(count + usize::from(newline.is_some()));
        if newline.is_some() {
            return String::from_utf8(bytes).map_err(|_| {
                io::Error::new(io::ErrorKind::InvalidData, "netd control line is not UTF-8")
            });
        }
    }
}

fn write_line(stream: &mut UnixStream, line: &str, deadline: Instant) -> io::Result<()> {
    if line.len() > MAX_REQUEST_LINE_BYTES {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "netd control request exceeds the byte limit",
        ));
    }
    stream.set_nonblocking(true)?;
    for mut pending in [line.as_bytes(), b"\n".as_slice()] {
        while !pending.is_empty() {
            remaining(deadline)?;
            match stream.write(pending) {
                Ok(0) => {
                    return Err(io::Error::new(
                        io::ErrorKind::WriteZero,
                        "netd closed before accepting the control request",
                    ));
                }
                Ok(written) => pending = &pending[written..],
                Err(error) if error.kind() == io::ErrorKind::Interrupted => continue,
                Err(error) if error.kind() == io::ErrorKind::WouldBlock => {
                    wait_for_io(stream, PollFlags::OUT, deadline)?;
                    continue;
                }
                Err(error) => return Err(error),
            }
            remaining(deadline)?;
        }
    }
    Ok(())
}

fn wait_for_io(stream: &UnixStream, interest: PollFlags, deadline: Instant) -> io::Result<()> {
    loop {
        let timeout = Timespec::try_from(remaining(deadline)?).map_err(|_| {
            io::Error::new(
                io::ErrorKind::InvalidInput,
                "netd control deadline is too distant",
            )
        })?;
        let mut descriptors = [PollFd::new(stream, interest)];
        match poll(&mut descriptors, Some(&timeout)) {
            Ok(0) => {
                remaining(deadline)?;
            }
            Ok(_) if descriptors[0].revents().contains(PollFlags::NVAL) => {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidInput,
                    "netd control descriptor is invalid",
                ));
            }
            // HUP/ERR also wake the caller: read must drain a final complete
            // record or report EOF, and write must observe the socket error.
            Ok(_) => return Ok(()),
            Err(rustix::io::Errno::INTR) => continue,
            Err(error) => return Err(error.into()),
        }
    }
}

impl NetdClient {
    /// Connect to the existing daemon before an absolute deadline, including Hello.
    /// Never starts a daemon, retries a connect, or creates a worker thread.
    pub fn connect_existing_until(deadline: Instant) -> Result<Self, ClientError> {
        Self::connect_existing_at_until(default_socket_path(), deadline)
    }

    /// Explicit-socket version of [`Self::connect_existing_until`].
    /// Shares the existing Hello marker and version checks. This local protocol
    /// banner is not cryptographic peer authentication.
    pub fn connect_existing_at_until(
        path: PathBuf,
        deadline: Instant,
    ) -> Result<Self, ClientError> {
        let connect = || -> io::Result<UnixStream> {
            remaining(deadline)?;
            let address = socket2::SockAddr::unix(&path)?;
            let socket = socket2::Socket::new(socket2::Domain::UNIX, socket2::Type::STREAM, None)?;
            socket.connect_timeout(&address, remaining(deadline)?)?;
            // Some Unix backlog failure shapes can look writable without a peer.
            // Require a completed connection before accepting a banner.
            socket.peer_addr()?;
            remaining(deadline)?;
            Ok(socket.into())
        };
        let stream = connect().map_err(|source| ClientError::Connect {
            socket: path.clone(),
            attempt: ConnectAttempt::ExistingOnly,
            source,
        })?;
        Self::finish_bounded_handshake(stream, path, deadline)
    }

    /// Read this daemon's local schema metadata within the caller's deadline.
    /// Never discovers peers, starts a daemon, reconnects, or makes a WAN query.
    pub fn query_serving_schema_until(
        &mut self,
        deadline: Instant,
    ) -> Result<ServingSchemaSnapshot, ClientError> {
        self.ensure_usable()?;
        let id = self.next_request_id();
        let request = Request::ServingSchemaSnapshot { id };
        match self.bounded_round_trip(&request, deadline)? {
            Response::ServingSchemaSnapshot(response) if response.id == id => {
                Ok(response.serving_schema)
            }
            Response::Error(error) if error.id == Some(id) => Err(ClientError::Netd {
                error: error.error,
                robot: error.robot,
                topic: error.topic,
            }),
            _ => {
                self.poison();
                Err(ClientError::Protocol(
                    "expected a correlated local schema snapshot response".into(),
                ))
            }
        }
    }

    fn finish_bounded_handshake(
        stream: UnixStream,
        socket: std::path::PathBuf,
        deadline: Instant,
    ) -> Result<Self, ClientError> {
        let reader = BufReader::new(stream.try_clone().map_err(ClientError::Io)?);
        let mut client = Self {
            stream,
            reader,
            next_id: 1,
            socket_path: socket,
            daemon_protocol: 0,
            poisoned: false,
        };
        let result = (|| {
            let line = read_line(&mut client.reader, deadline, MAX_HELLO_BYTES)
                .map_err(ClientError::Io)?;
            client.daemon_protocol = validate_hello(&line)?;
            remaining(deadline).map_err(ClientError::Io)?;
            client
                .stream
                .set_nonblocking(false)
                .map_err(ClientError::Io)?;
            client.restore_timeouts()
        })();
        if let Err(error) = result {
            client.poison();
            return Err(error);
        }
        Ok(client)
    }

    pub(super) fn ensure_usable(&self) -> Result<(), ClientError> {
        if self.poisoned {
            Err(ClientError::Protocol(
                "netd control connection is unusable after a failed exchange; reconnect".into(),
            ))
        } else {
            Ok(())
        }
    }

    pub(super) fn poison(&mut self) {
        self.poisoned = true;
        // Shutdown applies to both descriptors, including the buffered reader.
        // macOS SHUT_RDWR can return ENOTCONN after the peer half-closes while
        // our write half remains open. Shut each half separately so it sees EOF.
        let _ = self.stream.shutdown(Shutdown::Write);
        let _ = self.stream.shutdown(Shutdown::Read);
    }

    fn restore_timeouts(&self) -> Result<(), ClientError> {
        self.stream
            .set_read_timeout(Some(ROUNDTRIP_TIMEOUT))
            .and_then(|()| self.stream.set_write_timeout(Some(ROUNDTRIP_TIMEOUT)))
            .map_err(ClientError::Io)
    }

    /// A bounded exchange on this connection, without reconnecting or spawning.
    pub(super) fn bounded_round_trip(
        &mut self,
        request: &Request,
        deadline: Instant,
    ) -> Result<Response, ClientError> {
        self.ensure_usable()?;
        if let Some(message) = verb_compat_error(self.daemon_protocol, request) {
            return Err(ClientError::Protocol(message));
        }
        let result = (|| {
            write_line(&mut self.stream, &request.to_json_line(), deadline)
                .map_err(ClientError::Io)?;
            let line = read_line(&mut self.reader, deadline, MAX_RESPONSE_BYTES)
                .map_err(ClientError::Io)?;
            let response = serde_json::from_str::<Response>(line.trim())
                .map_err(|_| ClientError::Protocol("unparseable netd control response".into()))?;
            remaining(deadline).map_err(ClientError::Io)?;
            self.stream
                .set_nonblocking(false)
                .map_err(ClientError::Io)?;
            Ok(response)
        })();
        if result.is_err() {
            self.poison();
        }
        result
    }

    /// Execute one account request within a caller-owned absolute deadline.
    /// The same budget covers every partial write and read; never respawns or
    /// retries. This lets a robot listing bound all of its presence probes together.
    pub fn account_access_until(
        &mut self,
        action: AccountAccessRequest,
        deadline: Instant,
    ) -> Result<AccountAccessReply, ClientError> {
        self.ensure_usable()?;
        action.validate().map_err(ClientError::Protocol)?;
        let id = self.next_request_id();
        let request = Request::AccountAccess { id, action };
        match self.bounded_round_trip(&request, deadline)? {
            Response::AccountAccess(response) if response.id == id => {
                let Request::AccountAccess { action, .. } = &request else {
                    unreachable!("constructed account request")
                };
                if action.accepts(&response.account_access) {
                    Ok(response.account_access)
                } else {
                    self.poison();
                    Err(ClientError::Protocol(
                        "account response does not match the requested operation or robot".into(),
                    ))
                }
            }
            Response::Error(error) if error.id == Some(id) => Err(ClientError::Netd {
                error: error.error,
                robot: error.robot,
                topic: error.topic,
            }),
            _ => {
                self.poison();
                Err(ClientError::Protocol(
                    "expected a correlated account access response".into(),
                ))
            }
        }
    }
}

#[cfg(test)]
mod tests;
