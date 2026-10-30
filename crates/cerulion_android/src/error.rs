// SPDX-License-Identifier: AGPL-3.0-only
//! Runtime errors and their stable integer codes.
use thiserror::Error;

/// Stable error codes shared with Java and JNI callers.
pub mod codes {
    /// Operation succeeded.
    pub const OK: i32 = 0;
    /// Runtime facade has not been initialized.
    pub const NOT_INITIALIZED: i32 = 1;
    /// An initialization request conflicts with the existing process runtime.
    pub const ALREADY_INITIALIZED: i32 = 2;
    /// An argument or configuration value is invalid.
    pub const INVALID_ARGUMENT: i32 = 3;
    /// Operation is incompatible with the current runtime state.
    pub const BAD_STATE: i32 = 4;
    /// Graph parsing, loading, or execution failed.
    pub const GRAPH: i32 = 5;
    /// Network setup or gateway execution failed.
    pub const NETWORK: i32 = 6;
    /// Recording setup or execution failed.
    pub const RECORDER: i32 = 7;
    /// Requested publish or subscribe topic is not configured.
    pub const UNKNOWN_TOPIC: i32 = 8;
    /// JSON values could not be encoded.
    pub const ENCODE: i32 = 9;
    /// Transport publisher operation failed.
    pub const PUBLISH: i32 = 10;
    /// Another start or stop is in flight; retry. A retry answered
    /// BAD_STATE means a runtime is already running under another owner.
    pub const BUSY: i32 = 11;
    /// A panic was caught at the embedding boundary.
    pub const PANIC: i32 = 12;
    /// The OS refused to create a worker thread; retry later.
    pub const SPAWN: i32 = 13;
    /// A received frame could not be decoded into JSON; it was consumed.
    pub const DECODE: i32 = 14;
    /// The subscription queue holds no message.
    pub const EMPTY: i32 = 15;
    /// The transport refused to create a subscription or receive from it.
    pub const RECEIVE: i32 = 16;
    /// The remote plane has no keys or trust state under its state directory;
    /// provision them, then start again.
    pub const REMOTE_NOT_PROVISIONED: i32 = 17;
    /// The remote plane could not start or stopped serving: no host is
    /// linked, its endpoint could not bind, or it ended on its own.
    pub const REMOTE: i32 = 18;
}

#[derive(Debug, Error)]
/// Error returned by the embedded runtime.
pub enum EmbedError {
    /// Runtime facade has not been initialized.
    #[error("{0}")]
    NotInitialized(String),
    /// Initialization conflicts with an existing process runtime.
    #[error("{0}")]
    AlreadyInitialized(String),
    /// Argument or configuration validation failed.
    #[error("{0}")]
    InvalidArgument(String),
    /// Operation is incompatible with the current runtime state.
    #[error("{0}")]
    BadState(String),
    /// Graph setup or execution failed.
    #[error("{0}")]
    Graph(String),
    /// Network setup or execution failed.
    #[error("{0}")]
    Network(String),
    /// Recorder setup or execution failed.
    #[error("{0}")]
    Recorder(String),
    /// The requested publish or subscribe topic is not configured.
    #[error("{0}")]
    UnknownTopic(String),
    /// Publish JSON could not be encoded.
    #[error("{0}")]
    Encode(String),
    /// Transport publishing failed.
    #[error("{0}")]
    Publish(String),
    /// A nonblocking operation could not acquire the runtime.
    #[error("{0}")]
    Busy(String),
    /// A panic was caught at the embedding boundary.
    #[error("{0}")]
    Panic(String),
    /// The OS refused to create a worker thread.
    #[error("{0}")]
    Spawn(String),
    /// A received frame could not be decoded into JSON.
    #[error("{0}")]
    Decode(String),
    /// The transport refused to create a subscription or receive from it.
    #[error("{0}")]
    Receive(String),
    /// The remote plane has no keys or trust state to load.
    #[error("{0}")]
    RemoteNotProvisioned(String),
    /// The remote plane could not start or stopped serving.
    #[error("{0}")]
    Remote(String),
}

impl EmbedError {
    /// Return the stable integer code for this error.
    pub fn code(&self) -> i32 {
        match self {
            Self::NotInitialized(_) => codes::NOT_INITIALIZED,
            Self::AlreadyInitialized(_) => codes::ALREADY_INITIALIZED,
            Self::InvalidArgument(_) => codes::INVALID_ARGUMENT,
            Self::BadState(_) => codes::BAD_STATE,
            Self::Graph(_) => codes::GRAPH,
            Self::Network(_) => codes::NETWORK,
            Self::Recorder(_) => codes::RECORDER,
            Self::UnknownTopic(_) => codes::UNKNOWN_TOPIC,
            Self::Encode(_) => codes::ENCODE,
            Self::Publish(_) => codes::PUBLISH,
            Self::Busy(_) => codes::BUSY,
            Self::Panic(_) => codes::PANIC,
            Self::Spawn(_) => codes::SPAWN,
            Self::Decode(_) => codes::DECODE,
            Self::Receive(_) => codes::RECEIVE,
            Self::RemoteNotProvisioned(_) => codes::REMOTE_NOT_PROVISIONED,
            Self::Remote(_) => codes::REMOTE,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn error_codes_are_unique_and_exhaustive() {
        let cases = [
            (EmbedError::NotInitialized(String::new()), 1),
            (EmbedError::AlreadyInitialized(String::new()), 2),
            (EmbedError::InvalidArgument(String::new()), 3),
            (EmbedError::BadState(String::new()), 4),
            (EmbedError::Graph(String::new()), 5),
            (EmbedError::Network(String::new()), 6),
            (EmbedError::Recorder(String::new()), 7),
            (EmbedError::UnknownTopic(String::new()), 8),
            (EmbedError::Encode(String::new()), 9),
            (EmbedError::Publish(String::new()), 10),
            (EmbedError::Busy(String::new()), 11),
            (EmbedError::Panic(String::new()), 12),
            (EmbedError::Spawn(String::new()), 13),
            (EmbedError::Decode(String::new()), 14),
            (EmbedError::Receive(String::new()), 16),
            (EmbedError::RemoteNotProvisioned(String::new()), 17),
            (EmbedError::Remote(String::new()), 18),
        ];
        // EMPTY (15) is a poll outcome, not an error: no variant carries
        // it, and with it the codes run 1 to 18 without a gap.
        let mut actual = cases.iter().map(|(_, code)| *code).collect::<Vec<_>>();
        actual.push(codes::EMPTY);
        actual.sort_unstable();
        assert_eq!(actual, (1..=18).collect::<Vec<_>>());
        for (error, code) in cases {
            assert_eq!(error.code(), code);
        }
        assert_eq!(codes::OK, 0);
        assert_eq!(codes::EMPTY, 15);
    }
}
