// SPDX-License-Identifier: AGPL-3.0-only
//! The port names of a topic's publishers, by publisher id.
//!
//! An iceoryx2 publisher port can carry a [`PortName`] of up to
//! [`max_port_name_len`] bytes. The data service's dynamic config lists every
//! attached publisher, in every process, as its `UniquePublisherId` beside that
//! name, and a received sample carries the id of the port that committed it
//! ([`crate::transport::subscriber::ReceivedMessage::origin`],
//! [`crate::transport::subscriber::OwnedInboundSample::origin`]). A
//! [`PublisherDirectory`] joins the two: given a sample's origin it returns the
//! name its publisher was created under
//! ([`crate::transport::TransportManager::create_named_publisher_with_topic_config`]).

use iceoryx2::port::port_name::PortName;
use iceoryx2::prelude::{CallbackProgression, PortFactory as _};

use super::CerService;

/// The longest iceoryx2 port name, in bytes.
pub fn max_port_name_len() -> usize {
    PortName::max_len()
}

/// `name` as an iceoryx2 [`PortName`], or the reason it is not one (empty, or
/// longer than [`max_port_name_len`] bytes).
pub(crate) fn port_name(name: &str) -> Result<PortName, String> {
    if name.is_empty() {
        return Err("an empty port name is not a valid iceoryx2 port name".to_string());
    }
    PortName::new(name).map_err(|e| {
        format!(
            "'{name}' is not a valid iceoryx2 port name ({} bytes, at most {}): {e:?}",
            name.len(),
            max_port_name_len()
        )
    })
}

/// A handle on one topic's data service that reads its publishers' port names
/// ([`crate::transport::TransportManager::open_publisher_directory`]).
///
/// Each lookup walks the service's live publisher list in shared memory.
/// iceoryx2's `list_publishers` copies the list's state into two vectors
/// sized to the service's publisher maximum, so a lookup allocates: callers
/// cache its answers per origin. A publisher that has detached is no longer
/// listed, so a sample still queued from it resolves to `None`.
pub struct PublisherDirectory {
    data_service: iceoryx2::service::port_factory::publish_subscribe::PortFactory<CerService, [u8], ()>,
}

impl std::fmt::Debug for PublisherDirectory {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PublisherDirectory")
            .field("service", &self.data_service.name().as_str())
            .finish()
    }
}

impl PublisherDirectory {
    pub(crate) fn new(
        data_service: iceoryx2::service::port_factory::publish_subscribe::PortFactory<
            CerService,
            [u8],
            (),
        >,
    ) -> Self {
        Self { data_service }
    }

    /// The most publishers the data service admits at once
    /// (`max_publishers` in its static config, fixed when it was created).
    pub fn max_publishers(&self) -> usize {
        self.data_service.static_config().max_publishers()
    }

    /// Calls `f` with the port name of the attached publisher whose publisher
    /// id is `origin` (the raw `u128` a sample's origin carries) and returns its
    /// result, or `None` when no attached publisher has that id. A port created
    /// without a name reports `""`.
    pub fn with_publisher_name<R>(&self, origin: u128, f: impl FnOnce(&str) -> R) -> Option<R> {
        let mut f = Some(f);
        let mut out = None;
        self.data_service
            .dynamic_config()
            .list_publishers(|details| {
                if details.publisher_id.value() == origin {
                    if let Some(f) = f.take() {
                        out = Some(f(details.publisher_name.as_str()));
                    }
                    return CallbackProgression::Stop;
                }
                CallbackProgression::Continue
            });
        out
    }
}
