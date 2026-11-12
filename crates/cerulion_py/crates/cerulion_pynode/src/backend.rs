//! Local backend selection beneath the shared Python tick and buffer protocol.
use crate::FrameBacking;
use cerulion_core::clock::Clock;
use cerulion_core::graph::node::{AnyPublisher, AnySubscriber, NodeContext as NativeContext};
use cerulion_core::graph::node_plugin_abi as plugin;
use cerulion_core::node_abi::LoanRequestV1;
use cerulion_core::transport::publisher::RawShmLoan;
use indexmap::IndexMap;
use std::sync::Arc;

pub(crate) enum Context {
    Native(Box<NativeContext>),
    Plugin(plugin::NodeContext, plugin::HostClient),
}
impl Context {
    pub(crate) fn clock(&self) -> &Arc<dyn Clock> {
        match self {
            Self::Native(c) => c.clock(),
            Self::Plugin(c, _) => c.clock(),
        }
    }
    pub(crate) fn env_str(&self, key: &str, default: &str) -> String {
        match self {
            Self::Native(c) => c.env_str(key, default),
            Self::Plugin(c, _) => c.env_str(key, default),
        }
    }
    pub(crate) fn env_opt(&self, key: &str) -> Option<String> {
        match self {
            Self::Native(c) => c.env_opt(key),
            Self::Plugin(c, _) => c.env_opt(key),
        }
    }
    pub(crate) fn request_shutdown(&self) {
        match self {
            Self::Native(c) => c.request_shutdown(),
            Self::Plugin(c, _) => c.request_shutdown(),
        }
    }
    pub(crate) fn split(&mut self) -> (Publishers<'_>, Subscribers<'_>) {
        match self {
            Self::Native(c) => {
                let (p, s) = c.split_publishers_subscribers_mut();
                (Publishers::Native(p), Subscribers::Native(s))
            }
            Self::Plugin(c, client) => {
                let (p, s) = c.split_publishers_subscribers_mut();
                (
                    Publishers::Plugin(p, client),
                    Subscribers::Plugin(s, client),
                )
            }
        }
    }
}

pub(crate) enum Publishers<'a> {
    Native(&'a mut IndexMap<String, AnyPublisher>),
    Plugin(
        &'a mut IndexMap<String, plugin::AnyPublisher>,
        &'a plugin::HostClient,
    ),
}
impl Publishers<'_> {
    pub(crate) fn pending_failure(&self) -> bool {
        matches!(self, Self::Plugin(_, client) if client.has_failure())
    }
    pub(crate) fn contains(&self, name: &str) -> bool {
        match self {
            Self::Native(p) => p.contains_key(name),
            Self::Plugin(p, _) => p.contains_key(name),
        }
    }
    pub(crate) fn loan(
        &mut self,
        name: &str,
        total: usize,
        request: LoanRequestV1,
    ) -> Result<Loan, String> {
        match self {
            Self::Native(p) => {
                let Some(AnyPublisher::Ipc(p)) = p.get_mut(name) else {
                    return Err(format!("unknown output '{name}'"));
                };
                let mut loan = p.loan_raw_uninit(total).map_err(|e| e.to_string())?;
                for byte in loan.bytes_uninit_mut() {
                    byte.write(0);
                }
                // SAFETY: every byte of this exact-size raw loan was initialized.
                Ok(Loan::Native(unsafe { loan.assume_init() }))
            }
            Self::Plugin(p, client) => p
                .get_mut(name)
                .ok_or_else(|| format!("unknown output '{name}'"))?
                .loan_raw_frame(request, total)
                .map(Loan::Plugin)
                .map_err(|e| preserve_error(client, e)),
        }
    }
    pub(crate) fn initial_sequence(&self, name: &str) -> u32 {
        match self {
            Self::Native(p) => match p.get(name) {
                Some(AnyPublisher::Ipc(p)) => p.initial_sequence(),
                None => 0,
            },
            Self::Plugin(_, _) => 0, // Stable sequence ownership remains exclusively on the host.
        }
    }
    pub(crate) fn commit(
        &mut self,
        name: &str,
        loan: Loan,
        committed: &mut usize,
    ) -> Result<(), String> {
        match (self, loan) {
            (Self::Native(p), Loan::Native(loan)) => {
                let Some(AnyPublisher::Ipc(p)) = p.get_mut(name) else {
                    return Err(format!("unknown output '{name}'"));
                };
                p.send_raw_loan(loan).map_err(|e| e.to_string())?;
                *committed += 1;
                p.check_subscriber_events();
                p.notify_sent_sample()
                    .map(|_| ())
                    .map_err(|e| e.to_string())
            }
            (Self::Plugin(_, client), Loan::Plugin(loan)) => {
                loan.commit().map_err(|e| preserve_error(client, e))?;
                *committed += 1;
                Ok(())
            }
            _ => Err("output loan backend does not match its publisher".into()),
        }
    }
}

pub(crate) enum Subscribers<'a> {
    Native(&'a mut IndexMap<String, AnySubscriber>),
    Plugin(
        &'a mut IndexMap<String, plugin::AnySubscriber>,
        &'a plugin::HostClient,
    ),
}
impl Subscribers<'_> {
    pub(crate) fn read(
        &mut self,
        name: &str,
        hash: u64,
        copies: &mut std::collections::HashMap<String, std::rc::Rc<[u8]>>,
    ) -> Result<Option<FrameBacking>, String> {
        match self {
            Self::Native(s) => {
                let Some(s) = s.get_mut(name) else {
                    return Ok(None);
                };
                let Some(view) = s.view_raw_expecting(hash).map_err(|e| e.to_string())? else {
                    return Ok(None);
                };
                Ok(Some(match view.into_owned() {
                    Ok(view) => FrameBacking::Sample(view),
                    Err(held) => {
                        let copy = match copies.get(name).filter(|copy| copy.as_ref() == &held[..])
                        {
                            Some(copy) => std::rc::Rc::clone(copy),
                            None => {
                                // hot-path-alloc-ok: one copy per distinct native held frame.
                                let copy: std::rc::Rc<[u8]> = std::rc::Rc::from(&held[..]);
                                copies.insert(name.to_owned(), std::rc::Rc::clone(&copy));
                                copy
                            }
                        };
                        FrameBacking::Copied(copy)
                    }
                }))
            }
            Self::Plugin(s, client) => {
                let Some(s) = s.get_mut(name) else {
                    return Ok(None);
                };
                s.read_raw_frame(hash)
                    .map(|f| f.map(FrameBacking::Plugin))
                    .map_err(|e| preserve_error(client, e))
            }
        }
    }
}

pub(crate) enum Loan {
    Native(RawShmLoan),
    Plugin(plugin::RawFrameLoan),
}
impl Loan {
    pub(crate) fn bytes_mut(&mut self) -> &mut [u8] {
        match self {
            Self::Native(l) => l.bytes_mut(),
            Self::Plugin(l) => l.bytes_mut(),
        }
    }
    pub(crate) fn host_stamps(&self) -> bool {
        matches!(self, Self::Plugin(_))
    }
}

fn preserve_error(
    client: &plugin::HostClient,
    error: cerulion_core::error::TransportError,
) -> String {
    let message = plugin::diagnostic_reason(&error).into_owned();
    client.record_failure(error);
    message
}
