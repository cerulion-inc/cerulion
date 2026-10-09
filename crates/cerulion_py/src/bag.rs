// SPDX-License-Identifier: AGPL-3.0-only
//! Native finalized-bag reading for the Python binding.

use crate::errors::{map_bag_err, BagError};
use cerulion_bag::{
    AdviseCursor, BagCompleteness, BagReader, UserFrameWalk, WalkPosition, RESERVED_PREFIX,
};
use pyo3::prelude::*;
use pyo3::types::{PyBytes, PyString};
use std::cell::RefCell;
use std::collections::{HashMap, HashSet};
use std::path::PathBuf;
use std::rc::Rc;

/// The reader a bag and its iterators share; `close()` empties it for all.
type SharedReader = Rc<RefCell<Option<BagReader>>>;

/// A finalized bag reader. Record bytes are copied out of the read-only memory
/// map; `messages()` streams the data section, holding one walk position
/// instead of a span index, so its memory is independent of the record count.
#[pyclass(unsendable, name = "Bag")]
pub struct PyBag {
    reader: SharedReader,
}

#[pyclass(unsendable)]
pub struct BagRecordIter {
    reader: SharedReader,
    /// Topic per user channel, from the summary.
    topics: HashMap<u16, String>,
    /// The channels the topic filter selected; `None` selects every user channel.
    selected: Option<HashSet<u16>>,
    /// Where the walk resumes on the next record; `None` once the data section
    /// is consumed (or the walk failed).
    position: Option<WalkPosition>,
    /// This iterator's own advise-behind cursor: pages behind its walk are
    /// evicted as records are yielded.
    cursor: AdviseCursor,
    /// Set once `__next__` has raised for a closed bag; later calls stop.
    closure_reported: bool,
}

fn closed() -> PyErr {
    BagError::new_err("bag is closed")
}

/// Walk every user frame, evicting mapped pages behind the walk so a full
/// pass over a large bag does not stay resident. The terminal advise covers
/// a reserved or trace tail the walk skipped on its way to the end.
fn walk_user_frames(
    reader: &BagReader,
    mut visit: impl FnMut(&UserFrameWalk<'_>, u16),
) -> PyResult<()> {
    let mut cursor = AdviseCursor::new();
    let mut walk = reader.user_frames().map_err(map_bag_err)?;
    while let Some((channel_id, _)) = walk.next_user_frame().map_err(map_bag_err)? {
        visit(&walk, channel_id);
        reader.advise_evict_behind_scoped(&mut cursor, walk.file_frontier());
    }
    reader.advise_evict_behind_scoped(&mut cursor, walk.file_frontier());
    Ok(())
}

/// Accept `str`, `bytes` or `os.PathLike`. A `bytes` path keeps its exact
/// filesystem bytes: it never round-trips through Unicode (this crate is
/// Unix-only, see `lib.rs`).
fn bag_path(path: &Bound<'_, PyAny>) -> PyResult<PathBuf> {
    let path = path.py().import("os")?.call_method1("fspath", (path,))?;
    if let Ok(bytes) = path.cast::<PyBytes>() {
        use std::os::unix::ffi::OsStrExt;
        return Ok(PathBuf::from(std::ffi::OsStr::from_bytes(bytes.as_bytes())));
    }
    path.cast::<PyString>()?.extract()
}

/// Open a bag, refusing one whose chunk CRCs or framing do not verify, one
/// that was never finalized, and one whose footer or summary does not
/// describe the file (an out-of-range `summary_start`, a chunk index pointing
/// outside the bag), so every later `topics()` or `messages()` call reads a
/// bag that was validated here.
#[pyfunction]
pub fn open_bag(path: &Bound<'_, PyAny>) -> PyResult<PyBag> {
    let reader = BagReader::open(bag_path(path)?).map_err(map_bag_err)?;
    match reader.completeness().map_err(map_bag_err)? {
        BagCompleteness::Finalized => {}
        BagCompleteness::TornTail(e) => return Err(map_bag_err(e)),
        _ => {
            return Err(BagError::new_err(
                "bag is not finalized: the recording ended without its summary",
            ));
        }
    }
    // The completeness gate checks the footer's fingerprint, not its values:
    // starting a walk validates the footer's summary offset, and its position
    // reads the summary and every chunk index.
    reader
        .user_frames()
        .and_then(UserFrameWalk::into_position)
        .map_err(map_bag_err)?;
    Ok(PyBag {
        reader: Rc::new(RefCell::new(Some(reader))),
    })
}

#[pymethods]
impl PyBag {
    /// User channels in bag order with their per-channel counts, read from the
    /// summary's Statistics; a bag without that record is counted by one walk.
    fn topics(&self) -> PyResult<Vec<(String, String, u64, u64)>> {
        let guard = self.reader.borrow();
        let reader = guard.as_ref().ok_or_else(closed)?;
        let channels = reader.channels().map_err(map_bag_err)?;
        let counts: HashMap<u16, u64> =
            match reader.channel_message_counts().map_err(map_bag_err)? {
                Some(stats) => stats.into_iter().collect(),
                None => {
                    let mut counts = HashMap::new();
                    walk_user_frames(reader, |_, channel_id| {
                        *counts.entry(channel_id).or_default() += 1;
                    })?;
                    counts
                }
            };
        let topics = channels
            .into_iter()
            .filter(|channel| !channel.topic.starts_with(RESERVED_PREFIX))
            .map(|channel| {
                (
                    channel.topic,
                    channel.schema_name,
                    channel.descriptor.map(|d| d.schema_hash).unwrap_or(0),
                    counts.get(&channel.id).copied().unwrap_or(0),
                )
            })
            .collect::<Vec<_>>();
        Ok(topics)
    }

    fn messages(&self, topics: Option<Vec<String>>) -> PyResult<BagRecordIter> {
        let guard = self.reader.borrow();
        let reader = guard.as_ref().ok_or_else(closed)?;
        let user_channels: HashMap<u16, String> = reader
            .channels()
            .map_err(map_bag_err)?
            .into_iter()
            .filter(|channel| !channel.topic.starts_with(RESERVED_PREFIX))
            .map(|channel| (channel.id, channel.topic))
            .collect();
        let selected = topics
            .map(|names| {
                let mut selected = HashSet::new();
                for name in names {
                    let mut ids = user_channels
                        .iter()
                        .filter(|(_, topic)| **topic == name)
                        .map(|(id, _)| *id)
                        .peekable();
                    if ids.peek().is_none() {
                        return Err(pyo3::exceptions::PyValueError::new_err(format!(
                            "unknown topic {name:?}"
                        )));
                    }
                    selected.extend(ids);
                }
                Ok(selected)
            })
            .transpose()?;
        let position = reader
            .user_frames()
            .and_then(UserFrameWalk::into_position)
            .map_err(map_bag_err)?;
        Ok(BagRecordIter {
            reader: Rc::clone(&self.reader),
            topics: user_channels,
            selected,
            position: Some(position),
            cursor: AdviseCursor::new(),
            closure_reported: false,
        })
    }

    /// Unmap the bag. Iterators from `messages()` raise `BagError` afterwards;
    /// records already yielded stay valid.
    fn close(&mut self) {
        self.reader.borrow_mut().take();
    }
}

#[pymethods]
impl BagRecordIter {
    fn __iter__(slf: Py<Self>) -> Py<Self> {
        slf
    }

    /// Resume the walk, copy the next selected record out of the map, suspend
    /// the walk again and evict the pages behind it. The copy happens before
    /// the advise, so a yielded record never depends on a mapped page.
    fn __next__<'py>(
        &mut self,
        py: Python<'py>,
    ) -> PyResult<Option<(String, Bound<'py, PyBytes>)>> {
        if self.closure_reported {
            return Ok(None);
        }
        let guard = self.reader.borrow();
        let Some(reader) = guard.as_ref() else {
            self.position = None;
            self.topics = HashMap::new();
            self.closure_reported = true;
            return Err(closed());
        };
        let Some(position) = self.position.take() else {
            return Ok(None);
        };
        let mut walk = reader.resume_user_frames(position).map_err(map_bag_err)?;
        loop {
            let Some((channel_id, span)) = walk.next_user_frame().map_err(map_bag_err)? else {
                reader.advise_evict_behind_scoped(&mut self.cursor, walk.file_frontier());
                return Ok(None);
            };
            if self
                .selected
                .as_ref()
                .is_some_and(|selected| !selected.contains(&channel_id))
            {
                reader.advise_evict_behind_scoped(&mut self.cursor, walk.file_frontier());
                continue;
            }
            let record = PyBytes::new(py, reader.frame(&span));
            reader.advise_evict_behind_scoped(&mut self.cursor, walk.file_frontier());
            let topic = self.topics.get(&channel_id).cloned().unwrap_or_default();
            self.position = Some(walk.into_position().map_err(map_bag_err)?);
            return Ok(Some((topic, record)));
        }
    }
}
