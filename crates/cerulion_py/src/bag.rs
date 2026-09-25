// SPDX-License-Identifier: AGPL-3.0-only
//! Native finalized-bag reading for the Python binding.

use crate::errors::{map_bag_err, BagError};
use cerulion_bag::{
    AdviseCursor, BagCompleteness, BagReader, FrameSpan, UserFrameWalk, RESERVED_PREFIX,
};
use pyo3::prelude::*;
use pyo3::types::PyBytes;
use std::cell::RefCell;
use std::collections::{HashMap, HashSet};
use std::path::PathBuf;
use std::rc::Rc;

/// The reader a bag and its iterators share; `close()` empties it for all.
type SharedReader = Rc<RefCell<Option<BagReader>>>;

/// A finalized bag reader. Record bytes are copied out of the read-only memory
/// map, and `messages()` builds its 24-byte-per-frame span index up front.
#[pyclass(unsendable, name = "Bag")]
pub struct PyBag {
    reader: SharedReader,
}

#[pyclass(unsendable)]
pub struct BagRecordIter {
    reader: SharedReader,
    topics: HashMap<u16, String>,
    spans: std::vec::IntoIter<(u16, FrameSpan)>,
}

fn closed() -> PyErr {
    BagError::new_err("bag is closed")
}

/// Walk every user frame, evicting mapped pages behind the walk so a full
/// pass over a large bag does not stay resident.
fn walk_user_frames(
    reader: &BagReader,
    mut visit: impl FnMut(&UserFrameWalk<'_>, u16, FrameSpan),
) -> PyResult<()> {
    let mut cursor = AdviseCursor::new();
    let mut walk = reader.user_frames().map_err(map_bag_err)?;
    while let Some((channel_id, span)) = walk.next_user_frame().map_err(map_bag_err)? {
        visit(&walk, channel_id, span);
        reader.advise_evict_behind_scoped(&mut cursor, walk.file_frontier());
    }
    Ok(())
}

/// Open a bag, refusing one whose chunk CRCs or framing do not verify and
/// one that was never finalized.
#[pyfunction]
pub fn open_bag(path: PathBuf) -> PyResult<PyBag> {
    let reader = BagReader::open(&path).map_err(map_bag_err)?;
    match reader.completeness().map_err(map_bag_err)? {
        BagCompleteness::Finalized => {}
        BagCompleteness::TornTail(e) => return Err(map_bag_err(e)),
        _ => {
            return Err(BagError::new_err(
                "bag is not finalized: the recording ended without its summary",
            ));
        }
    }
    Ok(PyBag {
        reader: Rc::new(RefCell::new(Some(reader))),
    })
}

#[pymethods]
impl PyBag {
    fn topics(&self) -> PyResult<Vec<(String, String, u64, usize)>> {
        let guard = self.reader.borrow();
        let reader = guard.as_ref().ok_or_else(closed)?;
        let channels = reader.channels().map_err(map_bag_err)?;
        let mut counts = HashMap::<String, usize>::new();
        walk_user_frames(reader, |walk, channel_id, _| {
            *counts.entry(walk.topic(channel_id).to_owned()).or_default() += 1;
        })?;
        let topics = channels
            .into_iter()
            .filter(|channel| !channel.topic.starts_with(RESERVED_PREFIX))
            .map(|channel| {
                let count = counts.get(&channel.topic).copied().unwrap_or(0);
                (
                    channel.topic,
                    channel.schema_name,
                    channel.descriptor.map(|d| d.schema_hash).unwrap_or(0),
                    count,
                )
            })
            .collect::<Vec<_>>();
        Ok(topics)
    }

    fn messages(&self, topics: Option<Vec<String>>) -> PyResult<BagRecordIter> {
        let guard = self.reader.borrow();
        let reader = guard.as_ref().ok_or_else(closed)?;
        let channels = reader.channels().map_err(map_bag_err)?;
        let user_topics: HashSet<&str> = channels
            .iter()
            .filter(|channel| !channel.topic.starts_with(RESERVED_PREFIX))
            .map(|channel| channel.topic.as_str())
            .collect();
        let filter = topics
            .map(|names| {
                let mut filter = HashSet::with_capacity(names.len());
                for name in names {
                    if !user_topics.contains(name.as_str()) {
                        return Err(pyo3::exceptions::PyValueError::new_err(format!(
                            "unknown topic {name:?}"
                        )));
                    }
                    filter.insert(name);
                }
                Ok(filter)
            })
            .transpose()?;
        let mut topic_names = HashMap::new();
        let mut spans = Vec::new();
        walk_user_frames(reader, |walk, channel_id, span| {
            let topic = walk.topic(channel_id);
            if filter.as_ref().is_none_or(|names| names.contains(topic)) {
                topic_names
                    .entry(channel_id)
                    .or_insert_with(|| topic.to_owned());
                spans.push((channel_id, span));
            }
        })?;
        Ok(BagRecordIter {
            reader: Rc::clone(&self.reader),
            topics: topic_names,
            spans: spans.into_iter(),
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

    fn __next__<'py>(
        &mut self,
        py: Python<'py>,
    ) -> PyResult<Option<(String, Bound<'py, PyBytes>)>> {
        if self.spans.len() == 0 {
            return Ok(None);
        }
        let guard = self.reader.borrow();
        let Some(reader) = guard.as_ref() else {
            self.spans = Vec::new().into_iter();
            self.topics = HashMap::new();
            return Err(closed());
        };
        let Some((channel_id, span)) = self.spans.next() else {
            return Ok(None);
        };
        let topic = self.topics.get(&channel_id).cloned().unwrap_or_default();
        Ok(Some((topic, PyBytes::new(py, reader.frame(&span)))))
    }
}
