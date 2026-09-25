// SPDX-License-Identifier: AGPL-3.0-only
//! Native finalized-bag reading for the Python binding.

use crate::errors::{map_bag_err, BagError};
use cerulion_bag::{BagReader, FrameSpan, RESERVED_PREFIX};
use pyo3::prelude::*;
use pyo3::types::PyBytes;
use std::collections::{HashMap, HashSet};
use std::path::PathBuf;
use std::sync::Arc;

/// A finalized bag reader. Record bytes are copied out of the read-only memory
/// map, and `messages()` builds its 16-byte-per-frame span index up front.
#[pyclass(unsendable, name = "Bag")]
pub struct PyBag {
    reader: Option<Arc<BagReader>>,
}

#[pyclass(unsendable)]
pub struct BagRecordIter {
    reader: Arc<BagReader>,
    spans: std::vec::IntoIter<(String, FrameSpan)>,
}

fn live_reader(reader: &Option<Arc<BagReader>>) -> PyResult<&Arc<BagReader>> {
    reader
        .as_ref()
        .ok_or_else(|| BagError::new_err("bag is closed"))
}

#[pyfunction]
pub fn open_bag(path: PathBuf) -> PyResult<PyBag> {
    let reader = BagReader::open(&path).map_err(map_bag_err)?;
    Ok(PyBag {
        reader: Some(Arc::new(reader)),
    })
}

#[pymethods]
impl PyBag {
    fn topics(&self) -> PyResult<Vec<(String, String, u64, usize)>> {
        let reader = live_reader(&self.reader)?;
        let channels = reader.channels().map_err(map_bag_err)?;
        let mut counts = HashMap::<String, usize>::new();
        let mut walk = reader.user_frames().map_err(map_bag_err)?;
        while let Some((channel_id, _)) = walk.next_user_frame().map_err(map_bag_err)? {
            *counts.entry(walk.topic(channel_id).to_owned()).or_default() += 1;
        }
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
        let reader = live_reader(&self.reader)?;
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
        let mut walk = reader.user_frames().map_err(map_bag_err)?;
        let mut spans = Vec::new();
        while let Some((channel_id, span)) = walk.next_user_frame().map_err(map_bag_err)? {
            let topic = walk.topic(channel_id);
            if filter.as_ref().is_none_or(|names| names.contains(topic)) {
                spans.push((topic.to_string(), span));
            }
        }
        Ok(BagRecordIter {
            reader: Arc::clone(reader),
            spans: spans.into_iter(),
        })
    }

    fn close(&mut self) {
        self.reader = None;
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
        let Some((topic, span)) = self.spans.next() else {
            return Ok(None);
        };
        Ok(Some((topic, PyBytes::new(py, self.reader.frame(&span)))))
    }
}
