// SPDX-License-Identifier: AGPL-3.0-only
//! `cpu.governor`: each CPU cluster's frequency governor, the clock limits in
//! force, the Jetson power mode, and which cluster holds a clock floor (the
//! latency cluster). Reported, never changed.
//!
//! For every `/sys/devices/system/cpu/cpu<N>/cpufreq/` directory it reads
//! `related_cpus` and `affected_cpus` (the CPUs of the cpufreq policy, and the
//! online ones), `scaling_governor`, `scaling_min_freq` and
//! `scaling_max_freq` (the limits in force, which a power mode or a thermal
//! limit narrows inside the hardware range), `scaling_cur_freq` (the clock
//! now), `cpuinfo_min_freq` and `cpuinfo_max_freq` (the hardware range),
//! `scaling_available_frequencies` and, where the driver has it,
//! `amd_pstate_lowest_nonlinear_freq`, all in kHz. CPUs with the same
//! `related_cpus`, or whose `cpufreq` links resolve to the same policy
//! directory, are one cluster, read once. A policy whose CPUs are all offline
//! names no CPU in `affected_cpus`, and newer kernels refuse every read of it
//! with `EBUSY`; either way the cluster reads as offline. On a Jetson it also
//! reads the power mode `nvpmodel` last set, from `/var/lib/nvpmodel/status`,
//! and the modes `/etc/nvpmodel.conf` defines: at boot, the nvpmodel service
//! applies the mode in the status file from that file.
//!
//! A cluster has a clock floor when the power mode holds its minimum clock at
//! its maximum, when its governor is `performance` (the cpufreq governor, or
//! the policy of `intel_pstate` and `amd-pstate-epp`, each of which holds the
//! clock high), or when `scaling_min_freq` is above the minimum the clock
//! starts from. That minimum is, on a Jetson, the minimum the power mode sets
//! (the hardware minimum when no mode sets one), and elsewhere the driver's
//! lowest efficient clock when it reports one, else the hardware minimum;
//! each is raised to the next available frequency, as a kernel that resolves
//! limits to its frequency table stores it. On a Jetson whose power mode in
//! force cannot be read from `/etc/nvpmodel.conf` (or whose nvpmodel files
//! are absent, as in a container that has `/etc/nv_tegra_release` only), a
//! minimum above the boot default's minimum (else the lowest any mode in the
//! file sets, else the hardware minimum when the file defines modes) is a
//! floor from an unknown source. When none of these is known, a minimum at
//! the maximum and above the hardware minimum is one, which a thermal limit
//! that holds both limits at one clock also reads as. The latency cluster is
//! the cluster whose floor holds the highest clock, or every online cluster
//! tied there.
//!
//! The result is `info`, except a `warn` when the mode in the status file is
//! one `/etc/nvpmodel.conf` does not define, which an L4T upgrade that
//! rewrites the file leaves behind: the next boot cannot apply that mode.

// The decision and the readers compile and are tested on every host; only a
// Linux build calls them from `run`.
#![cfg_attr(not(target_os = "linux"), allow(dead_code))]

use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};

use super::{Check, Context, Outcome, Reading, Status};

pub(super) const GOVERNOR: Check = Check {
    id: "cpu.governor",
    why: "the work after each message's wake-up runs at the core's clock: a governor that lets \
          the clock fall between messages, or a power mode that caps it, adds time to every \
          message, and a clock floor on one cluster with both ends of a topic placed there \
          keeps that work at a high clock. Cerulion reports these settings and changes none.",
    network: false,
    run,
};

/// One cpufreq policy (a cluster of CPUs that share a clock) as read; `None`
/// for a file that is absent or empty, or for a clock that is not a number.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub(super) struct Cluster {
    /// The CPUs of the policy that have a `cpufreq` directory.
    pub(super) cpus: BTreeSet<u32>,
    /// Whether a CPU of the policy is online ([`cluster_online`]).
    pub(super) online: bool,
    pub(super) governor: Option<String>,
    /// `scaling_min_freq`.
    pub(super) min_khz: Option<u64>,
    /// `scaling_max_freq`.
    pub(super) max_khz: Option<u64>,
    /// `scaling_cur_freq`.
    pub(super) cur_khz: Option<u64>,
    /// `cpuinfo_min_freq`.
    pub(super) hw_min_khz: Option<u64>,
    /// `cpuinfo_max_freq`.
    pub(super) hw_max_khz: Option<u64>,
    /// `amd_pstate_lowest_nonlinear_freq`, the lowest efficient clock
    /// `amd-pstate` reports, where its `scaling_min_freq` can start.
    pub(super) lowest_nonlinear_khz: Option<u64>,
    /// `scaling_available_frequencies`, ascending; empty for a driver without
    /// a frequency table.
    pub(super) available_khz: Vec<u64>,
}

/// The Jetson power-mode state as read.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub(super) struct NvpModel {
    /// The mode in `/var/lib/nvpmodel/status` (`pmode:0003 fmode:quiet`);
    /// `None` when the file is absent or names no mode.
    pub(super) selected: Option<u32>,
    /// `/etc/nvpmodel.conf` parsed; `None` when it could not be read.
    pub(super) conf: Option<NvpConf>,
}

/// The parts of an nvpmodel configuration file this check uses.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub(super) struct NvpConf {
    /// Every `< POWER_MODEL ID=<n> NAME=<name> >` section, in file order.
    pub(super) modes: Vec<PowerMode>,
    /// `< PM_CONFIG DEFAULT=<n> >`.
    pub(super) default: Option<u32>,
}

impl NvpConf {
    /// The first mode with `id`.
    fn mode(&self, id: u32) -> Option<&PowerMode> {
        self.modes.iter().find(|m| m.id == id)
    }
}

/// One power mode of an nvpmodel configuration file.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub(super) struct PowerMode {
    pub(super) id: u32,
    pub(super) name: String,
    /// The CPU clock limits the mode sets, keyed by the CPU that the clock's
    /// `MIN_FREQ` or `MAX_FREQ` path names.
    pub(super) cpu_limits: BTreeMap<u32, CpuLimit>,
}

/// A CPU clock's `MIN_FREQ` and `MAX_FREQ` in one power mode, in kHz. The file
/// writes `-1` for the largest value, which the kernel clamps to the
/// hardware maximum.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub(super) struct CpuLimit {
    pub(super) min: Option<i64>,
    pub(super) max: Option<i64>,
}

impl CpuLimit {
    /// Whether the mode holds the clock at its maximum: its minimum reaches
    /// its maximum once each is clamped to `hw_max`, the hardware maximum, as
    /// the kernel clamps it, reading `-1` (the largest value) and an absent
    /// maximum as `hw_max`.
    fn pins(self, hw_max: Option<u64>) -> bool {
        let top = hw_max.unwrap_or(u64::MAX);
        let value = |v: i64| {
            if v == -1 {
                Some(top)
            } else {
                u64::try_from(v).ok().map(|v| v.min(top))
            }
        };
        match self.min.and_then(value) {
            Some(min) => self
                .max
                .map_or(Some(top), value)
                .is_some_and(|max| min >= max),
            None => false,
        }
    }

    /// The minimum in kHz, when the mode sets one as a number.
    fn min_khz(self) -> Option<u64> {
        self.min.and_then(|m| u64::try_from(m).ok())
    }
}

/// Everything the check reads from the host.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub(super) struct HostClock {
    /// The clusters, by first CPU.
    pub(super) clusters: Vec<Cluster>,
    /// The power-mode state; `None` on a host that is not L4T ([`read_nvpmodel`]).
    pub(super) nvpmodel: Option<NvpModel>,
}

/// What holds a cluster's clock up.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Floor {
    /// No CPU of the cluster is online.
    Offline,
    /// The clock can fall to the minimum the power mode, the driver or the
    /// hardware starts from.
    Unset,
    /// The power mode sets the minimum equal to the maximum.
    PowerMode,
    /// `scaling_min_freq` is above that starting minimum.
    ScalingMin,
    /// `scaling_min_freq` is raised, and the power mode in force, which would
    /// tell the mode's own floor from one set in `scaling_min_freq`, is not
    /// known.
    Raised,
    /// The governor is `performance`, which holds the clock high.
    Governor,
}

impl Floor {
    /// The word the `cluster_floors` reading carries.
    const fn as_str(self) -> &'static str {
        match self {
            Floor::Offline => "offline",
            Floor::Unset => "none",
            Floor::PowerMode => "nvpmodel",
            Floor::ScalingMin => "scaling_min_freq",
            Floor::Raised => "raised",
            Floor::Governor => "performance",
        }
    }
}

/// The floor of `c`, given `limit`, its CPU clock limits in the baseline power
/// mode ([`baseline_mode`]), `start`, the minimum its clock starts from
/// ([`start_khz`]), and `baseline_known`, whether that baseline is the one in
/// force (always so on a host with no power mode).
fn floor_of(
    c: &Cluster,
    limit: Option<CpuLimit>,
    start: Option<u64>,
    baseline_known: bool,
) -> Floor {
    if !c.online {
        return Floor::Offline;
    }
    let pinned = c.min_khz.is_some() && c.min_khz == c.max_khz;
    if baseline_known && pinned && limit.is_some_and(|l| l.pins(c.hw_max_khz)) {
        return Floor::PowerMode;
    }
    if c.governor.as_deref() == Some("performance") {
        return Floor::Governor;
    }
    let above_start = matches!((c.min_khz, start), (Some(min), Some(start)) if min > start);
    if baseline_known && above_start {
        return Floor::ScalingMin;
    }
    let pinned_above = pinned
        && matches!((c.min_khz, start.or(c.hw_min_khz)), (Some(min), Some(low)) if min > low);
    if !baseline_known && (above_start || pinned_above) {
        return Floor::Raised;
    }
    Floor::Unset
}

/// The minimum `c`'s clock starts from with no floor set, as the kernel
/// stores it ([`as_stored`]). On a Jetson that is the minimum the baseline
/// power mode sets for it, else the lowest minimum any mode in
/// `/etc/nvpmodel.conf` sets for it, else (no mode sets one) the hardware
/// minimum, and unknown when the file was not read or defines no mode.
/// Elsewhere it is the driver's lowest efficient clock when the driver reports
/// one, else the hardware minimum.
fn start_khz(c: &Cluster, nvpmodel: Option<&NvpModel>, mode: Option<&PowerMode>) -> Option<u64> {
    let start = match nvpmodel {
        None => c.lowest_nonlinear_khz.or(c.hw_min_khz),
        Some(n) => {
            let conf = n.conf.as_ref();
            limit_for(c, mode)
                .and_then(CpuLimit::min_khz)
                .or_else(|| {
                    conf?
                        .modes
                        .iter()
                        .filter_map(|m| limit_for(c, Some(m)).and_then(CpuLimit::min_khz))
                        .min()
                })
                .or_else(|| conf.filter(|f| !f.modes.is_empty()).and(c.hw_min_khz))
        }
    };
    start.map(|khz| as_stored(c, khz))
}

/// `khz` as a kernel that resolves limits to its frequency table stores
/// `c`'s minimum: raised to the hardware minimum, then to the first available
/// frequency at or above it. A kernel that does not resolve stores the
/// requested value, which is not above this.
fn as_stored(c: &Cluster, khz: u64) -> u64 {
    let khz = c.hw_min_khz.map_or(khz, |min| khz.max(min));
    c.available_khz
        .iter()
        .copied()
        .find(|f| *f >= khz)
        .unwrap_or(khz)
}

/// The clock `floor` holds `c` at or above, for ranking latency clusters:
/// `None` for no floor, `Some(0)` for a floor whose clock is not read.
fn floor_khz(c: &Cluster, floor: Floor) -> Option<u64> {
    let khz = match floor {
        Floor::PowerMode | Floor::ScalingMin | Floor::Raised => c.min_khz,
        Floor::Governor => c.max_khz.or(c.hw_max_khz),
        Floor::Offline | Floor::Unset => return None,
    };
    Some(khz.unwrap_or(0))
}

/// The power mode floors are measured against, and whether it is the mode in
/// force: the selected mode when `/etc/nvpmodel.conf` defines it (in force);
/// with no mode recorded, the file's boot default (in force, since boot
/// applied it); with a selected mode the file does not define, the boot
/// default (not in force).
fn baseline_mode(n: &NvpModel) -> (Option<&PowerMode>, bool) {
    let Some(conf) = n.conf.as_ref() else {
        return (None, false);
    };
    let default = conf.default.and_then(|id| conf.mode(id));
    match n.selected {
        Some(id) => match conf.mode(id) {
            Some(m) => (Some(m), true),
            None => (default, false),
        },
        None => (default, default.is_some()),
    }
}

/// `c`'s CPU clock limits in `mode`: the limits of the first of its CPUs the
/// mode names.
fn limit_for(c: &Cluster, mode: Option<&PowerMode>) -> Option<CpuLimit> {
    let mode = mode?;
    c.cpus
        .iter()
        .find_map(|cpu| mode.cpu_limits.get(cpu))
        .copied()
}

/// A kHz value as MHz with at most one decimal (`729600` is `729.6`,
/// `1984000` is `1984`).
fn mhz(khz: u64) -> String {
    let (whole, tenth) = (khz / 1000, (khz % 1000) / 100);
    if tenth == 0 {
        whole.to_string()
    } else {
        format!("{whole}.{tenth}")
    }
}

/// `lo` to `hi` in MHz, `?` for an unknown end; `None` when both are unknown.
/// With `collapse`, an equal pair prints once.
fn mhz_range(lo: Option<u64>, hi: Option<u64>, collapse: bool) -> Option<String> {
    let end = |v: Option<u64>| v.map_or_else(|| "?".to_string(), mhz);
    match (lo, hi) {
        (None, None) => None,
        (Some(a), Some(b)) if collapse && a == b => Some(format!("{} MHz", mhz(a))),
        _ => Some(format!("{}-{} MHz", end(lo), end(hi))),
    }
}

/// `cpus` as a Linux CPU list (`0-3,5`).
fn cpu_list(cpus: &BTreeSet<u32>) -> String {
    let mut parts: Vec<String> = Vec::new();
    let mut run: Option<(u32, u32)> = None;
    for &cpu in cpus {
        run = match run {
            Some((start, end)) if end.checked_add(1) == Some(cpu) => Some((start, cpu)),
            Some(done) => {
                parts.push(range_text(done));
                Some((cpu, cpu))
            }
            None => Some((cpu, cpu)),
        };
    }
    if let Some(done) = run {
        parts.push(range_text(done));
    }
    parts.join(",")
}

fn range_text((start, end): (u32, u32)) -> String {
    if start == end {
        start.to_string()
    } else {
        format!("{start}-{end}")
    }
}

/// `CPU 5` or `CPUs 0-3`.
fn cpu_label(cpus: &BTreeSet<u32>) -> String {
    let list = cpu_list(cpus);
    if cpus.len() == 1 {
        format!("CPU {list}")
    } else {
        format!("CPUs {list}")
    }
}

/// One line of the summary: a cluster, or a run of one-CPU clusters with the
/// same settings (a host with one cpufreq policy per CPU).
struct Row<'a> {
    cpus: BTreeSet<u32>,
    cluster: &'a Cluster,
    floor: Floor,
    single: bool,
    cur_lo: Option<u64>,
    cur_hi: Option<u64>,
}

impl Row<'_> {
    /// Whether `c`, with `floor`, prints as this row does.
    fn same_settings(&self, c: &Cluster, floor: Floor) -> bool {
        let a = self.cluster;
        floor == self.floor
            && (
                a.online,
                &a.governor,
                a.min_khz,
                a.max_khz,
                a.hw_min_khz,
                a.hw_max_khz,
            ) == (
                c.online,
                &c.governor,
                c.min_khz,
                c.max_khz,
                c.hw_min_khz,
                c.hw_max_khz,
            )
    }

    fn text(&self) -> String {
        let label = cpu_label(&self.cpus);
        if self.floor == Floor::Offline {
            return format!("{label}: offline");
        }
        let c = self.cluster;
        let mut s = format!(
            "{label}: {}",
            c.governor.as_deref().unwrap_or("governor unknown")
        );
        if let Some(now) = mhz_range(self.cur_lo, self.cur_hi, true) {
            s.push_str(&format!(", now {now}"));
        }
        if let Some(limits) = mhz_range(c.min_khz, c.max_khz, false) {
            s.push_str(&format!(", limits {limits}"));
        }
        if (c.hw_min_khz, c.hw_max_khz) != (c.min_khz, c.max_khz) {
            if let Some(hw) = mhz_range(c.hw_min_khz, c.hw_max_khz, false) {
                s.push_str(&format!(" (hardware {hw})"));
            }
        }
        s.push_str(match self.floor {
            Floor::PowerMode => ", clock floor from the power mode",
            Floor::ScalingMin => ", clock floor from scaling_min_freq",
            Floor::Governor => ", clock floor from the performance governor",
            Floor::Raised => ", clock floor from an unknown source",
            Floor::Unset | Floor::Offline => "",
        });
        s
    }
}

/// The summary's power-mode part, and the selected mode when
/// `/etc/nvpmodel.conf` does not define it.
fn nvpmodel_text(n: &NvpModel) -> (String, Option<u32>) {
    let named = |m: &PowerMode| {
        if m.name.is_empty() {
            format!("mode {}", m.id)
        } else {
            format!("mode {} ({})", m.id, m.name)
        }
    };
    match (n.selected, &n.conf) {
        (Some(id), Some(conf)) if conf.modes.is_empty() => (
            format!("nvpmodel mode {id}, no power mode read from /etc/nvpmodel.conf"),
            None,
        ),
        (Some(id), Some(conf)) => match conf.mode(id) {
            Some(m) => (format!("nvpmodel {}", named(m)), None),
            None => (
                format!(
                    "nvpmodel mode {id} is selected but /etc/nvpmodel.conf does not define it, \
                     so the next boot does not apply it"
                ),
                Some(id),
            ),
        },
        (Some(id), None) => (
            format!("nvpmodel mode {id}, /etc/nvpmodel.conf not read"),
            None,
        ),
        (None, Some(conf)) => match conf.default.and_then(|id| conf.mode(id)) {
            Some(m) => (
                format!("no nvpmodel mode recorded, boot default {}", named(m)),
                None,
            ),
            None => ("no nvpmodel mode recorded".to_string(), None),
        },
        (None, None) => (
            "no nvpmodel mode read, /etc/nvpmodel.conf not read".to_string(),
            None,
        ),
    }
}

/// `values` joined with `;`, `-` for an unknown one.
fn joined(values: impl Iterator<Item = Option<String>>) -> String {
    values
        .map(|v| v.unwrap_or_else(|| "-".to_string()))
        .collect::<Vec<_>>()
        .join(";")
}

/// The report over everything read.
pub(super) fn decide(host: &HostClock) -> Outcome {
    let mut parts: Vec<String> = Vec::new();
    let mut missing_mode = None;
    if let Some(n) = &host.nvpmodel {
        let (text, missing) = nvpmodel_text(n);
        parts.push(text);
        missing_mode = missing;
    }
    let (mode, baseline_known) = host.nvpmodel.as_ref().map_or((None, true), baseline_mode);
    let floors: Vec<Floor> = host
        .clusters
        .iter()
        .map(|c| {
            floor_of(
                c,
                limit_for(c, mode),
                start_khz(c, host.nvpmodel.as_ref(), mode),
                baseline_known,
            )
        })
        .collect();
    let latency = latency(host, &floors);

    if host.clusters.is_empty() {
        parts.push(
            "no cpufreq directory under /sys/devices/system/cpu, so there is no governor to read"
                .to_string(),
        );
    } else {
        parts.extend(rows(host, &floors).iter().map(Row::text));
        parts.push(match &latency {
            Latency::NoFloor => "no clock floor, so no latency cluster".to_string(),
            Latency::Every => {
                "a clock floor on every online cluster, so no single latency cluster".to_string()
            }
            Latency::Cpus(cpus) => format!("latency cluster {}", cpu_label(cpus)),
        });
    }

    let summary = parts.join("; ");
    let o = match missing_mode {
        Some(id) => Outcome::new(Status::Warn, summary).with_fix(format!(
            "put the `< POWER_MODEL ID={id} ... >` section back into /etc/nvpmodel.conf (an L4T \
             upgrade rewrites that file), or select a mode the file defines with `sudo nvpmodel \
             -m <ID>`"
        )),
        None => Outcome::new(Status::Info, summary),
    };
    readings(o, host, &floors, &latency)
}

/// The summary's rows: one per cluster, except that a run of one-CPU clusters
/// with the same settings (a host with one cpufreq policy per CPU) is one row.
fn rows<'a>(host: &'a HostClock, floors: &[Floor]) -> Vec<Row<'a>> {
    let mut rows: Vec<Row> = Vec::new();
    for (c, &floor) in host.clusters.iter().zip(floors) {
        let single = c.cpus.len() == 1;
        let joins = |row: &Row| single && row.single && row.same_settings(c, floor);
        if let Some(row) = rows.last_mut().filter(|row| joins(row)) {
            row.cpus.extend(&c.cpus);
            row.cur_lo = match (row.cur_lo, c.cur_khz) {
                (Some(a), Some(b)) => Some(a.min(b)),
                (a, b) => a.or(b),
            };
            row.cur_hi = row.cur_hi.max(c.cur_khz);
            continue;
        }
        rows.push(Row {
            cpus: c.cpus.clone(),
            cluster: c,
            floor,
            single,
            cur_lo: c.cur_khz,
            cur_hi: c.cur_khz,
        });
    }
    rows
}

/// Which cluster is the latency cluster.
enum Latency {
    /// No online cluster has a clock floor.
    NoFloor,
    /// Every online cluster, when more than one is online, shares the highest
    /// floor.
    Every,
    /// The CPUs of the clusters with the highest floor.
    Cpus(BTreeSet<u32>),
}

/// The latency cluster: the clusters whose floor holds the highest clock.
fn latency(host: &HostClock, floors: &[Floor]) -> Latency {
    let floored: Vec<(&Cluster, u64)> = host
        .clusters
        .iter()
        .zip(floors)
        .filter_map(|(c, &f)| floor_khz(c, f).map(|khz| (c, khz)))
        .collect();
    let Some(top) = floored.iter().map(|(_, khz)| *khz).max() else {
        return Latency::NoFloor;
    };
    let best: Vec<&Cluster> = floored
        .into_iter()
        .filter(|(_, khz)| *khz == top)
        .map(|(c, _)| c)
        .collect();
    let online = floors.iter().filter(|f| **f != Floor::Offline).count();
    if online > 1 && best.len() == online {
        return Latency::Every;
    }
    Latency::Cpus(best.iter().flat_map(|c| c.cpus.iter().copied()).collect())
}

/// The `--json` readings: the host-wide ones and one `;`-joined value per
/// cluster for each per-cluster field, clusters in the order of `clusters`,
/// with `-` for a clock that was not read and `unknown` for a governor that
/// was not.
/// `cpus` counts every CPU with a `cpufreq` directory; `governors`,
/// `min_freq_khz` (the lowest `scaling_min_freq`) and `max_freq_khz` (the
/// highest hardware maximum) cover every cluster, and `scaling_max_freq_khz`
/// (the highest cap in force) the online ones.
fn readings(mut o: Outcome, host: &HostClock, floors: &[Floor], latency: &Latency) -> Outcome {
    let cs = &host.clusters;
    let khz = |v: Option<u64>| v.map(|k| k.to_string());
    let cpus: usize = cs.iter().map(|c| c.cpus.len()).sum();
    o = o.reading(
        "cpus",
        Reading::Int(u64::try_from(cpus).unwrap_or(u64::MAX)),
    );
    if !cs.is_empty() {
        let governors: BTreeSet<&str> = cs
            .iter()
            .map(|c| c.governor.as_deref().unwrap_or("unknown"))
            .collect();
        o = o.reading(
            "governors",
            Reading::Text(governors.into_iter().collect::<Vec<_>>().join(",")),
        );
    }
    if let Some(min) = cs.iter().filter_map(|c| c.min_khz).min() {
        o = o.reading("min_freq_khz", Reading::Int(min));
    }
    if let Some(max) = cs.iter().filter_map(|c| c.hw_max_khz).max() {
        o = o.reading("max_freq_khz", Reading::Int(max));
    }
    if let Some(cap) = cs
        .iter()
        .filter(|c| c.online)
        .filter_map(|c| c.max_khz)
        .max()
    {
        o = o.reading("scaling_max_freq_khz", Reading::Int(cap));
    }
    if !cs.is_empty() {
        let list = |c: &Cluster| Some(cpu_list(&c.cpus));
        o = o
            .reading("clusters", Reading::Text(joined(cs.iter().map(list))))
            .reading(
                "cluster_governors",
                Reading::Text(joined(cs.iter().map(|c| {
                    Some(c.governor.clone().unwrap_or_else(|| "unknown".to_string()))
                }))),
            )
            .reading(
                "cluster_min_khz",
                Reading::Text(joined(cs.iter().map(|c| khz(c.min_khz)))),
            )
            .reading(
                "cluster_max_khz",
                Reading::Text(joined(cs.iter().map(|c| khz(c.max_khz)))),
            )
            .reading(
                "cluster_cur_khz",
                Reading::Text(joined(cs.iter().map(|c| khz(c.cur_khz)))),
            )
            .reading(
                "cluster_hw_min_khz",
                Reading::Text(joined(cs.iter().map(|c| khz(c.hw_min_khz)))),
            )
            .reading(
                "cluster_hw_max_khz",
                Reading::Text(joined(cs.iter().map(|c| khz(c.hw_max_khz)))),
            )
            .reading(
                "cluster_floors",
                Reading::Text(joined(floors.iter().map(|f| Some(f.as_str().to_string())))),
            );
        if let Latency::Cpus(cpus) = latency {
            o = o.reading("latency_cluster", Reading::Text(cpu_list(cpus)));
        }
    }
    if let Some(n) = &host.nvpmodel {
        if let Some(id) = n.selected {
            o = o.reading("nvpmodel_mode", Reading::Int(u64::from(id)));
            if let Some(conf) = n.conf.as_ref().filter(|c| !c.modes.is_empty()) {
                let mode = conf.mode(id);
                o = o.reading("nvpmodel_mode_defined", Reading::Bool(mode.is_some()));
                if let Some(m) = mode {
                    o = o.reading("nvpmodel_mode_name", Reading::Text(m.name.clone()));
                }
            }
        }
    }
    o
}

fn read_trimmed(path: &Path) -> Option<String> {
    std::fs::read_to_string(path)
        .ok()
        .map(|s| s.trim().to_string())
        .filter(|s| !s.is_empty())
}

/// The CPUs in a cpufreq CPU list (`related_cpus`, `affected_cpus`): numbers
/// separated by spaces (`0 1 2 3`). A token that is not a number is skipped.
pub(super) fn parse_cpus(text: &str) -> BTreeSet<u32> {
    text.split_whitespace()
        .filter_map(|t| t.parse().ok())
        .collect()
}

/// Whether a policy is online, from the read of its `affected_cpus`: a list
/// that names a CPU, or a read that failed for another reason than `EBUSY`
/// (an absent file). A policy whose CPUs are all offline names none, and newer
/// kernels refuse every read of it with `EBUSY`.
pub(super) fn cluster_online(affected: std::io::Result<String>) -> bool {
    match affected {
        Ok(s) => !parse_cpus(&s).is_empty(),
        Err(e) => e.kind() != std::io::ErrorKind::ResourceBusy,
    }
}

/// Read every cluster under `cpu_root` (`/sys/devices/system/cpu`), by first
/// CPU. A cluster's CPUs are its `related_cpus`; where that file cannot be
/// read (an offline policy), the CPUs whose `cpufreq` directories resolve to
/// the same policy directory. Each cluster is read once, from its first CPU.
pub(super) fn read_clusters(cpu_root: &Path) -> Vec<Cluster> {
    let Ok(entries) = std::fs::read_dir(cpu_root) else {
        return Vec::new();
    };
    let mut dirs: Vec<(u32, PathBuf)> = entries
        .filter_map(Result::ok)
        .filter_map(|e| {
            let index: u32 = e.file_name().to_str()?.strip_prefix("cpu")?.parse().ok()?;
            let freq = e.path().join("cpufreq");
            freq.is_dir().then_some((index, freq))
        })
        .collect();
    dirs.sort_by_key(|(i, _)| *i);
    let indices: BTreeSet<u32> = dirs.iter().map(|(i, _)| *i).collect();
    let mut seen: BTreeSet<u32> = BTreeSet::new();
    let mut clusters = Vec::new();
    for (index, freq) in &dirs {
        if seen.contains(index) {
            continue;
        }
        let related = read_trimmed(&freq.join("related_cpus")).map(|s| parse_cpus(&s));
        let mut cpus: BTreeSet<u32> = match related {
            Some(related) if related.contains(index) => {
                related.intersection(&indices).copied().collect()
            }
            _ => match std::fs::canonicalize(freq) {
                Ok(target) => dirs
                    .iter()
                    .filter(|(i, other)| {
                        !seen.contains(i)
                            && std::fs::canonicalize(other).ok().as_ref() == Some(&target)
                    })
                    .map(|(i, _)| *i)
                    .collect(),
                Err(_) => BTreeSet::new(),
            },
        };
        cpus.insert(*index);
        seen.extend(&cpus);
        let num = |file: &str| read_trimmed(&freq.join(file)).and_then(|s| s.parse().ok());
        let mut available_khz: Vec<u64> = read_trimmed(&freq.join("scaling_available_frequencies"))
            .map(|s| {
                s.split_whitespace()
                    .filter_map(|f| f.parse().ok())
                    .collect()
            })
            .unwrap_or_default();
        available_khz.sort_unstable();
        clusters.push(Cluster {
            cpus,
            online: cluster_online(std::fs::read_to_string(freq.join("affected_cpus"))),
            governor: read_trimmed(&freq.join("scaling_governor")),
            min_khz: num("scaling_min_freq"),
            max_khz: num("scaling_max_freq"),
            cur_khz: num("scaling_cur_freq"),
            hw_min_khz: num("cpuinfo_min_freq"),
            hw_max_khz: num("cpuinfo_max_freq"),
            lowest_nonlinear_khz: num("amd_pstate_lowest_nonlinear_freq"),
            available_khz,
        });
    }
    clusters
}

/// The mode in an nvpmodel status file (`pmode:0003 fmode:quiet`).
pub(super) fn selected_mode(status: &str) -> Option<u32> {
    status
        .split_whitespace()
        .find_map(|w| w.strip_prefix("pmode:"))
        .and_then(|d| d.parse().ok())
}

/// The CPU a cpufreq path names: `<N>` in `.../cpu/cpu<N>/cpufreq/...` or in
/// `.../cpufreq/policy<N>/...`.
fn cpufreq_cpu(path: &str) -> Option<u32> {
    if !path.contains("/cpufreq") {
        return None;
    }
    let leading = |rest: &str| {
        let digits: String = rest.chars().take_while(char::is_ascii_digit).collect();
        digits.parse().ok()
    };
    path.split_once("/cpufreq/policy")
        .and_then(|(_, rest)| leading(rest))
        .or_else(|| {
            path.split_once("/cpu/cpu")
                .and_then(|(_, rest)| leading(rest))
        })
}

/// Parse an nvpmodel configuration file: the `PARAM TYPE=CLOCK` sections whose
/// `MIN_FREQ` or `MAX_FREQ` path is a CPU's cpufreq file, the
/// `POWER_MODEL` sections and the `PM_CONFIG` default. A line that fits none of
/// these is skipped.
pub(super) fn parse_conf(text: &str) -> NvpConf {
    enum Section {
        Other,
        Clock(String),
        Mode(usize),
    }
    let mut conf = NvpConf::default();
    let mut clock_cpu: BTreeMap<String, u32> = BTreeMap::new();
    let mut settings: Vec<(usize, String, String, i64)> = Vec::new();
    let mut section = Section::Other;
    for line in text.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        if let Some(inner) = line.strip_prefix('<').and_then(|l| l.strip_suffix('>')) {
            let mut words = inner.split_whitespace();
            let kind = words.next().unwrap_or_default();
            let attrs: BTreeMap<&str, &str> = words.filter_map(|w| w.split_once('=')).collect();
            section = match kind {
                "PARAM" if attrs.get("TYPE") == Some(&"CLOCK") => attrs
                    .get("NAME")
                    .map_or(Section::Other, |n| Section::Clock((*n).to_string())),
                "POWER_MODEL" => match attrs.get("ID").and_then(|id| id.parse().ok()) {
                    Some(id) => {
                        conf.modes.push(PowerMode {
                            id,
                            name: attrs.get("NAME").copied().unwrap_or_default().to_string(),
                            cpu_limits: BTreeMap::new(),
                        });
                        Section::Mode(conf.modes.len() - 1)
                    }
                    None => Section::Other,
                },
                "PM_CONFIG" => {
                    if let Some(id) = attrs.get("DEFAULT").and_then(|id| id.parse().ok()) {
                        conf.default = Some(id);
                    }
                    Section::Other
                }
                _ => Section::Other,
            };
            continue;
        }
        let words: Vec<&str> = line.split_whitespace().collect();
        match (&section, words.as_slice()) {
            (Section::Clock(name), [arg, path])
                if matches!(
                    *arg,
                    "MIN_FREQ" | "MAX_FREQ" | "MIN_FREQ_KNEXT" | "MAX_FREQ_KNEXT"
                ) =>
            {
                if let Some(cpu) = cpufreq_cpu(path) {
                    clock_cpu.entry(name.clone()).or_insert(cpu);
                }
            }
            (Section::Mode(index), [param, arg, value]) => {
                if let Ok(v) = value.parse::<i64>() {
                    settings.push((*index, (*param).to_string(), (*arg).to_string(), v));
                }
            }
            _ => {}
        }
    }
    for (index, param, arg, value) in settings {
        let Some(&cpu) = clock_cpu.get(&param) else {
            continue;
        };
        let limit = conf.modes[index].cpu_limits.entry(cpu).or_default();
        match arg.as_str() {
            "MIN_FREQ" => limit.min = Some(value),
            "MAX_FREQ" => limit.max = Some(value),
            _ => {}
        }
    }
    conf
}

/// Read the nvpmodel files under `root`; `None` when none of
/// `var/lib/nvpmodel/status`, `etc/nvpmodel.conf` and `etc/nv_tegra_release`
/// (an L4T system, which runs a power mode even where the nvpmodel files are
/// absent) exists.
pub(super) fn read_nvpmodel(root: &Path) -> Option<NvpModel> {
    let status = root.join("var/lib/nvpmodel/status");
    let conf = root.join("etc/nvpmodel.conf");
    if !status.exists() && !conf.exists() && !root.join("etc/nv_tegra_release").exists() {
        return None;
    }
    Some(NvpModel {
        selected: std::fs::read_to_string(&status)
            .ok()
            .and_then(|s| selected_mode(&s)),
        conf: std::fs::read_to_string(&conf).ok().map(|t| parse_conf(&t)),
    })
}

/// Read the host under `root` (`/` on a live host).
pub(super) fn read_host(root: &Path) -> HostClock {
    HostClock {
        clusters: read_clusters(&root.join("sys/devices/system/cpu")),
        nvpmodel: read_nvpmodel(root),
    }
}

#[cfg(target_os = "linux")]
fn run(_ctx: &Context) -> Outcome {
    decide(&read_host(Path::new("/")))
}

#[cfg(not(target_os = "linux"))]
fn run(_ctx: &Context) -> Outcome {
    Outcome::not_applicable("the governor files are a Linux cpufreq interface")
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A fake root holding the sysfs and nvpmodel files the check reads,
    /// removed on drop.
    struct Tree(PathBuf);

    impl Tree {
        fn new(name: &str) -> Self {
            let root =
                std::env::temp_dir().join(format!("doctor-cpu-{name}-{}", std::process::id()));
            let _ = std::fs::remove_dir_all(&root);
            std::fs::create_dir_all(&root).unwrap();
            Self(root)
        }

        fn file(&self, rel: &str, body: &str) {
            let path = self.0.join(rel);
            std::fs::create_dir_all(path.parent().unwrap()).unwrap();
            std::fs::write(path, body).unwrap();
        }

        /// One cpufreq policy over `cpus`, written into each CPU's `cpufreq`
        /// directory as sysfs shows it, with `related_cpus` and
        /// `affected_cpus` naming every CPU of the policy.
        fn policy(&self, cpus: &[u32], files: &[(&str, &str)]) {
            let list = cpus
                .iter()
                .map(u32::to_string)
                .collect::<Vec<_>>()
                .join(" ");
            for cpu in cpus {
                let dir = format!("sys/devices/system/cpu/cpu{cpu}/cpufreq");
                self.file(&format!("{dir}/related_cpus"), &format!("{list}\n"));
                self.file(&format!("{dir}/affected_cpus"), &format!("{list}\n"));
                for (name, value) in files {
                    self.file(&format!("{dir}/{name}"), &format!("{value}\n"));
                }
            }
        }

        fn read(&self) -> HostClock {
            read_host(&self.0)
        }
    }

    impl Drop for Tree {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    /// The `PARAM` part of a Jetson Orin NX `nvpmodel.conf`: its two CPU
    /// clusters' clocks, the CPU online switches and one non-CPU clock, in
    /// that file's layout (including its header written without the space
    /// after `<`).
    const ORIN_PARAMS: &str = "\
# FORMAT:
# < PARAM TYPE=PARAM_TYPE NAME=PARAM_NAME >
< PARAM TYPE=FILE NAME=CPU_ONLINE >
CORE_4 /sys/devices/system/cpu/cpu4/online
CORE_5 /sys/devices/system/cpu/cpu5/online
CORE_6 /sys/devices/system/cpu/cpu6/online
CORE_7 /sys/devices/system/cpu/cpu7/online

< PARAM TYPE=CLOCK NAME=CPU_A78_0 >
FREQ_TABLE /sys/devices/system/cpu/cpu0/cpufreq/scaling_available_frequencies
MAX_FREQ /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq
MIN_FREQ /sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq
FREQ_TABLE_KNEXT /sys/devices/system/cpu/cpu0/cpufreq/scaling_available_frequencies
MAX_FREQ_KNEXT /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq
MIN_FREQ_KNEXT /sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq

< PARAM TYPE=CLOCK NAME=CPU_A78_1 >
FREQ_TABLE /sys/devices/system/cpu/cpu4/cpufreq/scaling_available_frequencies
MAX_FREQ /sys/devices/system/cpu/cpu4/cpufreq/scaling_max_freq
MIN_FREQ /sys/devices/system/cpu/cpu4/cpufreq/scaling_min_freq

<PARAM TYPE=CLOCK NAME=EMC >
MAX_FREQ /sys/kernel/nvpmodel_emc_cap/emc_iso_cap
";

    /// Three power modes whose CPU limits are the Orin NX 16GB's MAXN
    /// (`-1`, no cap: the hardware's 1984 MHz), 15W (four CPUs, 1420.8 MHz)
    /// and 25W (1497.6 MHz), each with a 729.6 MHz minimum.
    const ORIN_MODES: &str = "\
< POWER_MODEL ID=0 NAME=MAXN >
CPU_A78_0 MIN_FREQ 729600
CPU_A78_0 MAX_FREQ -1
CPU_A78_1 MIN_FREQ 729600
CPU_A78_1 MAX_FREQ -1
EMC MAX_FREQ 0

< POWER_MODEL ID=2 NAME=15W >
CPU_ONLINE CORE_4 0
CPU_ONLINE CORE_5 0
CPU_ONLINE CORE_6 0
CPU_ONLINE CORE_7 0
CPU_A78_0 MIN_FREQ 729600
CPU_A78_0 MAX_FREQ 1420800

< POWER_MODEL ID=3 NAME=25W >
CPU_A78_0 MIN_FREQ 729600
CPU_A78_0 MAX_FREQ 1497600
CPU_A78_1 MIN_FREQ 729600
CPU_A78_1 MAX_FREQ 1497600
";

    /// A custom mode: 25W, with the second cluster's minimum raised to its
    /// maximum.
    const LATENCY_MODE: &str = "\
< POWER_MODEL ID=8 NAME=LATENCY >
CPU_A78_0 MIN_FREQ 729600
CPU_A78_0 MAX_FREQ 1497600
CPU_A78_1 MIN_FREQ 1497600
CPU_A78_1 MAX_FREQ 1497600
";

    const ORIN_DEFAULT: &str = "\n# mandatory section\n< PM_CONFIG DEFAULT=2 >\n";

    fn orin_conf(extra: &str) -> String {
        format!("{ORIN_PARAMS}\n{ORIN_MODES}\n{extra}{ORIN_DEFAULT}")
    }

    /// Some of the Orin NX CPU clock's available frequencies (kHz): its
    /// lowest and highest, and the steps the power modes use.
    const ORIN_STEPS: &str = "115200 729600 1190400 1420800 1497600 1984000";

    /// An Orin NX cluster: governor schedutil, the hardware range
    /// 115.2-1984 MHz, and `min`, `max`, `cur` (kHz).
    fn orin_cluster(tree: &Tree, cpus: &[u32], min: u64, max: u64, cur: u64) {
        let (min, max, cur) = (min.to_string(), max.to_string(), cur.to_string());
        tree.policy(
            cpus,
            &[
                ("scaling_governor", "schedutil"),
                ("cpuinfo_min_freq", "115200"),
                ("cpuinfo_max_freq", "1984000"),
                ("scaling_available_frequencies", ORIN_STEPS),
                ("scaling_min_freq", &min),
                ("scaling_max_freq", &max),
                ("scaling_cur_freq", &cur),
            ],
        );
    }

    fn text(o: &Outcome, key: &str) -> Option<String> {
        match o.readings.get(key) {
            Some(Reading::Text(t)) => Some(t.clone()),
            None => None,
            other => panic!("{key} is not text: {other:?}"),
        }
    }

    fn int(o: &Outcome, key: &str) -> Option<u64> {
        match o.readings.get(key) {
            Some(Reading::Int(v)) => Some(*v),
            None => None,
            other => panic!("{key} is not an integer: {other:?}"),
        }
    }

    #[test]
    fn an_orin_in_25w_mode_reports_the_power_mode_cap_not_the_hardware_maximum() {
        let tree = Tree::new("orin-25w");
        orin_cluster(&tree, &[0, 1, 2, 3], 729_600, 1_497_600, 729_600);
        orin_cluster(&tree, &[4, 5, 6, 7], 729_600, 1_497_600, 1_497_600);
        tree.file("var/lib/nvpmodel/status", "pmode:0003 fmode:quiet\n");
        tree.file("etc/nvpmodel.conf", &orin_conf(""));
        let o = decide(&tree.read());
        assert_eq!(decide(&tree.read()), o, "a second read decides the same");
        assert_eq!(o.status, Status::Info);
        assert_eq!(
            o.summary,
            "nvpmodel mode 3 (25W); CPUs 0-3: schedutil, now 729.6 MHz, limits 729.6-1497.6 MHz \
             (hardware 115.2-1984 MHz); CPUs 4-7: schedutil, now 1497.6 MHz, limits \
             729.6-1497.6 MHz (hardware 115.2-1984 MHz); no clock floor, so no latency cluster"
        );
        assert_eq!(o.fix, None);
        assert_eq!(int(&o, "scaling_max_freq_khz"), Some(1_497_600));
        assert_eq!(int(&o, "max_freq_khz"), Some(1_984_000));
        assert_eq!(int(&o, "min_freq_khz"), Some(729_600));
        assert_eq!(int(&o, "cpus"), Some(8));
        assert_eq!(int(&o, "nvpmodel_mode"), Some(3));
        assert_eq!(text(&o, "nvpmodel_mode_name").as_deref(), Some("25W"));
        assert_eq!(
            o.readings.get("nvpmodel_mode_defined"),
            Some(&Reading::Bool(true))
        );
        assert_eq!(text(&o, "clusters").as_deref(), Some("0-3;4-7"));
        assert_eq!(
            text(&o, "cluster_max_khz").as_deref(),
            Some("1497600;1497600")
        );
        assert_eq!(
            text(&o, "cluster_hw_max_khz").as_deref(),
            Some("1984000;1984000")
        );
        assert_eq!(
            text(&o, "cluster_cur_khz").as_deref(),
            Some("729600;1497600")
        );
        assert_eq!(text(&o, "cluster_floors").as_deref(), Some("none;none"));
        assert_eq!(text(&o, "latency_cluster"), None);
    }

    #[test]
    fn an_orin_in_maxn_reports_the_hardware_maximum_as_the_cap() {
        let tree = Tree::new("orin-maxn");
        orin_cluster(&tree, &[0, 1, 2, 3], 729_600, 1_984_000, 1_984_000);
        orin_cluster(&tree, &[4, 5, 6, 7], 729_600, 1_984_000, 729_600);
        tree.file("var/lib/nvpmodel/status", "pmode:0000 fmode:quiet\n");
        tree.file("etc/nvpmodel.conf", &orin_conf(""));
        let o = decide(&tree.read());
        assert_eq!(o.status, Status::Info);
        assert_eq!(
            o.summary,
            "nvpmodel mode 0 (MAXN); CPUs 0-3: schedutil, now 1984 MHz, limits 729.6-1984 MHz \
             (hardware 115.2-1984 MHz); CPUs 4-7: schedutil, now 729.6 MHz, limits \
             729.6-1984 MHz (hardware 115.2-1984 MHz); no clock floor, so no latency cluster"
        );
        assert_eq!(int(&o, "scaling_max_freq_khz"), Some(1_984_000));
    }

    #[test]
    fn a_custom_mode_that_pins_one_cluster_names_it_the_latency_cluster() {
        let tree = Tree::new("orin-custom");
        orin_cluster(&tree, &[0, 1, 2, 3], 729_600, 1_497_600, 729_600);
        orin_cluster(&tree, &[4, 5, 6, 7], 1_497_600, 1_497_600, 1_497_600);
        tree.file("var/lib/nvpmodel/status", "pmode:0008 fmode:quiet\n");
        tree.file("etc/nvpmodel.conf", &orin_conf(LATENCY_MODE));
        let o = decide(&tree.read());
        assert_eq!(o.status, Status::Info);
        assert_eq!(
            o.summary,
            "nvpmodel mode 8 (LATENCY); CPUs 0-3: schedutil, now 729.6 MHz, limits \
             729.6-1497.6 MHz (hardware 115.2-1984 MHz); CPUs 4-7: schedutil, now 1497.6 MHz, \
             limits 1497.6-1497.6 MHz (hardware 115.2-1984 MHz), clock floor from the power \
             mode; latency cluster CPUs 4-7"
        );
        assert_eq!(text(&o, "cluster_floors").as_deref(), Some("none;nvpmodel"));
        assert_eq!(text(&o, "latency_cluster").as_deref(), Some("4-7"));
    }

    #[test]
    fn a_scaling_min_freq_above_the_modes_minimum_is_a_floor() {
        // The stock 25W mode, and a boot unit that raised the second
        // cluster's scaling_min_freq to its maximum.
        let tree = Tree::new("orin-unit");
        orin_cluster(&tree, &[0, 1, 2, 3], 729_600, 1_497_600, 729_600);
        orin_cluster(&tree, &[4, 5, 6, 7], 1_497_600, 1_497_600, 1_497_600);
        tree.file("var/lib/nvpmodel/status", "pmode:0003 fmode:quiet\n");
        tree.file("etc/nvpmodel.conf", &orin_conf(""));
        let o = decide(&tree.read());
        assert_eq!(o.status, Status::Info);
        assert!(
            o.summary.ends_with(
                "limits 1497.6-1497.6 MHz (hardware 115.2-1984 MHz), clock floor from \
                 scaling_min_freq; latency cluster CPUs 4-7"
            ),
            "{}",
            o.summary
        );
        assert_eq!(
            text(&o, "cluster_floors").as_deref(),
            Some("none;scaling_min_freq")
        );
        assert_eq!(text(&o, "latency_cluster").as_deref(), Some("4-7"));
    }

    #[test]
    fn a_floor_on_every_cluster_names_no_single_latency_cluster() {
        // Both clusters' minimum raised to the 25W cap, as `jetson_clocks`
        // leaves them.
        let tree = Tree::new("orin-all");
        orin_cluster(&tree, &[0, 1, 2, 3], 1_497_600, 1_497_600, 1_497_600);
        orin_cluster(&tree, &[4, 5, 6, 7], 1_497_600, 1_497_600, 1_497_600);
        tree.file("var/lib/nvpmodel/status", "pmode:0003 fmode:quiet\n");
        tree.file("etc/nvpmodel.conf", &orin_conf(""));
        let o = decide(&tree.read());
        assert!(
            o.summary
                .ends_with("; a clock floor on every online cluster, so no single latency cluster"),
            "{}",
            o.summary
        );
        assert_eq!(text(&o, "latency_cluster"), None);
    }

    #[test]
    fn the_higher_of_two_floors_is_the_latency_cluster() {
        let tree = Tree::new("orin-two-floors");
        orin_cluster(&tree, &[0, 1, 2, 3], 1_190_400, 1_497_600, 1_190_400);
        orin_cluster(&tree, &[4, 5, 6, 7], 1_497_600, 1_497_600, 1_497_600);
        tree.file("var/lib/nvpmodel/status", "pmode:0003 fmode:quiet\n");
        tree.file("etc/nvpmodel.conf", &orin_conf(""));
        let o = decide(&tree.read());
        assert_eq!(
            text(&o, "cluster_floors").as_deref(),
            Some("scaling_min_freq;scaling_min_freq")
        );
        assert_eq!(text(&o, "latency_cluster").as_deref(), Some("4-7"));
    }

    #[test]
    fn a_selected_mode_the_conf_no_longer_defines_warns_with_the_fix() {
        // Mode 8 was applied, then an L4T upgrade put back a file without it:
        // the live limits are still mode 8's.
        let tree = Tree::new("orin-rewritten");
        orin_cluster(&tree, &[0, 1, 2, 3], 729_600, 1_497_600, 729_600);
        orin_cluster(&tree, &[4, 5, 6, 7], 1_497_600, 1_497_600, 1_497_600);
        tree.file("var/lib/nvpmodel/status", "pmode:0008 fmode:quiet\n");
        tree.file("etc/nvpmodel.conf", &orin_conf(""));
        let o = decide(&tree.read());
        assert_eq!(o.status, Status::Warn);
        assert!(
            o.summary.starts_with(
                "nvpmodel mode 8 is selected but /etc/nvpmodel.conf does not define it, so the \
                 next boot does not apply it; CPUs 0-3: "
            ),
            "{}",
            o.summary
        );
        assert_eq!(
            o.fix.as_deref(),
            Some(
                "put the `< POWER_MODEL ID=8 ... >` section back into /etc/nvpmodel.conf (an L4T \
                 upgrade rewrites that file), or select a mode the file defines with `sudo \
                 nvpmodel -m <ID>`"
            )
        );
        assert_eq!(int(&o, "nvpmodel_mode"), Some(8));
        assert_eq!(
            o.readings.get("nvpmodel_mode_defined"),
            Some(&Reading::Bool(false))
        );
        assert_eq!(text(&o, "nvpmodel_mode_name"), None);
        // Mode 8 is not read, so the second cluster's raised minimum has no
        // known source: the boot default (15W) sets no limit on it, and the
        // lowest minimum another mode sets is below it.
        assert_eq!(text(&o, "cluster_floors").as_deref(), Some("none;raised"));
        assert!(
            o.summary.ends_with(
                "limits 1497.6-1497.6 MHz (hardware 115.2-1984 MHz), clock floor from an \
                 unknown source; latency cluster CPUs 4-7"
            ),
            "{}",
            o.summary
        );
    }

    #[test]
    fn a_status_with_no_mode_reports_the_boot_default() {
        let tree = Tree::new("orin-no-status-mode");
        orin_cluster(&tree, &[0, 1, 2, 3], 729_600, 1_420_800, 729_600);
        tree.file("var/lib/nvpmodel/status", "\n");
        tree.file("etc/nvpmodel.conf", &orin_conf(""));
        let o = decide(&tree.read());
        assert_eq!(o.status, Status::Info);
        assert!(
            o.summary
                .starts_with("no nvpmodel mode recorded, boot default mode 2 (15W); CPUs 0-3: "),
            "{}",
            o.summary
        );
        assert_eq!(int(&o, "nvpmodel_mode"), None);
    }

    #[test]
    fn an_l4t_system_without_nvpmodel_files_claims_no_floor_from_the_hardware_minimum() {
        // A container on a Jetson: sysfs and /etc/nv_tegra_release, no
        // nvpmodel files. The 729.6 MHz minimum is the power mode's, not a
        // floor.
        let tree = Tree::new("orin-container");
        orin_cluster(&tree, &[0, 1, 2, 3], 729_600, 1_497_600, 729_600);
        orin_cluster(&tree, &[4, 5, 6, 7], 729_600, 1_497_600, 1_497_600);
        tree.file(
            "etc/nv_tegra_release",
            "# R35 (release), REVISION: 4.1, GCID: 0, BOARD: t186ref\n",
        );
        let o = decide(&tree.read());
        assert_eq!(o.status, Status::Info);
        assert!(
            o.summary
                .starts_with("no nvpmodel mode read, /etc/nvpmodel.conf not read; CPUs 0-3: "),
            "{}",
            o.summary
        );
        assert_eq!(text(&o, "cluster_floors").as_deref(), Some("none;none"));

        // A thermal limit holding a cluster at the hardware minimum is no
        // floor either.
        orin_cluster(&tree, &[4, 5, 6, 7], 115_200, 115_200, 115_200);
        let o = decide(&tree.read());
        assert_eq!(text(&o, "cluster_floors").as_deref(), Some("none;none"));
    }

    #[test]
    fn a_status_without_a_readable_conf_is_info() {
        let tree = Tree::new("orin-no-conf");
        orin_cluster(&tree, &[0, 1, 2, 3], 729_600, 1_497_600, 729_600);
        tree.file("var/lib/nvpmodel/status", "pmode:0003\n");
        let o = decide(&tree.read());
        assert_eq!(o.status, Status::Info);
        assert!(
            o.summary
                .starts_with("nvpmodel mode 3, /etc/nvpmodel.conf not read; CPUs 0-3: "),
            "{}",
            o.summary
        );
        assert_eq!(o.readings.get("nvpmodel_mode_defined"), None);
        // With no mode to read, the minimum the clock starts from is unknown,
        // so the 729.6 MHz minimum is no floor; a minimum at the maximum is a
        // floor whose source is not known.
        assert_eq!(text(&o, "cluster_floors").as_deref(), Some("none"));
        orin_cluster(&tree, &[0, 1, 2, 3], 1_497_600, 1_497_600, 1_497_600);
        let o = decide(&tree.read());
        assert_eq!(text(&o, "cluster_floors").as_deref(), Some("raised"));
        assert_eq!(text(&o, "latency_cluster").as_deref(), Some("0-3"));
    }

    #[test]
    fn a_conf_with_no_power_mode_read_is_not_a_missing_mode() {
        let tree = Tree::new("orin-empty-conf");
        orin_cluster(&tree, &[0, 1, 2, 3], 729_600, 1_497_600, 729_600);
        tree.file("var/lib/nvpmodel/status", "pmode:0003\n");
        tree.file("etc/nvpmodel.conf", ORIN_PARAMS);
        let o = decide(&tree.read());
        assert_eq!(o.status, Status::Info);
        assert!(
            o.summary.starts_with(
                "nvpmodel mode 3, no power mode read from /etc/nvpmodel.conf; CPUs 0-3: "
            ),
            "{}",
            o.summary
        );
        assert_eq!(o.readings.get("nvpmodel_mode_defined"), None);
        // With no mode read, the 729.6 MHz minimum below the maximum is no
        // floor.
        assert_eq!(text(&o, "cluster_floors").as_deref(), Some("none"));
    }

    #[test]
    fn an_offline_cluster_reads_offline_and_holds_no_floor() {
        // The 15W mode takes CPUs 4-7 offline, and their policy names no
        // affected CPU (newer kernels refuse the read with EBUSY instead,
        // which the next test covers).
        let tree = Tree::new("orin-15w");
        orin_cluster(&tree, &[0, 1, 2, 3], 729_600, 1_420_800, 729_600);
        orin_cluster(&tree, &[4, 5, 6, 7], 729_600, 1_497_600, 729_600);
        for cpu in 4..8 {
            tree.file(
                &format!("sys/devices/system/cpu/cpu{cpu}/cpufreq/affected_cpus"),
                "\n",
            );
        }
        tree.file("var/lib/nvpmodel/status", "pmode:0002 fmode:quiet\n");
        tree.file("etc/nvpmodel.conf", &orin_conf(""));
        let o = decide(&tree.read());
        assert_eq!(
            o.summary,
            "nvpmodel mode 2 (15W); CPUs 0-3: schedutil, now 729.6 MHz, limits 729.6-1420.8 MHz \
             (hardware 115.2-1984 MHz); CPUs 4-7: offline; no clock floor, so no latency cluster"
        );
        assert_eq!(text(&o, "cluster_floors").as_deref(), Some("none;offline"));
        assert_eq!(int(&o, "scaling_max_freq_khz"), Some(1_420_800));
        assert_eq!(int(&o, "cpus"), Some(8));

        // A cluster whose reads were refused carries nothing but its CPUs.
        let mut host = tree.read();
        host.clusters[1] = Cluster {
            cpus: BTreeSet::from([4, 5, 6, 7]),
            online: false,
            ..Cluster::default()
        };
        let o = decide(&host);
        assert_eq!(text(&o, "cluster_floors").as_deref(), Some("none;offline"));
        assert_eq!(text(&o, "governors").as_deref(), Some("schedutil,unknown"));
        assert!(o.summary.contains("; CPUs 4-7: offline; "), "{}", o.summary);
    }

    #[test]
    fn a_policy_read_refused_with_ebusy_is_offline() {
        let busy = std::io::Error::from(std::io::ErrorKind::ResourceBusy);
        assert!(!cluster_online(Err(busy)));
        let absent = std::io::Error::from(std::io::ErrorKind::NotFound);
        assert!(cluster_online(Err(absent)));
        assert!(cluster_online(Ok("4 5 6 7\n".to_string())));
        assert!(!cluster_online(Ok("\n".to_string())));
    }

    /// On a live host each `cpu<N>/cpufreq` is a link to the policy's
    /// directory. With `related_cpus` unreadable, as on an offline policy, the
    /// links still group the CPUs into one cluster.
    #[cfg(unix)]
    #[test]
    fn cpus_whose_cpufreq_links_share_a_policy_are_one_cluster() {
        let tree = Tree::new("links");
        tree.file(
            "sys/devices/system/cpu/cpufreq/policy4/scaling_governor",
            "schedutil\n",
        );
        tree.file(
            "sys/devices/system/cpu/cpufreq/policy0/scaling_governor",
            "schedutil\n",
        );
        for cpu in 0..8 {
            let policy = if cpu < 4 { 0 } else { 4 };
            let dir = tree.0.join(format!("sys/devices/system/cpu/cpu{cpu}"));
            std::fs::create_dir_all(&dir).unwrap();
            std::os::unix::fs::symlink(format!("../cpufreq/policy{policy}"), dir.join("cpufreq"))
                .unwrap();
        }
        let clusters = read_clusters(&tree.0.join("sys/devices/system/cpu"));
        let cpus: Vec<BTreeSet<u32>> = clusters.iter().map(|c| c.cpus.clone()).collect();
        assert_eq!(
            cpus,
            vec![BTreeSet::from([0, 1, 2, 3]), BTreeSet::from([4, 5, 6, 7])]
        );
    }

    #[test]
    fn a_mode_minimum_off_a_frequency_step_or_below_the_hardware_is_no_floor() {
        // The kernel stores 729000 as the next step, 729600, and 0 as the
        // hardware minimum, 115200: neither live minimum is a floor.
        for (mode_min, live_min) in [(729_000, 729_600), (0, 115_200)] {
            let tree = Tree::new(&format!("orin-stored-{mode_min}"));
            orin_cluster(&tree, &[0, 1, 2, 3], live_min, 1_497_600, live_min);
            tree.file("var/lib/nvpmodel/status", "pmode:0009\n");
            let mode = format!(
                "< POWER_MODEL ID=9 NAME=ODD >\nCPU_A78_0 MIN_FREQ {mode_min}\n\
                 CPU_A78_0 MAX_FREQ 1497600\n"
            );
            tree.file("etc/nvpmodel.conf", &orin_conf(&mode));
            let o = decide(&tree.read());
            assert_eq!(
                text(&o, "cluster_floors").as_deref(),
                Some("none"),
                "mode minimum {mode_min}: {}",
                o.summary
            );
        }
    }

    #[test]
    fn a_mode_minimum_at_the_hardware_maximum_under_no_cap_pins_the_cluster() {
        let tree = Tree::new("orin-pin-maxn");
        orin_cluster(&tree, &[0, 1, 2, 3], 729_600, 1_984_000, 729_600);
        orin_cluster(&tree, &[4, 5, 6, 7], 1_984_000, 1_984_000, 1_984_000);
        tree.file("var/lib/nvpmodel/status", "pmode:0010\n");
        let mode = "< POWER_MODEL ID=10 NAME=MAXN_LATENCY >\nCPU_A78_0 MIN_FREQ 729600\n\
                    CPU_A78_0 MAX_FREQ -1\nCPU_A78_1 MIN_FREQ 1984000\nCPU_A78_1 MAX_FREQ -1\n";
        tree.file("etc/nvpmodel.conf", &orin_conf(mode));
        let o = decide(&tree.read());
        assert_eq!(text(&o, "cluster_floors").as_deref(), Some("none;nvpmodel"));
        assert_eq!(text(&o, "latency_cluster").as_deref(), Some("4-7"));
    }

    #[test]
    fn a_cluster_no_mode_sets_a_minimum_for_starts_at_the_hardware_minimum() {
        let tree = Tree::new("orin-cap-only");
        orin_cluster(&tree, &[0, 1, 2, 3], 115_200, 1_497_600, 115_200);
        orin_cluster(&tree, &[4, 5, 6, 7], 1_190_400, 1_497_600, 1_190_400);
        tree.file("var/lib/nvpmodel/status", "pmode:0005\n");
        let conf = format!(
            "{ORIN_PARAMS}\n< POWER_MODEL ID=5 NAME=CAP_ONLY >\nCPU_A78_0 MAX_FREQ 1497600\n\
             CPU_A78_1 MAX_FREQ 1497600\n< PM_CONFIG DEFAULT=5 >\n"
        );
        tree.file("etc/nvpmodel.conf", &conf);
        let o = decide(&tree.read());
        assert_eq!(
            text(&o, "cluster_floors").as_deref(),
            Some("none;scaling_min_freq")
        );
        assert_eq!(text(&o, "latency_cluster").as_deref(), Some("4-7"));
    }

    #[test]
    fn a_thermal_cap_below_the_modes_minimum_is_no_floor() {
        // A thermal limit holds both limits at 652.8 MHz, below the 25W
        // mode's 729.6 MHz minimum: the clock is pinned, but by the cap.
        let tree = Tree::new("orin-thermal");
        orin_cluster(&tree, &[0, 1, 2, 3], 652_800, 652_800, 652_800);
        tree.file("var/lib/nvpmodel/status", "pmode:0003\n");
        tree.file("etc/nvpmodel.conf", &orin_conf(""));
        let o = decide(&tree.read());
        assert_eq!(text(&o, "cluster_floors").as_deref(), Some("none"));
    }

    /// An x86 CPU under `intel_pstate`: one policy per CPU, 800-4700 MHz.
    fn x86_cpu(tree: &Tree, cpu: u32, governor: &str, min: u64, cur: u64) {
        let (min, cur) = (min.to_string(), cur.to_string());
        tree.policy(
            &[cpu],
            &[
                ("scaling_governor", governor),
                ("cpuinfo_min_freq", "800000"),
                ("cpuinfo_max_freq", "4700000"),
                ("scaling_min_freq", &min),
                ("scaling_max_freq", "4700000"),
                ("scaling_cur_freq", &cur),
            ],
        );
    }

    #[test]
    fn plain_x86_folds_one_cpu_policies_into_one_row_with_no_power_mode() {
        let tree = Tree::new("x86");
        for (cpu, cur) in [(0, 800_000), (1, 1_200_000), (2, 4_600_000), (3, 800_000)] {
            x86_cpu(&tree, cpu, "powersave", 800_000, cur);
        }
        let o = decide(&tree.read());
        assert_eq!(o.status, Status::Info);
        assert_eq!(
            o.summary,
            "CPUs 0-3: powersave, now 800-4600 MHz, limits 800-4700 MHz; no clock floor, so no \
             latency cluster"
        );
        assert_eq!(int(&o, "nvpmodel_mode"), None);
        assert_eq!(text(&o, "clusters").as_deref(), Some("0;1;2;3"));
        assert_eq!(int(&o, "scaling_max_freq_khz"), Some(4_700_000));
        assert_eq!(text(&o, "governors").as_deref(), Some("powersave"));
    }

    #[test]
    fn plain_x86_with_a_raised_minimum_on_two_cpus_names_them() {
        let tree = Tree::new("x86-floor");
        for cpu in 0..4 {
            let min = if cpu >= 2 { 3_000_000 } else { 800_000 };
            x86_cpu(&tree, cpu, "powersave", min, min);
        }
        let o = decide(&tree.read());
        assert_eq!(
            o.summary,
            "CPUs 0-1: powersave, now 800 MHz, limits 800-4700 MHz; CPUs 2-3: powersave, now \
             3000 MHz, limits 3000-4700 MHz (hardware 800-4700 MHz), clock floor from \
             scaling_min_freq; latency cluster CPUs 2-3"
        );
        assert_eq!(text(&o, "latency_cluster").as_deref(), Some("2-3"));
    }

    #[test]
    fn the_performance_governor_is_a_floor_ranked_at_the_maximum() {
        let tree = Tree::new("x86-perf");
        x86_cpu(&tree, 0, "performance", 800_000, 4_700_000);
        let o = decide(&tree.read());
        assert_eq!(text(&o, "cluster_floors").as_deref(), Some("performance"));
        assert!(
            o.summary
                .ends_with(", clock floor from the performance governor; latency cluster CPU 0"),
            "{}",
            o.summary
        );

        // A performance cluster with a raised minimum below its maximum still
        // runs at its maximum, so it outranks a raised minimum that is higher
        // than that minimum but below the maximum.
        x86_cpu(&tree, 0, "performance", 1_200_000, 4_700_000);
        x86_cpu(&tree, 1, "powersave", 3_000_000, 3_000_000);
        let o = decide(&tree.read());
        assert_eq!(
            text(&o, "cluster_floors").as_deref(),
            Some("performance;scaling_min_freq")
        );
        assert_eq!(text(&o, "latency_cluster").as_deref(), Some("0"));
    }

    #[test]
    fn amd_pstate_starting_at_its_lowest_efficient_clock_is_not_a_floor() {
        let tree = Tree::new("amd");
        tree.policy(
            &[0],
            &[
                ("scaling_governor", "powersave"),
                ("cpuinfo_min_freq", "400000"),
                ("cpuinfo_max_freq", "5000000"),
                ("amd_pstate_lowest_nonlinear_freq", "1400000"),
                ("scaling_min_freq", "1400000"),
                ("scaling_max_freq", "5000000"),
            ],
        );
        let o = decide(&tree.read());
        assert_eq!(text(&o, "cluster_floors").as_deref(), Some("none"));
        assert_eq!(text(&o, "cluster_cur_khz").as_deref(), Some("-"));
        assert_eq!(
            o.summary,
            "CPU 0: powersave, limits 1400-5000 MHz (hardware 400-5000 MHz); no clock floor, so \
             no latency cluster"
        );
    }

    #[test]
    fn no_cpufreq_is_info_too() {
        let o = decide(&HostClock::default());
        assert_eq!(o.status, Status::Info);
        assert_eq!(
            o.summary,
            "no cpufreq directory under /sys/devices/system/cpu, so there is no governor to read"
        );
        assert_eq!(int(&o, "cpus"), Some(0));
        assert_eq!(o.readings.get("governors"), None);
    }

    #[test]
    fn a_cluster_with_no_readable_files_reads_unknown() {
        let o = decide(&HostClock {
            clusters: vec![Cluster {
                cpus: BTreeSet::from([0]),
                online: true,
                ..Cluster::default()
            }],
            nvpmodel: None,
        });
        assert_eq!(
            o.summary,
            "CPU 0: governor unknown; no clock floor, so no latency cluster"
        );
        assert_eq!(text(&o, "governors").as_deref(), Some("unknown"));
        assert_eq!(text(&o, "cluster_min_khz").as_deref(), Some("-"));

        // The performance governor with no clock read still ranks as a floor.
        let o = decide(&HostClock {
            clusters: vec![Cluster {
                cpus: BTreeSet::from([0]),
                online: true,
                governor: Some("performance".to_string()),
                ..Cluster::default()
            }],
            nvpmodel: None,
        });
        assert_eq!(
            o.summary,
            "CPU 0: performance, clock floor from the performance governor; latency cluster \
             CPU 0"
        );
    }

    #[test]
    fn read_clusters_reads_each_policy_once_in_cpu_order() {
        let tree = Tree::new("read");
        orin_cluster(&tree, &[4, 5, 6, 7], 729_600, 1_497_600, 729_600);
        orin_cluster(&tree, &[0, 1, 2, 3], 729_600, 1_497_600, 1_497_600);
        // A CPU directory with no cpufreq, a cpufreq directory that is not a
        // CPU's, and a CPU whose related_cpus is missing.
        tree.file("sys/devices/system/cpu/cpu8/online", "1\n");
        tree.file("sys/devices/system/cpu/cpufreq/policy0/related_cpus", "0\n");
        tree.file(
            "sys/devices/system/cpu/cpu10/cpufreq/scaling_governor",
            "performance\n",
        );
        let clusters = read_clusters(&tree.0.join("sys/devices/system/cpu"));
        let cpus: Vec<BTreeSet<u32>> = clusters.iter().map(|c| c.cpus.clone()).collect();
        assert_eq!(
            cpus,
            vec![
                BTreeSet::from([0, 1, 2, 3]),
                BTreeSet::from([4, 5, 6, 7]),
                BTreeSet::from([10]),
            ]
        );
        assert_eq!(
            clusters[0],
            Cluster {
                cpus: BTreeSet::from([0, 1, 2, 3]),
                online: true,
                governor: Some("schedutil".to_string()),
                min_khz: Some(729_600),
                max_khz: Some(1_497_600),
                cur_khz: Some(1_497_600),
                hw_min_khz: Some(115_200),
                hw_max_khz: Some(1_984_000),
                lowest_nonlinear_khz: None,
                available_khz: vec![115_200, 729_600, 1_190_400, 1_420_800, 1_497_600, 1_984_000],
            }
        );
        // No affected_cpus file: online.
        assert!(clusters[2].online);
        assert_eq!(clusters[2].governor.as_deref(), Some("performance"));
        assert_eq!(clusters[2].min_khz, None);
    }

    #[test]
    fn read_host_reads_no_power_mode_where_neither_nvpmodel_file_exists() {
        let tree = Tree::new("no-nvpmodel");
        orin_cluster(&tree, &[0], 800_000, 800_000, 800_000);
        assert_eq!(tree.read().nvpmodel, None);
        tree.file("etc/nvpmodel.conf", &orin_conf(""));
        let host = tree.read();
        let n = host.nvpmodel.expect("the conf alone is a Jetson");
        assert_eq!(n.selected, None);
        assert_eq!(n.conf.map(|c| c.default), Some(Some(2)));
    }

    #[test]
    fn parse_conf_maps_each_mode_cpu_limit_to_the_cpu_its_clock_path_names() {
        let conf = parse_conf(&orin_conf(LATENCY_MODE));
        let ids: Vec<(u32, &str)> = conf.modes.iter().map(|m| (m.id, m.name.as_str())).collect();
        assert_eq!(
            ids,
            vec![(0, "MAXN"), (2, "15W"), (3, "25W"), (8, "LATENCY")]
        );
        assert_eq!(conf.default, Some(2));
        let limit = |id: u32, cpu: u32| conf.mode(id).unwrap().cpu_limits.get(&cpu).copied();
        let l = |min: i64, max: i64| {
            Some(CpuLimit {
                min: Some(min),
                max: Some(max),
            })
        };
        assert_eq!(limit(0, 0), l(729_600, -1));
        assert_eq!(limit(0, 4), l(729_600, -1));
        assert_eq!(limit(2, 0), l(729_600, 1_420_800));
        assert_eq!(limit(2, 4), None);
        assert_eq!(limit(3, 4), l(729_600, 1_497_600));
        assert_eq!(limit(8, 4), l(1_497_600, 1_497_600));
        // Neither the CPU online switches nor the EMC clock are CPU limits.
        assert_eq!(conf.mode(0).unwrap().cpu_limits.len(), 2);
    }

    #[test]
    fn parse_conf_reads_policy_paths_and_skips_lines_it_does_not_know() {
        let conf = parse_conf(
            "< PARAM TYPE=CLOCK NAME=BIG >\n\
             MAX_FREQ_KNEXT /sys/devices/system/cpu/cpufreq/policy6/scaling_max_freq\n\
             a stray line\n\
             < POWER_MODEL ID=x NAME=BAD >\n\
             BIG MAX_FREQ 1000\n\
             < POWER_MODEL ID=1 NAME=ONE >\n\
             BIG MIN_FREQ 2000000\n\
             BIG MAX_FREQ notanumber\n\
             UNDECLARED MIN_FREQ 5\n\
             < PM_CONFIG DEFAULT=7 >\n",
        );
        assert_eq!(conf.default, Some(7));
        assert_eq!(conf.modes.len(), 1);
        assert_eq!(
            conf.modes[0].cpu_limits.get(&6).copied(),
            Some(CpuLimit {
                min: Some(2_000_000),
                max: None,
            })
        );
    }

    #[test]
    fn a_mode_pins_a_clock_only_when_its_minimum_reaches_its_maximum() {
        let hw = Some(1_984_000);
        let pins = |min, max| CpuLimit { min, max }.pins(hw);
        assert!(pins(Some(1_497_600), Some(1_497_600)));
        assert!(pins(Some(-1), None));
        assert!(pins(Some(-1), Some(-1)));
        assert!(pins(Some(1_984_000), Some(-1)));
        assert!(pins(Some(2_201_600), Some(-1)));
        assert!(pins(Some(1_984_000), None));
        assert!(pins(Some(1_984_000), Some(2_201_600)));
        assert!(!pins(Some(729_600), Some(1_497_600)));
        assert!(!pins(Some(729_600), Some(-1)));
        assert!(!pins(Some(1_497_600), None));
        assert!(!pins(None, Some(1_497_600)));
        assert!(!pins(Some(-5), Some(1_497_600)));
        // With no hardware maximum read, `-1` is the largest value.
        assert!(!CpuLimit {
            min: Some(1_984_000),
            max: Some(-1)
        }
        .pins(None));
    }

    #[test]
    fn selected_mode_reads_the_pmode_field() {
        assert_eq!(selected_mode("pmode:0003 fmode:quiet\n"), Some(3));
        assert_eq!(selected_mode("pmode:0000 fmode:fanNull"), Some(0));
        assert_eq!(selected_mode("fmode:quiet pmode:12"), Some(12));
        assert_eq!(selected_mode(""), None);
        assert_eq!(selected_mode("pmode:"), None);
        assert_eq!(selected_mode("mode 3"), None);
    }

    #[test]
    fn cpufreq_cpu_reads_cpu_and_policy_paths_only() {
        assert_eq!(
            cpufreq_cpu("/sys/devices/system/cpu/cpu4/cpufreq/scaling_max_freq"),
            Some(4)
        );
        assert_eq!(
            cpufreq_cpu("/sys/devices/system/cpu/cpufreq/policy12/scaling_min_freq"),
            Some(12)
        );
        assert_eq!(cpufreq_cpu("/sys/devices/system/cpu/cpu4/online"), None);
        assert_eq!(
            cpufreq_cpu("/sys/kernel/nvpmodel_emc_cap/emc_iso_cap"),
            None
        );
    }

    #[test]
    fn cpu_lists_parse_and_print_in_the_linux_form() {
        assert_eq!(parse_cpus("4 5 6 7\n"), BTreeSet::from([4, 5, 6, 7]));
        assert_eq!(parse_cpus("3 1 x 1"), BTreeSet::from([1, 3]));
        assert_eq!(parse_cpus("\n"), BTreeSet::new());
        let set = |v: &[u32]| v.iter().copied().collect::<BTreeSet<u32>>();
        assert_eq!(cpu_list(&set(&[0, 1, 2, 3, 5, 7, 8])), "0-3,5,7-8");
        assert_eq!(cpu_label(&set(&[5])), "CPU 5");
        assert_eq!(cpu_label(&set(&[4, 5, 6, 7])), "CPUs 4-7");
    }

    #[test]
    fn mhz_prints_at_most_one_decimal() {
        assert_eq!(mhz(729_600), "729.6");
        assert_eq!(mhz(1_984_000), "1984");
        assert_eq!(mhz(115_200), "115.2");
        assert_eq!(mhz(4_700_000), "4700");
        assert_eq!(mhz(729_050), "729");
        assert_eq!(
            mhz_range(Some(800_000), None, false).as_deref(),
            Some("800-? MHz")
        );
        assert_eq!(mhz_range(None, None, true), None);
        assert_eq!(
            mhz_range(Some(800_000), Some(800_000), true).as_deref(),
            Some("800 MHz")
        );
    }
}
