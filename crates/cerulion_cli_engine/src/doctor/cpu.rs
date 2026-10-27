// SPDX-License-Identifier: AGPL-3.0-only
//! `cpu.governor`: the CPU frequency governor and frequency floor, reported
//! and never judged.
//!
//! Reads, for every `/sys/devices/system/cpu/cpu<N>/cpufreq/` directory,
//! `scaling_governor`, `scaling_min_freq` and `cpuinfo_max_freq` (kHz). The
//! result is always `info`: which governor suits a host depends on its power
//! budget, and Cerulion sets none.

// The decision and the sysfs reader compile and are tested on every host;
// only a Linux build calls them from `run`.
#![cfg_attr(not(target_os = "linux"), allow(dead_code))]

use std::collections::BTreeMap;
use std::path::Path;

use super::{Check, Context, Outcome, Reading, Status};

pub(super) const GOVERNOR: Check = Check {
    id: "cpu.governor",
    why: "a governor that lets a core's clock fall to its floor between messages adds wake-up \
          and run time to each message at low rates; Cerulion reports the setting and does \
          not change it.",
    network: false,
    run,
};

/// One CPU's cpufreq files as read; `None` for a file that is absent or not a
/// number.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub(super) struct CpuFreq {
    pub(super) governor: Option<String>,
    pub(super) min_khz: Option<u64>,
    pub(super) max_khz: Option<u64>,
}

/// The report over every CPU's reading.
pub(super) fn decide(cpus: &[CpuFreq]) -> Outcome {
    if cpus.is_empty() {
        return Outcome::new(
            Status::Info,
            "no cpufreq directory under /sys/devices/system/cpu, so there is no governor to read",
        )
        .reading("cpus", Reading::Int(0));
    }
    let mut governors: BTreeMap<&str, usize> = BTreeMap::new();
    for cpu in cpus {
        *governors
            .entry(cpu.governor.as_deref().unwrap_or("unknown"))
            .or_insert(0) += 1;
    }
    let governor_text: Vec<String> = governors
        .iter()
        .map(|(g, n)| format!("{g} on {n} CPU{}", if *n == 1 { "" } else { "s" }))
        .collect();
    let floor = cpus.iter().filter_map(|c| c.min_khz).min();
    let max = cpus.iter().filter_map(|c| c.max_khz).max();
    let mhz = |khz: u64| khz / 1000;
    let freq_text = match (floor, max) {
        (Some(f), Some(m)) => format!(
            "; frequency floor {} MHz of a {} MHz maximum",
            mhz(f),
            mhz(m)
        ),
        (Some(f), None) => format!("; frequency floor {} MHz", mhz(f)),
        (None, Some(m)) => format!("; maximum {} MHz", mhz(m)),
        (None, None) => String::new(),
    };
    let mut o = Outcome::new(
        Status::Info,
        format!("governor {}{freq_text}", governor_text.join(", ")),
    )
    .reading(
        "cpus",
        Reading::Int(u64::try_from(cpus.len()).unwrap_or(u64::MAX)),
    )
    .reading(
        "governors",
        Reading::Text(governors.keys().copied().collect::<Vec<_>>().join(",")),
    );
    if let Some(f) = floor {
        o = o.reading("min_freq_khz", Reading::Int(f));
    }
    if let Some(m) = max {
        o = o.reading("max_freq_khz", Reading::Int(m));
    }
    o
}

fn read_trimmed(path: &Path) -> Option<String> {
    std::fs::read_to_string(path)
        .ok()
        .map(|s| s.trim().to_string())
        .filter(|s| !s.is_empty())
}

/// Read every `cpu<N>/cpufreq` directory under `cpu_root`, in CPU order.
pub(super) fn read_cpus(cpu_root: &Path) -> Vec<CpuFreq> {
    let Ok(entries) = std::fs::read_dir(cpu_root) else {
        return Vec::new();
    };
    let mut indexed: Vec<(u32, CpuFreq)> = entries
        .filter_map(Result::ok)
        .filter_map(|e| {
            let name = e.file_name();
            let index: u32 = name.to_str()?.strip_prefix("cpu")?.parse().ok()?;
            let freq = e.path().join("cpufreq");
            if !freq.is_dir() {
                return None;
            }
            let num = |file: &str| read_trimmed(&freq.join(file)).and_then(|s| s.parse().ok());
            Some((
                index,
                CpuFreq {
                    governor: read_trimmed(&freq.join("scaling_governor")),
                    min_khz: num("scaling_min_freq"),
                    max_khz: num("cpuinfo_max_freq"),
                },
            ))
        })
        .collect();
    indexed.sort_by_key(|(i, _)| *i);
    indexed.into_iter().map(|(_, c)| c).collect()
}

#[cfg(target_os = "linux")]
fn run(_ctx: &Context) -> Outcome {
    decide(&read_cpus(Path::new("/sys/devices/system/cpu")))
}

#[cfg(not(target_os = "linux"))]
fn run(_ctx: &Context) -> Outcome {
    Outcome::not_applicable("the governor files are a Linux cpufreq interface")
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cpu(g: &str, min: u64, max: u64) -> CpuFreq {
        CpuFreq {
            governor: Some(g.to_string()),
            min_khz: Some(min),
            max_khz: Some(max),
        }
    }

    #[test]
    fn reports_the_governors_and_the_floor_as_info() {
        let o = decide(&[
            cpu("powersave", 800_000, 4_700_000),
            cpu("powersave", 400_000, 4_700_000),
            cpu("performance", 4_700_000, 4_700_000),
        ]);
        assert_eq!(o.status, Status::Info);
        assert_eq!(
            o.summary,
            "governor performance on 1 CPU, powersave on 2 CPUs; frequency floor 400 MHz of a \
             4700 MHz maximum"
        );
        assert_eq!(o.readings.get("min_freq_khz"), Some(&Reading::Int(400_000)));
        assert_eq!(o.fix, None);
    }

    #[test]
    fn no_cpufreq_is_info_too() {
        let o = decide(&[]);
        assert_eq!(o.status, Status::Info);
        assert_eq!(
            o.summary,
            "no cpufreq directory under /sys/devices/system/cpu, so there is no governor to read"
        );
    }

    #[test]
    fn a_missing_governor_file_reads_unknown() {
        let o = decide(&[CpuFreq::default()]);
        assert_eq!(o.summary, "governor unknown on 1 CPU");
    }

    #[test]
    fn read_cpus_reads_sysfs_shaped_directories_in_cpu_order() {
        let root = std::env::temp_dir().join(format!("doctor-cpu-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&root);
        for (i, g) in [(10, "schedutil"), (2, "performance")] {
            let d = root.join(format!("cpu{i}")).join("cpufreq");
            std::fs::create_dir_all(&d).unwrap();
            std::fs::write(d.join("scaling_governor"), format!("{g}\n")).unwrap();
            std::fs::write(d.join("scaling_min_freq"), "1000\n").unwrap();
        }
        // Not a CPU directory, and a CPU without cpufreq.
        std::fs::create_dir_all(root.join("cpufreq")).unwrap();
        std::fs::create_dir_all(root.join("cpu3")).unwrap();
        let cpus = read_cpus(&root);
        std::fs::remove_dir_all(&root).unwrap();
        assert_eq!(
            cpus,
            vec![
                CpuFreq {
                    governor: Some("performance".to_string()),
                    min_khz: Some(1000),
                    max_khz: None,
                },
                CpuFreq {
                    governor: Some("schedutil".to_string()),
                    min_khz: Some(1000),
                    max_khz: None,
                },
            ]
        );
    }
}
