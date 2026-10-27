// SPDX-License-Identifier: AGPL-3.0-only
//! `limits.open-files`: the open-file limit a graph starts under.
//!
//! Reads the limit the same way `graph run` does: it calls
//! [`cerulion_core::fd_limit::raise_open_file_limit`], which raises this
//! process's own soft limit to the ceiling a Cerulion process may reach, and
//! reports the limits in force afterwards. Inside a workspace it estimates
//! every graph under `graphs/` with
//! [`cerulion_core::fd_limit::open_file_shortfall`], the estimate `graph run`
//! refuses a graph on. When the raise reports no limit, the check reads the
//! current limit instead (`graph run` then refuses nothing), and
//! `CERULION_OPEN_FILE_CHECK=0` turns `graph run`'s refusal into a warning.

use std::path::Path;

use cerulion_core::fd_limit::{GraphPorts, OpenFileLimit, OpenFileShortfall, RemedyPlatform};

use super::{Check, Context, Outcome, Reading, Status};

pub(super) const OPEN_FILES: Check = Check {
    id: "limits.open-files",
    why: "iceoryx2 keeps a file open for every port, so the process that starts a graph holds \
          files in proportion to its ports; past the open-file limit port creation fails with \
          errors that do not name the limit, and `graph run` refuses a graph it estimates \
          past it.",
    network: false,
    run,
};

/// One workspace graph and the ports it was estimated from.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) struct GraphEstimate {
    pub(super) name: String,
    pub(super) ports: GraphPorts,
}

/// The decision over the readings: `limit` is `None` when the limit could not
/// be read (`errno` names why), `graphs` are the workspace's parsed graphs.
pub(super) fn decide(
    limit: Result<OpenFileLimit, i32>,
    select_ceiling: Option<u64>,
    platform: RemedyPlatform,
    graphs: &[GraphEstimate],
    unparsed_graphs: usize,
) -> Outcome {
    let limit = match limit {
        Ok(limit) => limit,
        Err(errno) => {
            return Outcome::new(
                Status::Warn,
                format!("could not read the open-file limit (getrlimit errno {errno})"),
            )
            .with_fix("run `ulimit -n` in the shell that starts the graph to see the limit")
            .reading("errno", Reading::Int(u64::try_from(errno).unwrap_or(0)));
        }
    };
    let shortfalls: Vec<(&GraphEstimate, OpenFileShortfall)> = graphs
        .iter()
        .filter_map(|g| {
            cerulion_core::fd_limit::open_file_shortfall(g.ports, limit, select_ceiling, platform)
                .map(|s| (g, s))
        })
        .collect();
    let base = |status, summary: String| {
        let mut o = Outcome::new(status, summary)
            .reading("soft", Reading::Int(limit.soft))
            .reading("hard", Reading::Int(limit.hard))
            .reading("graphs_estimated", Reading::Int(len(graphs.len())))
            .reading("graphs_unparsed", Reading::Int(len(unparsed_graphs)));
        if let Some(cap) = select_ceiling {
            o = o.reading("select_ceiling", Reading::Int(cap));
        }
        o
    };
    let shown = |v: u64| {
        if v == u64::MAX {
            "unlimited".to_string()
        } else {
            v.to_string()
        }
    };
    match shortfalls.as_slice() {
        [] => {
            let scope = match graphs.len() {
                0 => "no workspace graph to estimate".to_string(),
                1 => "the one workspace graph fits".to_string(),
                n => format!("all {n} workspace graphs fit"),
            };
            let summary = format!(
                "soft limit {} after the raise a graph run makes (hard {}); {scope}",
                shown(limit.soft),
                shown(limit.hard)
            );
            if unparsed_graphs > 0 {
                base(
                    Status::Warn,
                    format!(
                        "{summary}, but {} under graphs/ did not parse and were not \
                         estimated",
                        if unparsed_graphs == 1 {
                            "1 graph file".to_string()
                        } else {
                            format!("{unparsed_graphs} graph files")
                        }
                    ),
                )
                .with_fix(
                    "run `cerulion graph validate <NAME>` on each to see why it does not parse",
                )
            } else {
                base(Status::Pass, summary)
            }
        }
        [(_, first), ..] => {
            let names: Vec<&str> = shortfalls.iter().map(|(g, _)| g.name.as_str()).collect();
            let worst = shortfalls.iter().map(|(_, s)| s).fold(first, |worst, s| {
                if s.needed > worst.needed {
                    s
                } else {
                    worst
                }
            });
            base(
                Status::Fail,
                format!(
                    "`graph run` refuses {} of the workspace graphs at this limit: {}",
                    names.len(),
                    names.join(", ")
                ),
            )
            .with_fix(worst.to_string())
            .reading("largest_need", Reading::Int(worst.needed))
        }
    }
}

fn len(n: usize) -> u64 {
    u64::try_from(n).unwrap_or(u64::MAX)
}

/// The ports of every graph file under `graphs_dir`, by file stem, and the
/// count of graph files that did not parse.
pub(super) fn workspace_graphs(graphs_dir: &Path) -> (Vec<GraphEstimate>, usize) {
    let Ok(entries) = std::fs::read_dir(graphs_dir) else {
        return (Vec::new(), 0);
    };
    let mut paths: Vec<_> = entries
        .filter_map(Result::ok)
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|e| e == "yaml"))
        // A bridge sidecar (`<name>.bridge.yaml`) is not a graph of its own.
        .filter(|p| {
            !p.file_name()
                .and_then(|n| n.to_str())
                .is_some_and(|n| n.ends_with(".bridge.yaml"))
        })
        .collect();
    paths.sort();
    let mut graphs = Vec::new();
    let mut unparsed = 0;
    for path in paths {
        let parsed = std::fs::read_to_string(&path)
            .ok()
            .and_then(|text| cerulion_core::graph::parse_graph_raw(&text).ok());
        match parsed {
            Some(config) => graphs.push(GraphEstimate {
                name: path
                    .file_stem()
                    .and_then(|s| s.to_str())
                    .unwrap_or_default()
                    .to_string(),
                ports: GraphPorts {
                    nodes: config.nodes.len(),
                    outputs: config.nodes.iter().map(|n| n.outputs.len()).sum(),
                    inputs: config.nodes.iter().map(|n| n.inputs.len()).sum(),
                },
            }),
            None => unparsed += 1,
        }
    }
    (graphs, unparsed)
}

#[cfg(unix)]
fn run(ctx: &Context) -> Outcome {
    let raised = cerulion_core::fd_limit::raise_open_file_limit().limit();
    let limit = match raised {
        Some(limit) => Ok(limit),
        None => cerulion_core::fd_limit::current_open_file_limit(),
    };
    let (graphs, unparsed) = ctx
        .workspace_root
        .as_deref()
        .map(|root| workspace_graphs(&root.join("graphs")))
        .unwrap_or_default();
    decide(
        limit,
        cerulion_core::fd_limit::select_ceiling(),
        RemedyPlatform::current(),
        &graphs,
        unparsed,
    )
}

#[cfg(not(unix))]
fn run(_ctx: &Context) -> Outcome {
    Outcome::not_applicable("this platform has no RLIMIT_NOFILE")
}

#[cfg(test)]
mod tests {
    use super::*;

    fn graph(name: &str, outputs: usize, inputs: usize) -> GraphEstimate {
        GraphEstimate {
            name: name.to_string(),
            ports: GraphPorts {
                nodes: outputs,
                outputs,
                inputs,
            },
        }
    }

    const LIMIT_1024: OpenFileLimit = OpenFileLimit {
        soft: 1024,
        hard: 1024,
    };

    #[test]
    fn passes_when_every_graph_fits() {
        // 40 + 10 x 14 + 10 x 22 = 400 files, under 1024.
        let o = decide(
            Ok(LIMIT_1024),
            None,
            RemedyPlatform::Linux,
            &[graph("small", 10, 10)],
            0,
        );
        assert_eq!(o.status, Status::Pass);
        assert_eq!(o.fix, None);
        assert_eq!(
            o.summary,
            "soft limit 1024 after the raise a graph run makes (hard 1024); the one workspace \
             graph fits"
        );
    }

    #[test]
    fn fails_naming_each_graph_past_the_limit_and_the_largest_need() {
        // wide: 40 + 40 x 14 + 40 x 22 = 1480; wider: 40 + 50 x 14 + 50 x 22 = 1840.
        let o = decide(
            Ok(LIMIT_1024),
            None,
            RemedyPlatform::Linux,
            &[
                graph("small", 10, 10),
                graph("wide", 40, 40),
                graph("wider", 50, 50),
            ],
            0,
        );
        assert_eq!(o.status, Status::Fail);
        assert_eq!(
            o.summary,
            "`graph run` refuses 2 of the workspace graphs at this limit: wide, wider"
        );
        assert_eq!(o.readings.get("largest_need"), Some(&Reading::Int(1840)));
        let fix = o.fix.unwrap();
        assert!(fix.contains("ulimit -Hn 1840"), "{fix}");
    }

    #[test]
    fn the_select_ceiling_fails_a_graph_the_soft_limit_allows() {
        let big = OpenFileLimit {
            soft: 10_240,
            hard: 10_240,
        };
        let o = decide(
            Ok(big),
            Some(1024),
            RemedyPlatform::MacOs,
            &[graph("wide", 40, 40)],
            0,
        );
        assert_eq!(o.status, Status::Fail);
        assert!(o.fix.unwrap().contains("Run this graph on Linux"));
    }

    #[test]
    fn passes_outside_a_workspace_and_reports_unlimited() {
        let o = decide(
            Ok(OpenFileLimit {
                soft: 4096,
                hard: u64::MAX,
            }),
            None,
            RemedyPlatform::Linux,
            &[],
            0,
        );
        assert_eq!(o.status, Status::Pass);
        assert_eq!(
            o.summary,
            "soft limit 4096 after the raise a graph run makes (hard unlimited); no workspace \
             graph to estimate"
        );
    }

    #[test]
    fn a_graph_file_that_did_not_parse_warns_instead_of_passing() {
        let o = decide(
            Ok(LIMIT_1024),
            None,
            RemedyPlatform::Linux,
            &[graph("a", 1, 1), graph("b", 1, 1)],
            2,
        );
        assert_eq!(o.status, Status::Warn);
        assert_eq!(
            o.summary,
            "soft limit 1024 after the raise a graph run makes (hard 1024); all 2 workspace \
             graphs fit, but 2 graph files under graphs/ did not parse and were not estimated"
        );
        assert_eq!(o.readings.get("graphs_unparsed"), Some(&Reading::Int(2)));
    }

    #[test]
    fn an_unreadable_limit_warns_with_the_errno() {
        let o = decide(Err(1), None, RemedyPlatform::Linux, &[], 0);
        assert_eq!(o.status, Status::Warn);
        assert_eq!(
            o.summary,
            "could not read the open-file limit (getrlimit errno 1)"
        );
        assert_eq!(o.readings.get("errno"), Some(&Reading::Int(1)));
        assert!(o.fix.is_some());
    }

    #[test]
    fn workspace_graphs_reads_each_graph_and_counts_the_unparsed() {
        let dir = std::env::temp_dir().join(format!("doctor-graphs-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(
            dir.join("pair.yaml"),
            "nodes:\n- id: a\n  type: a\n  inputs: []\n  outputs:\n  - name: out\n    schema: \
             std_msgs/Int32\n- id: b\n  type: b\n  inputs:\n  - name: in\n    source: a/out\n  \
             outputs: []\n",
        )
        .unwrap();
        std::fs::write(dir.join("broken.yaml"), "nodes: [").unwrap();
        std::fs::write(dir.join("pair.bridge.yaml"), "not a graph").unwrap();
        let (graphs, unparsed) = workspace_graphs(&dir);
        std::fs::remove_dir_all(&dir).unwrap();
        assert_eq!(unparsed, 1);
        assert_eq!(
            graphs,
            vec![GraphEstimate {
                name: "pair".to_string(),
                ports: GraphPorts {
                    nodes: 2,
                    outputs: 1,
                    inputs: 1,
                },
            }]
        );
    }
}
