// SPDX-License-Identifier: AGPL-3.0-only
//! Topic discovery verbs: `topic type`, `topic find`, `topic bw` and the
//! typed `topic list -t` listing.
//!
//! The decision functions here are pure (oracle-tested below); the observing
//! entry points reuse the same subscriber, scope and schema-naming ladder
//! `topic info` uses.

use crate::error::{CliError, CliResult};

/// One received frame as `topic bw` sees it: the publisher's wire stamp and
/// the payload size in bytes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BwFrame {
    /// The frame's wire `timestamp_ns` (the publisher's stamp, so a replayed
    /// stream measures the same bandwidth as the live one).
    pub timestamp_ns: u64,
    /// The payload size in bytes.
    pub size_bytes: u64,
}

/// What [`bandwidth`] computes over one window of frames.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BwStats {
    /// Bytes per second, rounded to the nearest byte.
    pub bytes_per_sec: u64,
    /// How many frames the window held.
    pub frames: usize,
    /// The mean payload size in bytes (rounded down).
    pub mean_size: u64,
    /// The smallest payload size in bytes.
    pub min_size: u64,
    /// The largest payload size in bytes.
    pub max_size: u64,
}

/// The bandwidth of a window of frames.
///
/// The rate is the bytes that arrived AFTER the earliest frame divided by the
/// time from the earliest stamp to the latest one: `n` frames span `n - 1`
/// intervals, so a steady stream of `S` bytes every `T` seconds measures
/// exactly `S / T`. `None` when the window cannot give a rate: no frame, one
/// frame, or every frame on one stamp (no time span). Integer arithmetic in
/// `u128`, so the largest sizes and spans never overflow; a rate above
/// `u64::MAX` saturates.
pub fn bandwidth(window: &[BwFrame]) -> Option<BwStats> {
    let first = window.iter().min_by_key(|f| f.timestamp_ns)?;
    let last_ns = window.iter().map(|f| f.timestamp_ns).max()?;
    let span_ns = u128::from(last_ns - first.timestamp_ns);
    if span_ns == 0 {
        return None;
    }
    let total: u128 = window.iter().map(|f| u128::from(f.size_bytes)).sum();
    let after_first = total - u128::from(first.size_bytes);
    let rate = (after_first * 1_000_000_000 + span_ns / 2) / span_ns;
    let frames = window.len();
    Some(BwStats {
        bytes_per_sec: u64::try_from(rate).unwrap_or(u64::MAX),
        frames,
        mean_size: u64::try_from(total / frames as u128).unwrap_or(u64::MAX),
        min_size: window.iter().map(|f| f.size_bytes).min().unwrap_or(0),
        max_size: window.iter().map(|f| f.size_bytes).max().unwrap_or(0),
    })
}

/// The decimal unit prefixes `topic bw` scales by (1 KB = 1000 B, as
/// `ros2 topic bw` prints them).
const UNIT_PREFIXES: [&str; 5] = ["", "K", "M", "G", "T"];

/// Format `value` bytes with `suffix` (`B` or `B/s`) on the largest decimal
/// unit that keeps the figure under 1000: `999 B/s`, `1.00 KB/s`,
/// `1.50 MB/s`. Below 1000 the figure is an exact integer; above, two
/// decimals rounded half up, and a value that rounds to 1000.00 moves up a
/// unit (`999999 B/s` is `1.00 MB/s`, never `1000.00 KB/s`). Terabytes is
/// the top unit.
pub fn format_bytes(value: u64, suffix: &str) -> String {
    if value < 1000 {
        return format!("{value} {suffix}");
    }
    let value = u128::from(value);
    let mut scale: u128 = 1;
    for (i, prefix) in UNIT_PREFIXES.iter().enumerate().skip(1) {
        scale *= 1000;
        let hundredths = (value * 100 + scale / 2) / scale;
        if hundredths < 100_000 || i == UNIT_PREFIXES.len() - 1 {
            return format!(
                "{}.{:02} {prefix}{suffix}",
                hundredths / 100,
                hundredths % 100
            );
        }
    }
    unreachable!("the loop returns on its last unit")
}

/// The report line `topic bw` prints for one window.
pub fn render_bw_line(stats: &BwStats) -> String {
    format!(
        "bandwidth: {}, mean: {}, min: {}, max: {}, window: {}",
        format_bytes(stats.bytes_per_sec, "B/s"),
        format_bytes(stats.mean_size, "B"),
        format_bytes(stats.min_size, "B"),
        format_bytes(stats.max_size, "B"),
        stats.frames
    )
}

/// Parse a message type name as `topic find` takes it and return its
/// canonical `package/Type` form.
///
/// Both the Cerulion form `geometry_msgs/Vector3` and the ROS 2 form
/// `geometry_msgs/msg/Vector3` are accepted. Each part is a non-empty run of
/// ASCII letters, digits and `_` that does not start with a digit. Anything
/// else is a validation error naming the input and the expected shape.
pub fn parse_type_name(input: &str) -> CliResult<String> {
    let parts: Vec<&str> = input.split('/').collect();
    let (package, ty) = match parts.as_slice() {
        [package, ty] | [package, "msg", ty] => (*package, *ty),
        _ => return Err(malformed_type(input)),
    };
    if !is_identifier(package) || !is_identifier(ty) {
        return Err(malformed_type(input));
    }
    Ok(format!("{package}/{ty}"))
}

fn is_identifier(part: &str) -> bool {
    let mut chars = part.chars();
    matches!(chars.next(), Some(c) if c.is_ascii_alphabetic() || c == '_')
        && chars.all(|c| c.is_ascii_alphanumeric() || c == '_')
}

fn malformed_type(input: &str) -> CliError {
    CliError::Validation(format!(
        "malformed type name '{}': expected <package>/<Type>, for example \
         geometry_msgs/Vector3",
        super::sanitize_display(input)
    ))
}

/// What a typed listing knows about one topic's message type.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TopicType {
    /// The type's `package/Type` name.
    Named(String),
    /// A frame arrived, but its schema hash names no built-in or workspace type.
    Unnamed(u64),
    /// No frame arrived in the wait, so the type is unknown.
    NoMessage,
}

impl std::fmt::Display for TopicType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TopicType::Named(name) => write!(f, "[{name}]"),
            TopicType::Unnamed(hash) => write!(f, "[0x{hash:016x}]"),
            TopicType::NoMessage => write!(f, "[unknown]"),
        }
    }
}

/// One local topic with its type.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TypedTopic {
    /// The topic name.
    pub name: String,
    /// What is known about its type.
    pub ty: TopicType,
}

/// Render the LOCAL section of `topic list -t`: the same section
/// `render_local_topics_section` prints, with each row's type in brackets
/// after the topic name (`/robot/cmd  [geometry_msgs/Vector3]`). The topic
/// name stays the first token of the row; the `internal` marker of
/// `--all` follows the type. Internal topics are hidden and counted exactly
/// as in the untyped listing.
pub fn render_local_typed_topics_section(local: &[TypedTopic], show_all: bool) -> String {
    let mut rows = String::new();
    let mut hidden = 0usize;
    for topic in local {
        let internal = super::is_internal_topic(&topic.name);
        if internal && !show_all {
            hidden += 1;
            continue;
        }
        rows.push_str(&topic.name);
        rows.push_str("  ");
        rows.push_str(&topic.ty.to_string());
        if internal {
            rows.push_str("  ");
            rows.push_str(super::INTERNAL_TOPIC_MARKER);
        }
        rows.push('\n');
    }
    let mut out = if rows.is_empty() {
        String::from("No active local topics.\n")
    } else {
        format!("TOPIC\n{rows}")
    };
    if hidden > 0 {
        out.push_str(&super::render_hidden_internal_count(hidden));
    }
    out
}

/// The topics whose type is `type_name` (canonical `package/Type`), sorted
/// by name with duplicates removed. A topic with an unknown or unnamed type
/// never matches.
pub fn topics_of_type<'a>(topics: &'a [TypedTopic], type_name: &str) -> Vec<&'a str> {
    let mut found: Vec<&str> = topics
        .iter()
        .filter(|t| matches!(&t.ty, TopicType::Named(name) if name == type_name))
        .map(|t| t.name.as_str())
        .collect();
    found.sort_unstable();
    found.dedup();
    found
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(timestamp_ns: u64, size_bytes: u64) -> BwFrame {
        BwFrame {
            timestamp_ns,
            size_bytes,
        }
    }

    const SEC: u64 = 1_000_000_000;

    #[test]
    fn an_empty_window_has_no_bandwidth() {
        assert_eq!(bandwidth(&[]), None);
    }

    #[test]
    fn one_frame_has_no_bandwidth() {
        assert_eq!(bandwidth(&[frame(5, 100)]), None);
    }

    #[test]
    fn frames_on_one_stamp_have_no_bandwidth() {
        assert_eq!(bandwidth(&[frame(7, 10), frame(7, 20)]), None);
    }

    #[test]
    fn a_steady_rate_measures_size_over_period() {
        // 24 bytes every 50 ms is 480 B/s.
        let window: Vec<BwFrame> = (0..10).map(|i| frame(i * 50_000_000, 24)).collect();
        assert_eq!(
            bandwidth(&window),
            Some(BwStats {
                bytes_per_sec: 480,
                frames: 10,
                mean_size: 24,
                min_size: 24,
                max_size: 24,
            })
        );
    }

    #[test]
    fn a_burst_counts_every_byte_after_the_first_frame() {
        // Three frames in the first 2 ms, then one more at 1 s:
        // 200 + 300 + 400 bytes after the first over 1 s.
        let window = [
            frame(0, 100),
            frame(1_000_000, 200),
            frame(2_000_000, 300),
            frame(SEC, 400),
        ];
        let stats = bandwidth(&window).unwrap();
        assert_eq!(stats.bytes_per_sec, 900);
        assert_eq!(stats.mean_size, 250);
        assert_eq!(stats.min_size, 100);
        assert_eq!(stats.max_size, 400);
    }

    #[test]
    fn the_earliest_stamp_starts_the_span_whatever_the_arrival_order() {
        let window = [frame(SEC, 400), frame(0, 100), frame(2 * SEC, 600)];
        assert_eq!(bandwidth(&window).unwrap().bytes_per_sec, 500);
    }

    #[test]
    fn a_rate_rounds_to_the_nearest_byte() {
        // 2 bytes over 3 s is 0.67 B/s, which rounds to 1.
        assert_eq!(
            bandwidth(&[frame(0, 1), frame(3 * SEC, 2)])
                .unwrap()
                .bytes_per_sec,
            1
        );
        // 1 byte over 3 s is 0.33 B/s, which rounds to 0.
        assert_eq!(
            bandwidth(&[frame(0, 1), frame(3 * SEC, 1)])
                .unwrap()
                .bytes_per_sec,
            0
        );
    }

    #[test]
    fn the_largest_sizes_and_spans_do_not_overflow() {
        // u64::MAX bytes in 1 ns saturates instead of wrapping.
        let huge = bandwidth(&[frame(0, u64::MAX), frame(1, u64::MAX)]).unwrap();
        assert_eq!(huge.bytes_per_sec, u64::MAX);
        assert_eq!(huge.mean_size, u64::MAX);
        // u64::MAX bytes over u64::MAX ns is 1e9 B/s.
        let long = bandwidth(&[frame(0, 0), frame(u64::MAX, u64::MAX)]).unwrap();
        assert_eq!(long.bytes_per_sec, 1_000_000_000);
    }

    #[test]
    fn bytes_below_one_thousand_print_as_an_integer() {
        assert_eq!(format_bytes(0, "B/s"), "0 B/s");
        assert_eq!(format_bytes(999, "B/s"), "999 B/s");
    }

    #[test]
    fn bytes_scale_by_one_thousand_with_two_decimals() {
        assert_eq!(format_bytes(1000, "B/s"), "1.00 KB/s");
        assert_eq!(format_bytes(1500, "B"), "1.50 KB");
        assert_eq!(format_bytes(1_234_567, "B/s"), "1.23 MB/s");
        assert_eq!(format_bytes(1_235_000, "B/s"), "1.24 MB/s");
        assert_eq!(format_bytes(2_000_000_000, "B/s"), "2.00 GB/s");
    }

    #[test]
    fn a_figure_that_rounds_to_one_thousand_moves_up_a_unit() {
        assert_eq!(format_bytes(999_994, "B/s"), "999.99 KB/s");
        assert_eq!(format_bytes(999_995, "B/s"), "1.00 MB/s");
        assert_eq!(format_bytes(999_999, "B/s"), "1.00 MB/s");
    }

    #[test]
    fn terabytes_is_the_top_unit() {
        assert_eq!(format_bytes(u64::MAX, "B/s"), "18446744.07 TB/s");
    }

    #[test]
    fn the_bw_line_names_rate_sizes_and_window() {
        let stats = BwStats {
            bytes_per_sec: 480,
            frames: 10,
            mean_size: 24,
            min_size: 20,
            max_size: 1500,
        };
        assert_eq!(
            render_bw_line(&stats),
            "bandwidth: 480 B/s, mean: 24 B, min: 20 B, max: 1.50 KB, window: 10"
        );
    }

    #[test]
    fn a_type_name_is_package_slash_type() {
        assert_eq!(
            parse_type_name("geometry_msgs/Vector3").unwrap(),
            "geometry_msgs/Vector3"
        );
        assert_eq!(parse_type_name("_p/T_2").unwrap(), "_p/T_2");
    }

    #[test]
    fn the_ros_2_msg_form_names_the_same_type() {
        assert_eq!(
            parse_type_name("geometry_msgs/msg/Vector3").unwrap(),
            "geometry_msgs/Vector3"
        );
    }

    #[test]
    fn a_malformed_type_name_is_refused_with_its_shape() {
        for bad in [
            "",
            "Vector3",
            "geometry_msgs/",
            "/Vector3",
            "a/b/c",
            "a/srv/B",
            "geometry_msgs/Vec tor3",
            "1pkg/Type",
            "pkg/Ty-pe",
            "pkg//Type",
        ] {
            let err = parse_type_name(bad).expect_err(bad).to_string();
            assert!(
                err.contains(&format!("malformed type name '{bad}'"))
                    && err.contains("expected <package>/<Type>, for example geometry_msgs/Vector3"),
                "{bad:?}: {err}"
            );
        }
    }

    #[test]
    fn a_malformed_type_name_cannot_inject_terminal_escapes() {
        let err = parse_type_name("a\u{1b}[2J").unwrap_err().to_string();
        assert!(!err.contains('\u{1b}'), "{err:?}");
        assert!(err.contains("'a\u{FFFD}[2J'"), "{err:?}");
    }

    fn typed(name: &str, ty: TopicType) -> TypedTopic {
        TypedTopic {
            name: name.to_string(),
            ty,
        }
    }

    fn named(name: &str) -> TopicType {
        TopicType::Named(name.to_string())
    }

    #[test]
    fn a_typed_listing_puts_the_type_after_the_topic() {
        let rows = [
            typed("/demo/a/cmd", named("geometry_msgs/Vector3")),
            typed("/demo/b/raw", TopicType::Unnamed(0xab)),
            typed("/demo/c/idle", TopicType::NoMessage),
        ];
        assert_eq!(
            render_local_typed_topics_section(&rows, false),
            "TOPIC\n\
             /demo/a/cmd  [geometry_msgs/Vector3]\n\
             /demo/b/raw  [0x00000000000000ab]\n\
             /demo/c/idle  [unknown]\n"
        );
    }

    #[test]
    fn a_typed_listing_hides_and_counts_internal_topics_like_the_plain_one() {
        let rows = [
            typed("/bagd/status", named("std_msgs/String")),
            typed("/demo/a/cmd", named("geometry_msgs/Vector3")),
        ];
        assert_eq!(
            render_local_typed_topics_section(&rows, false),
            "TOPIC\n/demo/a/cmd  [geometry_msgs/Vector3]\n\
             1 internal topic hidden (--all shows it)\n"
        );
        assert_eq!(
            render_local_typed_topics_section(&rows, true),
            "TOPIC\n/bagd/status  [std_msgs/String]  internal\n\
             /demo/a/cmd  [geometry_msgs/Vector3]\n"
        );
    }

    #[test]
    fn an_empty_typed_listing_says_so() {
        assert_eq!(
            render_local_typed_topics_section(&[], false),
            "No active local topics.\n"
        );
        assert_eq!(
            render_local_typed_topics_section(
                &[typed("/bagd/status", TopicType::NoMessage)],
                false
            ),
            "No active local topics.\n1 internal topic hidden (--all shows it)\n"
        );
    }

    #[test]
    fn find_lists_only_topics_of_that_type_sorted_once_each() {
        let rows = [
            typed("/z/cmd", named("geometry_msgs/Vector3")),
            typed("/a/name", named("std_msgs/String")),
            typed("/a/cmd", named("geometry_msgs/Vector3")),
            typed("/a/cmd", named("geometry_msgs/Vector3")),
            typed("/m/raw", TopicType::Unnamed(1)),
            typed("/m/idle", TopicType::NoMessage),
        ];
        assert_eq!(
            topics_of_type(&rows, "geometry_msgs/Vector3"),
            vec!["/a/cmd", "/z/cmd"]
        );
        assert_eq!(topics_of_type(&rows, "std_msgs/String"), vec!["/a/name"]);
    }

    #[test]
    fn find_of_an_unknown_type_is_empty() {
        let rows = [typed("/a/cmd", named("geometry_msgs/Vector3"))];
        assert!(topics_of_type(&rows, "geometry_msgs/Twist").is_empty());
        assert!(topics_of_type(&[], "geometry_msgs/Vector3").is_empty());
    }
}
