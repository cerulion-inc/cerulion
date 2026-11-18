// SPDX-License-Identifier: AGPL-3.0-only
//! Period-park: the live loop's opt-in kernel sleep between expected events
//! (`CERULION_PERIOD_PARK=1`, default off).
//!
//! The monitor-wait park holds its core between steps: a `UMWAIT` or `WFE`
//! parked thread is running to the OS. With the period-park on, a park that has
//! something to expect (the next Period deadline, or the next arrival a learner
//! predicts from earlier data wakes) sleeps in the kernel in short naps until a
//! measured lead before it, then runs the monitor-wait park only for the window
//! that remains. A ring on the armed doorbell still ends a nap at once through
//! the doorbell wake word. The nap is short ([`NAP`]) because a core left idle
//! for a whole period comes back at a lower clock, and the step after the wake
//! then runs slower.
//!
//! Everything here decides WHEN the loop re-polls, never WHAT fires
//! (Principle #7): the step still reads every frame off the iceoryx2 queue. The
//! arithmetic is pure and integer-only: the same inputs give the same state.

use std::sync::OnceLock;
use std::time::{Duration, Instant};

/// The longest single kernel sleep. A ring on an input topic other than the
/// armed one is seen at the end of the nap it lands in, at the latest.
pub(crate) const NAP: Duration = Duration::from_micros(100);

/// A kernel sleep shorter than this is not worth its wake: the park runs instead.
const MIN_KERNEL_SLEEP_NS: u64 = 20_000;

/// Smallest and largest kernel lead (ns).
const LEAD_MIN_NS: u64 = 10_000;
const LEAD_MAX_NS: u64 = 400_000;
/// Fixed margin added to the measured overshoot (ns).
const LEAD_MARGIN_NS: u64 = 5_000;
/// Overshoot estimate before any sleep has been measured (ns).
const LEAD_INIT_MEAN_NS: u64 = 60_000;
const LEAD_INIT_DEV_NS: u64 = 20_000;

/// Data gaps shorter than this belong to one burst and are not a period (ns).
const MIN_GAP_NS: u64 = 200_000;
/// Data gaps longer than this are an idle spell (ns).
const MAX_GAP_NS: u64 = 100_000_000;
/// In-range gaps needed before arrivals are predicted.
const REGULAR_AFTER: u32 = 8;
/// Smallest and largest half-width of a predicted data window (ns).
const MIN_WINDOW_NS: u64 = 20_000;
const MAX_WINDOW_NS: u64 = 100_000;
/// Longest predicted window, early widening included (ns).
const MAX_PARK_NS: u64 = 2 * MAX_WINDOW_NS;

/// What `CERULION_PERIOD_PARK` asks for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Mode {
    /// Unset, empty or `0`.
    Off,
    /// `1`.
    On,
    /// Anything else: off, and said so once.
    Unrecognized,
}

impl Mode {
    pub(crate) fn on(self) -> bool {
        matches!(self, Mode::On)
    }
}

/// PURE: classify a raw `CERULION_PERIOD_PARK` value.
pub(crate) fn classify(raw: Option<&str>) -> Mode {
    match raw {
        None | Some("") | Some("0") => Mode::Off,
        Some("1") => Mode::On,
        Some(_) => Mode::Unrecognized,
    }
}

/// The process's `CERULION_PERIOD_PARK` mode, read once.
pub(crate) fn env_mode() -> Mode {
    static MODE: OnceLock<Mode> = OnceLock::new();
    *MODE.get_or_init(|| classify(std::env::var("CERULION_PERIOD_PARK").ok().as_deref()))
}

/// What decides whether the period-park applies to one park block.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Gate {
    /// The switch is on.
    pub(crate) on: bool,
    /// The monitor-wait park is the live loop's idle.
    pub(crate) park_active: bool,
    /// The host carries a real CPU monitor-wait primitive.
    pub(crate) primitive: bool,
    /// The runtime waits on a barrier word.
    pub(crate) barrier: bool,
    /// The runtime produces on a cross-process `block` edge (a credit word it
    /// may wait on).
    pub(crate) credit_edges: bool,
    /// The park polls an external fd source.
    pub(crate) fd_sources: bool,
    /// An input topic is written from outside this runtime, whatever the
    /// doorbell policy.
    pub(crate) outside_inputs: bool,
    /// The doorbell armed a line for such an input.
    pub(crate) armed_line: bool,
    /// The doorbell wake word can end a kernel nap on that line.
    pub(crate) data_wake: bool,
}

/// Why a requested period-park does not engage: the first [`Gate`] term that
/// fails, in declaration order: first the host's CPU, then the run's park
/// policy, then this runtime's wiring, where a wake the park waits for cannot
/// end a kernel nap.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum NotEngaged {
    /// No CPU monitor-wait primitive: the park already sleeps.
    NoMonitorWaitPrimitive,
    /// The monitor-wait park is off.
    ParkOff,
    /// A lockstep barrier rank: a barrier arrival cannot end a nap.
    Barrier,
    /// A producer on a cross-process `block` edge: a credit drain cannot.
    CreditEdge,
    /// The park polls an external fd source: its readiness cannot.
    FdSource,
    /// An input written from outside the runtime has no doorbell wake word.
    NoDataWake,
}

impl NotEngaged {
    /// The `reason` field value.
    pub(crate) fn as_str(self) -> &'static str {
        match self {
            NotEngaged::NoMonitorWaitPrimitive => "no_monitor_wait_primitive",
            NotEngaged::ParkOff => "park_off",
            NotEngaged::Barrier => "barrier",
            NotEngaged::CreditEdge => "credit_edge",
            NotEngaged::FdSource => "fd_source",
            NotEngaged::NoDataWake => "no_data_wake",
        }
    }
}

/// PURE: the first gate term that keeps the period-park off, in the order of
/// [`NotEngaged`], or `None` when every term holds. The switch is not a term:
/// with it off there is nothing to refuse.
pub(crate) fn refusal(g: Gate) -> Option<NotEngaged> {
    if !g.primitive {
        Some(NotEngaged::NoMonitorWaitPrimitive)
    } else if !g.park_active {
        Some(NotEngaged::ParkOff)
    } else if g.barrier {
        Some(NotEngaged::Barrier)
    } else if g.credit_edges {
        Some(NotEngaged::CreditEdge)
    } else if g.fd_sources {
        Some(NotEngaged::FdSource)
    } else if g.outside_inputs && !(g.armed_line && g.data_wake) {
        Some(NotEngaged::NoDataWake)
    } else {
        None
    }
}

/// PURE: does the period-park apply? The switch is on and no term refuses it:
/// only where every wake the park waits for can end a kernel nap, or has none.
pub(crate) fn admits(g: Gate) -> bool {
    g.on && refusal(g).is_none()
}

/// Say whether a requested period-park engages, at a live-loop entry: the
/// `live loop period park on` line when it does, and when it does not, one
/// WARN naming the first refusing term, at most once per `warned` latch.
/// Logs nothing with the switch off. Returns whether it engages.
pub(crate) fn announce(g: Gate, warned: &mut bool, graph: &str) -> bool {
    if !g.on {
        return false;
    }
    match refusal(g) {
        None => {
            tracing::info!(
                graph = %graph,
                armed_line = g.armed_line,
                data_wake = g.data_wake,
                "live loop period park on"
            );
            true
        }
        Some(reason) => {
            if !std::mem::replace(warned, true) {
                tracing::warn!(
                    graph = %graph,
                    reason = reason.as_str(),
                    target_arch = std::env::consts::ARCH,
                    "period park requested but not engaged"
                );
            }
            false
        }
    }
}

fn ewma(old: u64, sample: u64, shift: u32) -> u64 {
    if sample >= old {
        old + ((sample - old) >> shift)
    } else {
        old - ((old - sample) >> shift)
    }
}

/// How early a kernel sleep must end so the core is in the monitor-wait park by
/// the target: the measured timer overshoot, as a mean plus four deviations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct KernelLead {
    mean_ns: u64,
    dev_ns: u64,
}

impl Default for KernelLead {
    fn default() -> Self {
        Self {
            mean_ns: LEAD_INIT_MEAN_NS,
            dev_ns: LEAD_INIT_DEV_NS,
        }
    }
}

impl KernelLead {
    /// Fold in one measured overshoot: how long after its requested end a
    /// kernel sleep that ran to its end returned.
    pub(crate) fn observe(&mut self, overshoot_ns: u64) {
        let o = overshoot_ns.min(2 * LEAD_MAX_NS);
        let err = o.abs_diff(self.mean_ns);
        self.mean_ns = ewma(self.mean_ns, o, 3);
        self.dev_ns = ewma(self.dev_ns, err, 2);
    }

    pub(crate) fn lead_ns(&self) -> u64 {
        self.mean_ns
            .saturating_add(self.dev_ns.saturating_mul(4))
            .saturating_add(LEAD_MARGIN_NS)
            .clamp(LEAD_MIN_NS, LEAD_MAX_NS)
    }
}

/// Learns the period and phase of data wakes, to predict the next one.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub(crate) struct ArrivalLearner {
    last_ns: Option<u64>,
    period_ns: u64,
    dev_ns: u64,
    run: u32,
    early_ns: u64,
}

impl ArrivalLearner {
    fn forget(&mut self) {
        self.period_ns = 0;
        self.dev_ns = 0;
        self.run = 0;
        self.early_ns = 0;
    }

    /// Fold in a data wake at `now_ns`. Returns `false` for a later member of
    /// the same burst (a gap shorter than a period), which changes nothing.
    pub(crate) fn observe(&mut self, now_ns: u64) -> bool {
        let Some(prev) = self.last_ns else {
            self.last_ns = Some(now_ns);
            return true;
        };
        if now_ns <= prev {
            self.forget();
            self.last_ns = Some(now_ns);
            return true;
        }
        let gap = now_ns - prev;
        if gap < MIN_GAP_NS {
            return false;
        }
        self.last_ns = Some(now_ns);
        if gap > MAX_GAP_NS {
            self.forget();
            return true;
        }
        if self.period_ns == 0 {
            self.period_ns = gap;
            self.run = 1;
            return true;
        }
        let err = self.period_ns.abs_diff(gap);
        let tolerance = 3 * self.dev_ns.max(self.period_ns / 16);
        if self.run >= REGULAR_AFTER && err > tolerance {
            self.forget();
            self.period_ns = gap;
            self.run = 1;
            return true;
        }
        self.period_ns = ewma(self.period_ns, gap, 3);
        self.dev_ns = ewma(self.dev_ns, err, 3);
        self.run = self.run.saturating_add(1);
        true
    }

    pub(crate) fn regular(&self) -> bool {
        self.run >= REGULAR_AFTER
            && self.dev_ns.saturating_mul(8) <= self.period_ns
            && self.dev_ns.saturating_mul(4) <= MAX_WINDOW_NS
    }

    /// The predicted window's half-width. Read only while [`Self::regular`]
    /// holds, so it never exceeds half a period: there four deviations are at
    /// most half the period, and the period is at least [`MIN_GAP_NS`], ten
    /// times [`MIN_WINDOW_NS`].
    fn half_width_ns(&self) -> u64 {
        self.dev_ns.saturating_mul(4).max(MIN_WINDOW_NS)
    }

    /// The next predicted window `(open_ns, close_ns)` that has not closed by
    /// `now_ns`, or `None` while the arrivals are not regular.
    pub(crate) fn predict(&self, now_ns: u64) -> Option<(u64, u64)> {
        if !self.regular() {
            return None;
        }
        let last = self.last_ns?;
        let w = self.half_width_ns();
        let early = self
            .early_ns
            .min(MAX_PARK_NS.saturating_sub(w.saturating_mul(2)));
        let mut next = last.saturating_add(self.period_ns);
        if next.saturating_add(w) <= now_ns {
            // Beats were missed: step to the first beat whose window is open.
            let behind = now_ns - next.saturating_add(w);
            next = next.saturating_add((behind / self.period_ns + 1) * self.period_ns);
        }
        Some((next.saturating_sub(w + early), next.saturating_add(w)))
    }

    /// The predicted arrival was seen while the loop slept in the kernel before
    /// its window: open the next window earlier, doubling per such arrival.
    pub(crate) fn arrived_before_window(&mut self) {
        if !self.regular() {
            return;
        }
        let room = MAX_PARK_NS.saturating_sub(self.half_width_ns().saturating_mul(2));
        self.early_ns = self.early_ns.saturating_mul(2).max(MIN_WINDOW_NS).min(room);
    }

    /// The monitor-wait park saw the arrival: the noted time is the ring time.
    pub(crate) fn caught(&mut self) {
        self.early_ns = 0;
    }
}

/// How the wait that preceded a wake was spent.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum LastWait {
    /// No wait yet in this block (a wake already pending at entry).
    None,
    /// A kernel sleep before a predicted data window opened.
    KernelBeforeWindow,
    /// A kernel sleep with no data window ahead of it.
    KernelOther,
    /// The CPU monitor-wait park.
    Hot,
}

/// What the block should do at `now`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Phase {
    /// Run the monitor-wait park.
    Hot,
    /// Sleep in the kernel until `until` (in naps of at most [`NAP`]).
    Kernel { until: Instant, before_window: bool },
}

/// One block's plan, fixed at its entry.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Plan {
    /// The block's own deadline (the timer target).
    pub(crate) deadline: Instant,
    /// The predicted data window, if any.
    pub(crate) window: Option<(Instant, Instant)>,
    /// The kernel lead.
    pub(crate) lead: Duration,
}

impl Plan {
    /// PURE: the phase at `now`.
    pub(crate) fn phase(&self, now: Instant) -> Phase {
        let timer_hot = self.deadline.checked_sub(self.lead).unwrap_or(now);
        let hot_or_kernel = |until: Instant, before_window: bool| {
            if until.saturating_duration_since(now) >= Duration::from_nanos(MIN_KERNEL_SLEEP_NS) {
                Phase::Kernel {
                    until,
                    before_window,
                }
            } else {
                Phase::Hot
            }
        };
        // A window that goes hot before the timer and has not closed: nap
        // until it goes hot, or park once it has (nothing is left to sleep).
        if let Some((open, close)) = self.window {
            let window_hot = open.checked_sub(self.lead).unwrap_or(now);
            if window_hot < timer_hot && now < close {
                return hot_or_kernel(window_hot, true);
            }
        }
        // Nap until one lead before the deadline, or park once that is past.
        hot_or_kernel(timer_hot, false)
    }
}

/// Record-only counters for the exit line and the tests, lifetime-cumulative.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct PeriodParkStats {
    /// Park blocks the period-park planned.
    pub blocks: u64,
    /// Blocks that had a predicted data window.
    pub predicted_blocks: u64,
    /// Kernel naps that ran.
    pub kernel_sleeps: u64,
    /// Kernel naps a ring ended before their end.
    pub kernel_sleeps_cut_by_data: u64,
    /// Blocks that returned at their deadline.
    pub timer_wakes: u64,
    /// Of those, the ones more than 5 us past it.
    pub timer_late: u64,
    /// Blocks that returned on a data wake.
    pub data_wakes: u64,
    /// Data wakes the monitor-wait park saw.
    pub data_caught_hot: u64,
    /// Data wakes seen after a kernel nap before a predicted window.
    pub data_before_window: u64,
}

impl PeriodParkStats {
    /// The counts since `earlier` (a snapshot of the same lifetime-cumulative
    /// counters), so a resumed live loop reports only its own session.
    pub(crate) fn since(self, earlier: Self) -> Self {
        Self {
            blocks: self.blocks - earlier.blocks,
            predicted_blocks: self.predicted_blocks - earlier.predicted_blocks,
            kernel_sleeps: self.kernel_sleeps - earlier.kernel_sleeps,
            kernel_sleeps_cut_by_data: self.kernel_sleeps_cut_by_data
                - earlier.kernel_sleeps_cut_by_data,
            timer_wakes: self.timer_wakes - earlier.timer_wakes,
            timer_late: self.timer_late - earlier.timer_late,
            data_wakes: self.data_wakes - earlier.data_wakes,
            data_caught_hot: self.data_caught_hot - earlier.data_caught_hot,
            data_before_window: self.data_before_window - earlier.data_before_window,
        }
    }
}

/// The live loop's period-park state. Only the live-loop thread touches it.
#[derive(Debug, Default)]
pub(crate) struct State {
    /// This runtime has warned that a requested period-park did not engage.
    pub(crate) warned: bool,
    epoch: Option<Instant>,
    pub(crate) lead: KernelLead,
    pub(crate) arrivals: ArrivalLearner,
    pub(crate) stats: PeriodParkStats,
}

impl State {
    fn ns(&mut self, at: Instant) -> u64 {
        let epoch = *self.epoch.get_or_insert(at);
        at.saturating_duration_since(epoch).as_nanos() as u64
    }

    fn instant(&self, ns: u64) -> Option<Instant> {
        self.epoch?.checked_add(Duration::from_nanos(ns))
    }

    /// The plan for a block entered at `now` that must return by `deadline`.
    pub(crate) fn plan(&mut self, now: Instant, deadline: Instant) -> Plan {
        let now_ns = self.ns(now);
        let window = self.arrivals.predict(now_ns).and_then(|(open, close)| {
            let open = self.instant(open)?;
            let close = self.instant(close)?;
            (open < deadline).then_some((open, close.min(deadline)))
        });
        self.stats.blocks += 1;
        if window.is_some() {
            self.stats.predicted_blocks += 1;
        }
        Plan {
            deadline,
            window,
            lead: Duration::from_nanos(self.lead.lead_ns()),
        }
    }

    /// A kernel nap asked to end at `target` returned at `woke`; `rang` says
    /// whether the armed line rang since the park began. A nap that returned
    /// early without a ring (a signal or a spurious wake ends the kernel wait
    /// too) teaches the lead nothing and is not a data cut.
    pub(crate) fn note_kernel_sleep(&mut self, target: Instant, woke: Instant, rang: bool) {
        self.stats.kernel_sleeps += 1;
        if woke >= target {
            self.lead
                .observe(woke.duration_since(target).as_nanos() as u64);
        } else if rang {
            self.stats.kernel_sleeps_cut_by_data += 1;
        }
    }

    /// The block returned at its deadline.
    pub(crate) fn note_timer_wake(&mut self, deadline: Instant, woke: Instant) {
        self.stats.timer_wakes += 1;
        if woke.saturating_duration_since(deadline) > Duration::from_micros(5) {
            self.stats.timer_late += 1;
        }
    }

    /// The block returned on a data wake at `woke`, after a wait spent as
    /// `last`. A burst member (a frame right behind the one that started the
    /// burst) only counts.
    pub(crate) fn note_data_wake(&mut self, woke: Instant, last: LastWait) {
        let ns = self.ns(woke);
        self.stats.data_wakes += 1;
        if !self.arrivals.observe(ns) {
            return;
        }
        match last {
            LastWait::Hot => {
                self.stats.data_caught_hot += 1;
                self.arrivals.caught();
            }
            LastWait::KernelBeforeWindow | LastWait::None => {
                self.stats.data_before_window += 1;
                self.arrivals.arrived_before_window();
            }
            LastWait::KernelOther => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_switch_reads_one_as_on_and_only_zero_or_nothing_as_off() {
        assert_eq!(classify(None), Mode::Off);
        assert_eq!(classify(Some("")), Mode::Off);
        assert_eq!(classify(Some("0")), Mode::Off);
        assert_eq!(classify(Some("1")), Mode::On);
        for raw in ["yes", "true", "on", " 1", "1 ", "2", "grid"] {
            assert_eq!(classify(Some(raw)), Mode::Unrecognized, "{raw:?}");
        }
        assert!(Mode::On.on());
        assert!(!Mode::Off.on());
        assert!(!Mode::Unrecognized.on());
    }

    fn open_gate() -> Gate {
        Gate {
            on: true,
            park_active: true,
            primitive: true,
            barrier: false,
            credit_edges: false,
            fd_sources: false,
            outside_inputs: true,
            armed_line: true,
            data_wake: true,
        }
    }

    /// Each refusing term, alone, against a hand-written reason.
    fn one_refusal_per_term() -> [(Gate, NotEngaged, &'static str); 6] {
        [
            (
                Gate {
                    primitive: false,
                    ..open_gate()
                },
                NotEngaged::NoMonitorWaitPrimitive,
                "no_monitor_wait_primitive",
            ),
            (
                Gate {
                    park_active: false,
                    ..open_gate()
                },
                NotEngaged::ParkOff,
                "park_off",
            ),
            (
                Gate {
                    barrier: true,
                    ..open_gate()
                },
                NotEngaged::Barrier,
                "barrier",
            ),
            (
                Gate {
                    credit_edges: true,
                    ..open_gate()
                },
                NotEngaged::CreditEdge,
                "credit_edge",
            ),
            (
                Gate {
                    fd_sources: true,
                    ..open_gate()
                },
                NotEngaged::FdSource,
                "fd_source",
            ),
            (
                Gate {
                    data_wake: false,
                    ..open_gate()
                },
                NotEngaged::NoDataWake,
                "no_data_wake",
            ),
        ]
    }

    #[test]
    fn each_refusing_term_names_its_own_reason() {
        assert_eq!(refusal(open_gate()), None);
        assert_eq!(
            refusal(Gate {
                outside_inputs: false,
                armed_line: false,
                data_wake: false,
                ..open_gate()
            }),
            None,
            "no input written from outside needs no wake word"
        );
        // An input written from outside with the doorbell off (or its registry
        // failed to open): no line armed, so nothing can end a nap on it.
        assert_eq!(
            refusal(Gate {
                armed_line: false,
                data_wake: false,
                ..open_gate()
            }),
            Some(NotEngaged::NoDataWake)
        );
        assert_eq!(
            refusal(Gate {
                armed_line: false,
                ..open_gate()
            }),
            Some(NotEngaged::NoDataWake)
        );
        for (g, reason, text) in one_refusal_per_term() {
            assert_eq!(refusal(g), Some(reason), "{text}");
            assert_eq!(reason.as_str(), text);
            assert!(!admits(g), "{text}");
        }
    }

    #[test]
    fn the_first_refusing_term_in_order_is_the_reason() {
        let all_fail = Gate {
            on: true,
            park_active: false,
            primitive: false,
            barrier: true,
            credit_edges: true,
            fd_sources: true,
            outside_inputs: true,
            armed_line: true,
            data_wake: false,
        };
        assert_eq!(refusal(all_fail), Some(NotEngaged::NoMonitorWaitPrimitive));
        let host_ok = Gate {
            primitive: true,
            ..all_fail
        };
        assert_eq!(refusal(host_ok), Some(NotEngaged::ParkOff));
        assert_eq!(
            refusal(Gate {
                park_active: true,
                ..host_ok
            }),
            Some(NotEngaged::Barrier)
        );
        assert_eq!(
            refusal(Gate {
                barrier: false,
                credit_edges: false,
                ..open_gate()
            }),
            None
        );
        assert_eq!(
            refusal(Gate {
                credit_edges: true,
                fd_sources: true,
                data_wake: false,
                ..open_gate()
            }),
            Some(NotEngaged::CreditEdge)
        );
        assert_eq!(
            refusal(Gate {
                fd_sources: true,
                data_wake: false,
                ..open_gate()
            }),
            Some(NotEngaged::FdSource)
        );
    }

    /// The lines one `announce` call wrote, by the two messages it can write.
    fn announced(lines: &[&str], graph: &str) -> (Vec<String>, usize) {
        let mine = |l: &&&str| l.contains(&format!("graph={graph}"));
        let warns = lines
            .iter()
            .filter(mine)
            .filter(|l| l.contains("period park requested but not engaged"))
            .map(|l| l.to_string())
            .collect();
        let ons = lines
            .iter()
            .filter(mine)
            .filter(|l| l.contains("live loop period park on"))
            .count();
        (warns, ons)
    }

    #[tracing_test::traced_test]
    #[test]
    fn a_refused_period_park_warns_once_with_its_reason_and_never_says_on() {
        for (i, (g, _, text)) in one_refusal_per_term().into_iter().enumerate() {
            let graph = format!("refused{i}");
            let one_warning = |entry: &str| {
                logs_assert(|lines: &[&str]| {
                    let (warns, ons) = announced(lines, &graph);
                    match warns.as_slice() {
                        [line]
                            if line.contains("WARN")
                                && line.contains(&format!("reason=\"{text}\""))
                                && line.contains("target_arch")
                                && ons == 0 =>
                        {
                            Ok(())
                        }
                        _ => Err(format!(
                            "{text} after the {entry} entry: {} warn lines {warns:?}, {ons} on lines",
                            warns.len()
                        )),
                    }
                })
            };
            let mut warned = false;
            assert!(!announce(g, &mut warned, &graph), "{text}");
            one_warning("first");
            assert!(!announce(g, &mut warned, &graph), "{text}: a second entry");
            one_warning("second");
        }
    }

    #[tracing_test::traced_test]
    #[test]
    fn an_engaged_period_park_says_on_and_never_warns() {
        let mut warned = false;
        assert!(announce(open_gate(), &mut warned, "engaged"));
        assert!(!warned);
        let mut off = false;
        assert!(!announce(
            Gate {
                on: false,
                barrier: true,
                ..open_gate()
            },
            &mut off,
            "switchoff"
        ));
        logs_assert(|lines: &[&str]| {
            let (warns, ons) = announced(lines, "engaged");
            let (off_warns, off_ons) = announced(lines, "switchoff");
            if warns.is_empty() && ons == 1 && off_warns.is_empty() && off_ons == 0 {
                Ok(())
            } else {
                Err(format!(
                    "engaged: {warns:?} {ons}; switch off: {off_warns:?} {off_ons}"
                ))
            }
        });
    }

    #[test]
    fn the_period_park_applies_only_where_every_wake_can_end_a_nap() {
        assert!(admits(open_gate()));
        assert!(
            admits(Gate {
                outside_inputs: false,
                armed_line: false,
                data_wake: false,
                ..open_gate()
            }),
            "no input written from outside: nothing to wake a nap"
        );
        assert!(
            !admits(Gate {
                armed_line: false,
                data_wake: false,
                ..open_gate()
            }),
            "an input written from outside with the doorbell off: no wake word"
        );
        let closed = [
            (
                "switch off",
                Gate {
                    on: false,
                    ..open_gate()
                },
            ),
            (
                "park off",
                Gate {
                    park_active: false,
                    ..open_gate()
                },
            ),
            (
                "no CPU primitive",
                Gate {
                    primitive: false,
                    ..open_gate()
                },
            ),
            (
                "barrier participant",
                Gate {
                    barrier: true,
                    ..open_gate()
                },
            ),
            (
                "credit word",
                Gate {
                    credit_edges: true,
                    ..open_gate()
                },
            ),
            (
                "external fd",
                Gate {
                    fd_sources: true,
                    ..open_gate()
                },
            ),
            (
                "armed line without a wake word",
                Gate {
                    data_wake: false,
                    ..open_gate()
                },
            ),
        ];
        for (why, g) in closed {
            assert!(!admits(g), "{why}");
        }
    }

    #[test]
    fn the_lead_starts_at_its_seed_and_follows_measured_overshoot() {
        let mut l = KernelLead::default();
        assert_eq!(l.lead_ns(), 60_000 + 4 * 20_000 + 5_000);
        for _ in 0..200 {
            l.observe(50_000);
        }
        let lead = l.lead_ns();
        assert!((55_000..=55_100).contains(&lead), "lead {lead}");
        for _ in 0..200 {
            l.observe(0);
        }
        assert_eq!(l.lead_ns(), LEAD_MIN_NS);
        for _ in 0..50 {
            l.observe(u64::MAX);
        }
        assert_eq!(l.lead_ns(), LEAD_MAX_NS);
    }

    #[test]
    fn an_overshoot_past_twice_the_largest_lead_is_clipped_before_it_is_folded_in() {
        let mut l = KernelLead::default();
        l.observe(5_000_000);
        // Clipped to 2 x 400 us = 800 us. Mean: 60 + (800 - 60) / 8 = 152.5 us.
        // Deviation: 20 + (|800 - 60| - 20) / 4 = 200 us.
        assert_eq!((l.mean_ns, l.dev_ns), (152_500, 200_000));
    }

    #[test]
    fn one_late_overshoot_raises_the_lead_by_its_deviation() {
        let mut l = KernelLead::default();
        for _ in 0..200 {
            l.observe(50_000);
        }
        let steady = l.lead_ns();
        l.observe(150_000);
        assert!(
            l.lead_ns() > steady + 4 * 20_000,
            "one overshoot 100 us above the mean widens the lead: {} vs {steady}",
            l.lead_ns()
        );
    }

    fn regular_stream(learner: &mut ArrivalLearner, count: u64, period: u64) {
        for k in 0..count {
            learner.observe(k * period);
        }
    }

    #[test]
    fn a_regular_one_ms_stream_is_predicted_one_period_after_the_last_arrival() {
        let mut a = ArrivalLearner::default();
        regular_stream(&mut a, REGULAR_AFTER as u64, 1_000_000);
        assert!(
            a.predict(0).is_none(),
            "{REGULAR_AFTER} arrivals are one gap short"
        );
        a.observe(REGULAR_AFTER as u64 * 1_000_000);
        let last = REGULAR_AFTER as u64 * 1_000_000;
        let (open, close) = a.predict(last + 10).expect("regular");
        assert_eq!(open, last + 1_000_000 - MIN_WINDOW_NS);
        assert_eq!(close, last + 1_000_000 + MIN_WINDOW_NS);
    }

    #[test]
    fn a_gap_of_exactly_the_shortest_period_is_a_period_not_a_burst() {
        let mut a = ArrivalLearner::default();
        assert!(a.observe(0));
        assert!(
            a.observe(MIN_GAP_NS),
            "a gap of exactly MIN_GAP_NS is a period"
        );
        assert_eq!((a.period_ns, a.run), (MIN_GAP_NS, 1));
        let mut b = ArrivalLearner::default();
        b.observe(0);
        assert!(
            !b.observe(MIN_GAP_NS - 1),
            "one ns shorter is a burst member"
        );
        assert_eq!((b.period_ns, b.run), (0, 0));
    }

    #[test]
    fn a_gap_of_exactly_the_longest_period_is_learned_and_one_ns_more_forgets() {
        let mut a = ArrivalLearner::default();
        a.observe(0);
        assert!(a.observe(MAX_GAP_NS));
        assert_eq!(
            (a.period_ns, a.run),
            (MAX_GAP_NS, 1),
            "exactly the longest gap is a period"
        );
        let mut b = ArrivalLearner::default();
        b.observe(0);
        assert!(b.observe(MAX_GAP_NS + 1));
        assert_eq!(
            (b.period_ns, b.run, b.last_ns),
            (0, 0, Some(MAX_GAP_NS + 1)),
            "one ns more is an idle spell"
        );
    }

    #[test]
    fn a_regular_stream_folds_a_gap_up_to_its_tolerance_and_relearns_past_it() {
        // Twelve arrivals 1 ms apart: eleven gaps, deviation 0, so the
        // tolerance is 3 x max(0, 1 ms / 16) = 187.5 us.
        let steady = || {
            let mut a = ArrivalLearner::default();
            regular_stream(&mut a, 12, 1_000_000);
            a
        };
        let mut inside = steady();
        assert_eq!(
            (inside.period_ns, inside.dev_ns, inside.run),
            (1_000_000, 0, 11)
        );
        assert!(inside.observe(11_000_000 + 1_187_500));
        // Folded: period 1 ms + 187.5 us / 8 and deviation 187.5 us / 8,
        // both rounded down.
        assert_eq!(
            (inside.period_ns, inside.dev_ns, inside.run),
            (1_023_437, 23_437, 12)
        );
        let mut outside = steady();
        assert!(outside.observe(11_000_000 + 1_187_501));
        assert_eq!(
            (outside.period_ns, outside.dev_ns, outside.run),
            (1_187_501, 0, 1),
            "one ns past the tolerance starts the period over"
        );
    }

    #[test]
    fn an_idle_spell_or_a_step_back_in_time_forgets_a_regular_stream() {
        let mut a = ArrivalLearner::default();
        regular_stream(&mut a, 12, 1_000_000);
        assert!(
            a.regular(),
            "apparatus: twelve arrivals 1 ms apart are regular"
        );
        let back = 11_000_000 + MAX_GAP_NS + 1;
        assert!(a.observe(back));
        assert!(
            !a.regular(),
            "an idle spell longer than the largest gap starts over"
        );
        assert_eq!(a.predict(back + 10), None);

        let mut b = ArrivalLearner::default();
        regular_stream(&mut b, 12, 1_000_000);
        assert!(b.observe(5_000_000), "an arrival before the last one");
        assert!(!b.regular(), "a step back in time starts over");
        assert_eq!(b.predict(5_000_010), None);
    }

    #[test]
    fn a_burst_member_neither_moves_the_phase_nor_resets_the_period() {
        let mut a = ArrivalLearner::default();
        for k in 0..12u64 {
            assert!(a.observe(k * 1_000_000));
            assert!(!a.observe(k * 1_000_000 + 15_000), "a burst member");
        }
        let (open, _) = a.predict(11_000_100).expect("regular");
        assert_eq!(open, 12_000_000 - MIN_WINDOW_NS);
    }

    #[test]
    fn missed_beats_step_the_window_forward_and_a_changed_rate_relearns() {
        let mut a = ArrivalLearner::default();
        regular_stream(&mut a, 12, 1_000_000);
        let (open, close) = a.predict(14_500_000).expect("regular");
        assert_eq!((open, close), (15_000_000 - 20_000, 15_000_000 + 20_000));
        a.observe(16_000_000);
        assert!(
            a.predict(16_000_100).is_none(),
            "a 5 ms gap is out of tolerance"
        );
    }

    #[test]
    fn a_jittery_stream_is_not_predicted() {
        let mut a = ArrivalLearner::default();
        let mut t = 0;
        for k in 0..40u64 {
            t += if k % 2 == 0 { 600_000 } else { 1_400_000 };
            a.observe(t);
        }
        assert!(!a.regular(), "a deviation over an eighth of the period");
        assert!(a.predict(t).is_none());
    }

    #[test]
    fn early_widening_doubles_up_to_its_room_and_a_catch_drops_it() {
        let mut a = ArrivalLearner::default();
        regular_stream(&mut a, 12, 1_000_000);
        a.arrived_before_window();
        assert_eq!(
            a.predict(11_000_100).expect("regular").0,
            12_000_000 - 40_000
        );
        a.arrived_before_window();
        assert_eq!(
            a.predict(11_000_100).expect("regular").0,
            12_000_000 - 60_000
        );
        for _ in 0..10 {
            a.arrived_before_window();
        }
        let (open, close) = a.predict(11_000_100).expect("regular");
        assert_eq!(
            close - open,
            MAX_PARK_NS,
            "the window never exceeds its cap"
        );
        a.caught();
        assert_eq!(
            a.predict(11_000_100).expect("regular").0,
            12_000_000 - 20_000
        );
    }

    #[test]
    fn early_widening_needs_a_regular_stream() {
        let mut a = ArrivalLearner::default();
        regular_stream(&mut a, 3, 1_000_000);
        a.arrived_before_window();
        assert_eq!(a.early_ns, 0);
    }

    #[test]
    fn a_timer_plan_sleeps_until_one_lead_before_the_deadline_then_parks() {
        let t0 = Instant::now();
        let plan = Plan {
            deadline: t0 + Duration::from_micros(1_000),
            window: None,
            lead: Duration::from_micros(100),
        };
        assert_eq!(
            plan.phase(t0),
            Phase::Kernel {
                until: t0 + Duration::from_micros(900),
                before_window: false
            }
        );
        assert_eq!(plan.phase(t0 + Duration::from_micros(885)), Phase::Hot);
        assert_eq!(plan.phase(t0 + Duration::from_micros(950)), Phase::Hot);
        assert_eq!(plan.phase(t0 + Duration::from_micros(2_000)), Phase::Hot);
    }

    #[test]
    fn a_data_window_before_the_deadline_gets_its_own_lead_and_park() {
        let t0 = Instant::now();
        let plan = Plan {
            deadline: t0 + Duration::from_millis(250),
            window: Some((
                t0 + Duration::from_micros(980),
                t0 + Duration::from_micros(1_020),
            )),
            lead: Duration::from_micros(80),
        };
        assert_eq!(
            plan.phase(t0),
            Phase::Kernel {
                until: t0 + Duration::from_micros(900),
                before_window: true
            }
        );
        assert_eq!(plan.phase(t0 + Duration::from_micros(905)), Phase::Hot);
        assert_eq!(plan.phase(t0 + Duration::from_micros(1_010)), Phase::Hot);
        assert_eq!(
            plan.phase(t0 + Duration::from_micros(1_020)),
            Phase::Kernel {
                until: t0 + Duration::from_micros(250_000 - 80),
                before_window: false
            },
            "the window has closed at its close: back to the timer plan"
        );
        assert_eq!(
            plan.phase(t0 + Duration::from_micros(1_030)),
            Phase::Kernel {
                until: t0 + Duration::from_micros(250_000 - 80),
                before_window: false
            }
        );
    }

    #[test]
    fn a_window_that_opens_at_the_deadline_leaves_the_timer_plan() {
        let t0 = Instant::now();
        let plan = Plan {
            deadline: t0 + Duration::from_micros(1_000),
            window: Some((
                t0 + Duration::from_micros(1_000),
                t0 + Duration::from_micros(1_020),
            )),
            lead: Duration::from_micros(100),
        };
        assert_eq!(
            plan.phase(t0),
            Phase::Kernel {
                until: t0 + Duration::from_micros(900),
                before_window: false
            }
        );
    }

    #[test]
    fn the_state_plans_a_window_only_from_a_regular_stream_and_counts_its_blocks() {
        let t0 = Instant::now();
        let mut st = State::default();
        let deadline = t0 + Duration::from_millis(250);
        assert!(st.plan(t0, deadline).window.is_none());
        for k in 1..=10u64 {
            st.note_data_wake(t0 + Duration::from_millis(k), LastWait::KernelOther);
        }
        let plan = st.plan(t0 + Duration::from_micros(10_100), deadline);
        let (open, close) = plan.window.expect("a regular 1 ms stream predicts");
        assert_eq!(open, t0 + Duration::from_micros(11_000 - 20));
        assert_eq!(close, t0 + Duration::from_micros(11_000 + 20));
        assert_eq!(st.stats.blocks, 2);
        assert_eq!(st.stats.predicted_blocks, 1);
        assert_eq!(st.stats.data_wakes, 10);
    }

    #[test]
    fn a_nap_that_ran_to_its_end_teaches_the_lead_and_a_cut_one_does_not() {
        let t0 = Instant::now();
        let mut st = State::default();
        let seed = st.lead;
        st.note_kernel_sleep(t0, t0 - Duration::from_micros(30), true);
        assert_eq!(st.lead, seed, "a ring cut the nap: no overshoot sample");
        assert_eq!(st.stats.kernel_sleeps_cut_by_data, 1);
        st.note_kernel_sleep(t0, t0 + Duration::from_micros(50), true);
        assert_ne!(st.lead, seed);
        assert_eq!(st.stats.kernel_sleeps, 2);
        assert_eq!(
            st.stats.kernel_sleeps_cut_by_data, 1,
            "a nap that ran to its end is not cut, ring or not"
        );
    }

    #[test]
    fn a_nap_that_ended_early_without_a_ring_is_not_a_data_cut() {
        let t0 = Instant::now();
        let mut st = State::default();
        let seed = st.lead;
        st.note_kernel_sleep(t0, t0 - Duration::from_micros(30), false);
        assert_eq!(
            (st.stats.kernel_sleeps, st.stats.kernel_sleeps_cut_by_data),
            (1, 0)
        );
        assert_eq!(st.lead, seed, "an early return carries no overshoot sample");
    }

    #[test]
    fn a_late_timer_wake_is_counted_past_five_microseconds() {
        let t0 = Instant::now();
        let mut st = State::default();
        st.note_timer_wake(t0, t0 + Duration::from_micros(5));
        st.note_timer_wake(t0, t0 + Duration::from_micros(6));
        assert_eq!((st.stats.timer_wakes, st.stats.timer_late), (2, 1));
    }

    #[test]
    fn a_caught_wake_drops_the_early_widening_and_a_kernel_one_widens_it() {
        let t0 = Instant::now();
        let mut st = State::default();
        for k in 1..=12u64 {
            st.note_data_wake(t0 + Duration::from_millis(k), LastWait::KernelOther);
        }
        st.note_data_wake(t0 + Duration::from_millis(13), LastWait::KernelBeforeWindow);
        assert_eq!(st.stats.data_before_window, 1);
        assert_eq!(st.arrivals.early_ns, MIN_WINDOW_NS);
        st.note_data_wake(t0 + Duration::from_millis(14), LastWait::Hot);
        assert_eq!(st.stats.data_caught_hot, 1);
        assert_eq!(st.arrivals.early_ns, 0);
    }

    #[test]
    fn a_session_reports_its_counts_since_its_entry_snapshot() {
        let t0 = Instant::now();
        let mut st = State::default();
        st.plan(t0, t0 + Duration::from_millis(1));
        st.note_timer_wake(t0, t0 + Duration::from_micros(9));
        let entry = st.stats;
        st.plan(t0, t0 + Duration::from_millis(1));
        st.note_kernel_sleep(t0, t0 + Duration::from_micros(40), false);
        let session = st.stats.since(entry);
        assert_eq!(
            session,
            PeriodParkStats {
                blocks: 1,
                kernel_sleeps: 1,
                ..Default::default()
            }
        );
    }

    #[test]
    fn two_identical_arrival_sequences_give_identical_plans() {
        let run = || {
            let mut a = ArrivalLearner::default();
            let mut l = KernelLead::default();
            for k in 0..64u64 {
                a.observe(k * 1_000_000 + (k * 7_919) % 3_000);
                l.observe((k * 104_729) % 90_000);
            }
            (a.predict(64_000_000), l.lead_ns(), a)
        };
        assert_eq!(run(), run());
    }
}
