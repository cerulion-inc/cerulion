// SPDX-License-Identifier: AGPL-3.0-only
//! The v1 limits the robot enforces before any network call.
//!
//! - Rate: a token bucket of [`BURST`] tokens refilled at
//!   [`LOOKS_PER_SECOND`]; an ask that finds it empty is answered `throttled`.
//! - In flight: at most [`MAX_IN_FLIGHT`] requests on the network, a request
//!   whose answer came late included; one more is answered `throttled`.
//! - Shape: one JPEG of at most [`MAX_IMAGE_BYTES`] bytes and
//!   [`MAX_IMAGE_WIDTH`] x [`MAX_IMAGE_HEIGHT`] pixels, at least
//!   [`MIN_IMAGE_SIDE`] pixels a side, and at most [`MAX_INPUT_TOKENS`]
//!   estimated input tokens; anything else is answered `refused`.
//! - Time: no answer within [`TIMEOUT_MS`] is answered `timeout`.
//!
//! The spend limits (per robot, per account, per fleet) are enforced by the
//! route, which answers with a status the client maps to `quota`; the eye
//! then sends nothing until 00:00 UTC ([`crate::eye::host::EyeCore::note`]).

use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

/// Sustained looks per second.
pub const LOOKS_PER_SECOND: u32 = 5;

/// Looks a full bucket allows at once.
pub const BURST: u32 = 10;

/// Requests on the network at once.
pub const MAX_IN_FLIGHT: usize = 2;

/// The largest image sent, bytes (200 KB).
pub const MAX_IMAGE_BYTES: usize = 200_000;

/// The widest image sent, pixels.
pub const MAX_IMAGE_WIDTH: u32 = 640;

/// The tallest image sent, pixels.
pub const MAX_IMAGE_HEIGHT: u32 = 480;

/// The shortest side TypeSafe accepts, pixels.
pub const MIN_IMAGE_SIDE: u32 = 280;

/// The most input tokens one request may carry.
pub const MAX_INPUT_TOKENS: u64 = 4_000;

/// The longest an ask waits for its answer, milliseconds.
pub const TIMEOUT_MS: u64 = 1_500;

/// How long a request that missed its deadline is kept on the network to
/// log a late answer before it is cancelled.
pub const LATE_GRACE: Duration = Duration::from_secs(5);

/// Pixels per estimated image token. Measured: a 640 x 480 frame with about
/// 1,400 bytes of text cost 1,425 input tokens; 200 overestimates the image.
const PIXELS_PER_TOKEN: u64 = 200;

/// Text bytes per estimated token; 3 overestimates English text.
const BYTES_PER_TOKEN: u64 = 3;

/// An overestimate of a request's input tokens: its text bytes and an image
/// of `width` x `height` pixels.
pub fn estimate_input_tokens(text_bytes: usize, width: u32, height: u32) -> u64 {
    let text = (text_bytes as u64).div_ceil(BYTES_PER_TOKEN);
    let image = (u64::from(width) * u64::from(height)).div_ceil(PIXELS_PER_TOKEN);
    text + image
}

/// The most fixed text bytes a question set may carry so that a request
/// with the largest allowed image stays within [`MAX_INPUT_TOKENS`], with
/// room kept for the request's structure and numbers.
pub fn max_fixed_text_bytes() -> usize {
    let image = estimate_input_tokens(0, MAX_IMAGE_WIDTH, MAX_IMAGE_HEIGHT);
    let tokens = MAX_INPUT_TOKENS.saturating_sub(image);
    usize::try_from(tokens * BYTES_PER_TOKEN).unwrap_or(usize::MAX) - 1_024
}

/// The rate limit: [`BURST`] tokens, refilled at [`LOOKS_PER_SECOND`].
#[derive(Debug, Clone)]
pub struct TokenBucket {
    tokens: f64,
    last: Instant,
}

impl TokenBucket {
    /// A full bucket at `now`.
    pub fn new(now: Instant) -> Self {
        Self {
            tokens: f64::from(BURST),
            last: now,
        }
    }

    /// Take one token at `now`; `false` when the bucket is empty.
    pub fn try_take(&mut self, now: Instant) -> bool {
        let elapsed = now.saturating_duration_since(self.last).as_secs_f64();
        self.last = self.last.max(now);
        self.tokens = (self.tokens + elapsed * f64::from(LOOKS_PER_SECOND)).min(f64::from(BURST));
        if self.tokens >= 1.0 {
            self.tokens -= 1.0;
            true
        } else {
            false
        }
    }
}

/// The in-flight count, shared with the tasks that hold requests.
#[derive(Debug, Clone, Default)]
pub struct InFlight(Arc<AtomicUsize>);

/// One request on the network; dropping it frees the slot.
#[derive(Debug)]
pub struct InFlightGuard(Arc<AtomicUsize>);

impl InFlight {
    /// A slot, or `None` when [`MAX_IN_FLIGHT`] requests are on the network.
    pub fn try_acquire(&self) -> Option<InFlightGuard> {
        let mut seen = self.0.load(Ordering::SeqCst);
        loop {
            if seen >= MAX_IN_FLIGHT {
                return None;
            }
            match self
                .0
                .compare_exchange_weak(seen, seen + 1, Ordering::SeqCst, Ordering::SeqCst)
            {
                Ok(_) => return Some(InFlightGuard(Arc::clone(&self.0))),
                Err(now) => seen = now,
            }
        }
    }

    /// Requests on the network now.
    pub fn count(&self) -> usize {
        self.0.load(Ordering::SeqCst)
    }
}

impl Drop for InFlightGuard {
    fn drop(&mut self) {
        self.0.fetch_sub(1, Ordering::SeqCst);
    }
}

/// Why a frame is not sent.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ShapeRefusal {
    /// The bytes are not a JPEG.
    NotJpeg,
    /// A JPEG whose size could not be read.
    Unreadable,
    /// More than [`MAX_IMAGE_BYTES`] bytes.
    TooManyBytes,
    /// Wider than [`MAX_IMAGE_WIDTH`] or taller than [`MAX_IMAGE_HEIGHT`].
    TooLarge,
    /// A side shorter than [`MIN_IMAGE_SIDE`].
    TooSmall,
    /// More than [`MAX_INPUT_TOKENS`] estimated input tokens.
    TooManyTokens,
}

impl ShapeRefusal {
    /// The `detail` an answer frame carries.
    pub fn code(self) -> &'static str {
        match self {
            Self::NotJpeg => "image_not_jpeg",
            Self::Unreadable => "image_unreadable",
            Self::TooManyBytes => "image_too_many_bytes",
            Self::TooLarge => "image_too_large",
            Self::TooSmall => "image_too_small",
            Self::TooManyTokens => "too_many_tokens",
        }
    }
}

/// A JPEG's size in pixels.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct JpegSize {
    /// Pixels across.
    pub width: u32,
    /// Pixels down.
    pub height: u32,
}

/// The size a JPEG's first start-of-frame marker declares.
///
/// # Errors
///
/// [`ShapeRefusal::NotJpeg`] without the start-of-image marker,
/// [`ShapeRefusal::Unreadable`] when no start-of-frame marker precedes the
/// scan data.
pub fn jpeg_size(bytes: &[u8]) -> Result<JpegSize, ShapeRefusal> {
    if bytes.len() < 4 || bytes[0] != 0xFF || bytes[1] != 0xD8 {
        return Err(ShapeRefusal::NotJpeg);
    }
    let mut i = 2;
    loop {
        // Fill bytes before a marker are 0xFF.
        while bytes.get(i) == Some(&0xFF) && bytes.get(i + 1) == Some(&0xFF) {
            i += 1;
        }
        if bytes.get(i) != Some(&0xFF) {
            return Err(ShapeRefusal::Unreadable);
        }
        let marker = *bytes.get(i + 1).ok_or(ShapeRefusal::Unreadable)?;
        i += 2;
        match marker {
            // Standalone markers carry no length.
            0x01 | 0xD0..=0xD7 => continue,
            // End of image or start of scan before any frame header.
            0xD9 | 0xDA => return Err(ShapeRefusal::Unreadable),
            _ => {}
        }
        let len = usize::from(u16::from_be_bytes([
            *bytes.get(i).ok_or(ShapeRefusal::Unreadable)?,
            *bytes.get(i + 1).ok_or(ShapeRefusal::Unreadable)?,
        ]));
        if len < 2 {
            return Err(ShapeRefusal::Unreadable);
        }
        // Start-of-frame markers, less DHT (C4), JPG (C8) and DAC (CC).
        if matches!(marker, 0xC0..=0xCF) && !matches!(marker, 0xC4 | 0xC8 | 0xCC) {
            let segment = bytes.get(i + 2..i + len).ok_or(ShapeRefusal::Unreadable)?;
            if segment.len() < 5 {
                return Err(ShapeRefusal::Unreadable);
            }
            let height = u32::from(u16::from_be_bytes([segment[1], segment[2]]));
            let width = u32::from(u16::from_be_bytes([segment[3], segment[4]]));
            if width == 0 || height == 0 {
                return Err(ShapeRefusal::Unreadable);
            }
            return Ok(JpegSize { width, height });
        }
        i += len;
    }
}

/// Check one frame against the image shape and return its size.
///
/// # Errors
///
/// The first [`ShapeRefusal`] that applies.
pub fn check_image(bytes: &[u8]) -> Result<JpegSize, ShapeRefusal> {
    if bytes.len() > MAX_IMAGE_BYTES {
        return Err(ShapeRefusal::TooManyBytes);
    }
    let size = jpeg_size(bytes)?;
    if size.width > MAX_IMAGE_WIDTH || size.height > MAX_IMAGE_HEIGHT {
        return Err(ShapeRefusal::TooLarge);
    }
    if size.width < MIN_IMAGE_SIDE || size.height < MIN_IMAGE_SIDE {
        return Err(ShapeRefusal::TooSmall);
    }
    Ok(size)
}

/// A minimal JPEG header of `width` x `height`: start of image, one baseline
/// start-of-frame segment and end of image. Not a decodable image; enough
/// for [`jpeg_size`] and for tests that only need the shape.
pub fn jpeg_header_for_test(width: u16, height: u16, padding: usize) -> Vec<u8> {
    let mut out = vec![0xFF, 0xD8];
    // An APP0 segment of `padding` bytes to set the file size.
    if padding > 0 {
        let len = u16::try_from(padding.min(60_000) + 2).unwrap_or(u16::MAX);
        out.extend_from_slice(&[0xFF, 0xE0]);
        out.extend_from_slice(&len.to_be_bytes());
        out.extend(std::iter::repeat_n(0x20, usize::from(len) - 2));
        let mut rest = padding.saturating_sub(60_000);
        while rest > 0 {
            let n = rest.min(60_000);
            let len = u16::try_from(n + 2).unwrap_or(u16::MAX);
            out.extend_from_slice(&[0xFF, 0xE1]);
            out.extend_from_slice(&len.to_be_bytes());
            out.extend(std::iter::repeat_n(0x20, n));
            rest -= n;
        }
    }
    out.extend_from_slice(&[0xFF, 0xC0, 0x00, 0x11, 0x08]);
    out.extend_from_slice(&height.to_be_bytes());
    out.extend_from_slice(&width.to_be_bytes());
    out.extend_from_slice(&[0x03, 0x01, 0x22, 0x00, 0x02, 0x11, 0x01, 0x03, 0x11, 0x01]);
    out.extend_from_slice(&[0xFF, 0xD9]);
    out
}

#[cfg(test)]
mod tests {
    use std::time::{Duration, Instant};

    use super::{
        check_image, estimate_input_tokens, jpeg_header_for_test, jpeg_size, max_fixed_text_bytes,
        InFlight, JpegSize, ShapeRefusal, TokenBucket, BURST, MAX_INPUT_TOKENS, MAX_IN_FLIGHT,
    };

    #[test]
    fn the_bucket_allows_a_burst_of_ten_then_five_a_second() {
        let start = Instant::now();
        let mut bucket = TokenBucket::new(start);
        let burst = (0..20).filter(|_| bucket.try_take(start)).count();
        assert_eq!(burst, BURST as usize);
        // One second later five more, not ten.
        let later = start + Duration::from_secs(1);
        let refill = (0..20).filter(|_| bucket.try_take(later)).count();
        assert_eq!(refill, 5);
        // A long idle never fills past the burst.
        let idle = later + Duration::from_secs(60);
        assert_eq!((0..20).filter(|_| bucket.try_take(idle)).count(), 10);
        // A clock that steps back takes nothing extra.
        assert!(!bucket.try_take(start));
    }

    #[test]
    fn two_requests_fly_at_once_and_a_dropped_guard_frees_its_slot() {
        let in_flight = InFlight::default();
        let first = in_flight.try_acquire().expect("first");
        let _second = in_flight.try_acquire().expect("second");
        assert!(in_flight.try_acquire().is_none());
        assert_eq!(in_flight.count(), MAX_IN_FLIGHT);
        drop(first);
        assert!(in_flight.try_acquire().is_some());
    }

    #[test]
    fn the_frame_shape_is_one_jpeg_within_640_by_480_and_200_kb() {
        let ok = jpeg_header_for_test(640, 480, 1_000);
        assert_eq!(
            jpeg_size(&ok),
            Ok(JpegSize {
                width: 640,
                height: 480
            })
        );
        assert!(check_image(&ok).is_ok());
        assert_eq!(
            check_image(&jpeg_header_for_test(641, 480, 0)),
            Err(ShapeRefusal::TooLarge)
        );
        assert_eq!(
            check_image(&jpeg_header_for_test(640, 481, 0)),
            Err(ShapeRefusal::TooLarge)
        );
        assert_eq!(
            check_image(&jpeg_header_for_test(279, 480, 0)),
            Err(ShapeRefusal::TooSmall)
        );
        let big = jpeg_header_for_test(640, 480, 200_001);
        assert_eq!(check_image(&big), Err(ShapeRefusal::TooManyBytes));
        let edge = jpeg_header_for_test(640, 480, 200_000 - 40);
        assert!(edge.len() <= 200_000, "{}", edge.len());
        assert!(check_image(&edge).is_ok());
        assert_eq!(
            check_image(b"\x89PNG\r\n\x1a\n...."),
            Err(ShapeRefusal::NotJpeg)
        );
        assert_eq!(
            check_image(&[0xFF, 0xD8, 0xFF, 0xDA, 0, 2]),
            Err(ShapeRefusal::Unreadable)
        );
        assert_eq!(
            check_image(&[0xFF, 0xD8, 0xFF, 0xE0, 0xFF]),
            Err(ShapeRefusal::Unreadable)
        );
    }

    #[test]
    fn the_largest_frame_and_the_text_budget_fit_the_token_limit() {
        let budget = max_fixed_text_bytes();
        assert!(estimate_input_tokens(budget + 1_024, 640, 480) <= MAX_INPUT_TOKENS);
        assert!(estimate_input_tokens(budget + 1_024 + 4, 640, 480) > MAX_INPUT_TOKENS);
    }
}
