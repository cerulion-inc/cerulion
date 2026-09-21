// Pure format, payload and requeue rules. No camera, no kernel: these run on
// every platform, macOS included.
use super::frame::*;
use std::cell::Cell;

fn supported() -> V4l2PixFormat {
    V4l2PixFormat {
        width: 640,
        height: 480,
        pixel_format: u32::from_le_bytes(*b"YUYV"),
        bytes_per_line: 1280,
        ..Default::default()
    }
}

#[test]
fn exact_format_matches_and_every_incompatible_field_is_refused() {
    assert!(validate_format(&supported()).is_ok());
    let mut other = supported();
    other.width = 320;
    assert!(validate_format(&other).is_err());
    let mut other = supported();
    other.height = 240;
    assert!(validate_format(&other).is_err());
    let mut other = supported();
    other.pixel_format = u32::from_le_bytes(*b"UYVY");
    assert!(
        validate_format(&other).is_err(),
        "same byte length, wrong pixel order"
    );
    let mut other = supported();
    other.bytes_per_line = 1296;
    assert!(
        validate_format(&other).is_err(),
        "padded rows need different metadata"
    );
}

#[test]
fn g_fmt_request_matches_64_bit_linux_uapi() {
    assert_eq!(g_fmt_request(), 0xc0d0_5604);
}

#[test]
fn captured_frame_must_fit_mapping_and_declared_shape() {
    assert_eq!(capture_len(24, 32, 12, 2).unwrap(), 24);
    assert!(capture_len(23, 32, 12, 2).is_err(), "short frame");
    assert!(capture_len(25, 32, 12, 2).is_err(), "extra bytes");
    assert!(capture_len(24, 23, 12, 2).is_err(), "outside mmap");
}

#[test]
fn every_publish_result_returns_the_capture_buffer() {
    for publication in [Ok(()), Err("publish")] {
        let requeued = Cell::new(0);
        let result = publish_and_requeue(
            || publication,
            || {
                requeued.set(requeued.get() + 1);
                Ok(())
            },
        );
        assert_eq!(requeued.get(), 1);
        assert_eq!(result, publication);
    }
    assert_eq!(
        publish_and_requeue(|| Ok(()), || Err("requeue")),
        Err("requeue")
    );
    assert_eq!(
        publish_and_requeue(|| Err("publish"), || Err("requeue")),
        Err("requeue")
    );
}
