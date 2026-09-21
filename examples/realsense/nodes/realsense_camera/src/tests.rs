// Pure plane and slot rules. No camera, no SDK: these run everywhere, with
// or without the `realsense` feature.
use super::frame::*;

fn plane(width: u32, height: u32, stride: u32, bytes: &[u8]) -> Plane<'_> {
    Plane {
        width,
        height,
        stride,
        bytes,
    }
}

#[test]
fn a_plane_of_the_requested_shape_is_accepted_and_every_mismatch_refused() {
    let color = vec![0u8; (WIDTH * COLOR_BYTES_PER_PIXEL * HEIGHT) as usize];
    let depth = vec![0u8; (WIDTH * DEPTH_BYTES_PER_PIXEL * HEIGHT) as usize];
    assert!(check_plane(
        &plane(WIDTH, HEIGHT, WIDTH * 3, &color),
        COLOR_BYTES_PER_PIXEL
    )
    .is_ok());
    assert!(check_plane(
        &plane(WIDTH, HEIGHT, WIDTH * 2, &depth),
        DEPTH_BYTES_PER_PIXEL
    )
    .is_ok());
    // Wrong dimensions, a stride shorter than a row, and a byte count that
    // disagrees with stride * height are each refused on their own.
    assert!(check_plane(&plane(320, HEIGHT, 320 * 3, &color[..320 * 3 * 480]), 3).is_err());
    assert!(check_plane(&plane(WIDTH, 240, WIDTH * 3, &color[..640 * 3 * 240]), 3).is_err());
    assert!(check_plane(
        &plane(WIDTH, HEIGHT, WIDTH * 2, &color),
        COLOR_BYTES_PER_PIXEL
    )
    .is_err());
    assert!(check_plane(
        &plane(WIDTH, HEIGHT, WIDTH * 3, &color[1..]),
        COLOR_BYTES_PER_PIXEL
    )
    .is_err());
    // A padded stride is fine as long as the bytes agree with it.
    let padded = vec![0u8; (2000 * HEIGHT) as usize];
    assert!(check_plane(&plane(WIDTH, HEIGHT, 2000, &padded), COLOR_BYTES_PER_PIXEL).is_ok());
}

#[test]
fn the_slot_keeps_the_latest_value_and_counts_what_it_replaced() {
    let mut slot: Slot<u32> = Slot::default();
    assert_eq!(slot.take(), None);
    slot.offer(1);
    assert_eq!(slot.take(), Some(1));
    assert_eq!(slot.overwritten(), 0, "a taken value is not a replacement");
    slot.offer(2);
    slot.offer(3);
    slot.offer(4);
    assert_eq!(
        slot.overwritten(),
        2,
        "two values were replaced before a take"
    );
    assert_eq!(slot.take(), Some(4), "latest wins");
    assert_eq!(slot.take(), None, "a take empties the slot");
}

#[test]
fn the_depth_unit_this_example_publishes_is_one_millimetre() {
    assert_eq!(DEPTH_UNIT_M, 0.001);
    assert_eq!(DEPTH_ENCODING, "16UC1");
}
