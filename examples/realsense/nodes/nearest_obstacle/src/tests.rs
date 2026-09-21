use super::*;
// Isolated transport is test support, not part of the example's node API.
use cerulion_core::testing::TestTransport;

// Hand-encoded Header: zero timestamp and "depth" at offset 16.
const HEADER: [u8; 21] = [
    0, 0, 0, 0, 0, 0, 0, 0, 16, 0, 0, 0, 5, 0, 0, 0, b'd', b'e', b'p', b't', b'h',
];

/// A `w x h` 16UC1 image filled with `fill`, with `count` written at (x, y).
fn depth_image(w: usize, h: usize, fill: u16, at: Option<(usize, usize, u16)>) -> Vec<u8> {
    let mut counts = vec![fill; w * h];
    if let Some((x, y, count)) = at {
        counts[y * w + x] = count;
    }
    counts.iter().flat_map(|c| c.to_le_bytes()).collect()
}

#[test]
fn cone_span_is_the_central_tenth_and_never_empty() {
    assert_eq!(cone_span(640), (288, 352));
    assert_eq!(cone_span(480), (216, 264));
    assert_eq!(cone_span(1), (0, 1));
    assert_eq!(cone_span(4), (1, 2), "rounds to one pixel, centred");
}

#[test]
fn nearest_in_cone_ignores_zero_counts_and_pixels_outside_the_cone() {
    // 20x20: the cone is the central 2x2 block at x,y in 9..11.
    let inside = depth_image(20, 20, 5000, Some((9, 9, 1234)));
    assert_eq!(
        nearest_in_cone(&inside, 20, 20, 40, ENCODING).unwrap(),
        Some(1.234)
    );
    let outside = depth_image(20, 20, 5000, Some((0, 0, 1)));
    assert_eq!(
        nearest_in_cone(&outside, 20, 20, 40, ENCODING).unwrap(),
        Some(5.0)
    );
    let zeros_in_cone = depth_image(20, 20, 0, None);
    assert_eq!(
        nearest_in_cone(&zeros_in_cone, 20, 20, 40, ENCODING).unwrap(),
        None
    );
    let one_valid = depth_image(20, 20, 0, Some((10, 10, 300)));
    assert_eq!(
        nearest_in_cone(&one_valid, 20, 20, 40, ENCODING).unwrap(),
        Some(0.3)
    );
}

#[test]
fn malformed_depth_metadata_is_rejected() {
    for (data, width, height, step, encoding) in [
        (&[][..], 0, 1, 2, ENCODING),
        (&[][..], 1, 0, 2, ENCODING),
        (&[0; 4][..], 2, 1, 3, ENCODING),
        (&[0; 4][..], 1, 2, 4, ENCODING),
        (&[0; 4][..], 1, 2, 2, "rgb8"),
    ] {
        assert!(nearest_in_cone(data, width, height, step, encoding).is_err());
    }
}

#[test]
fn a_range_is_published_for_every_depth_frame() {
    let transport = TestTransport::with_buffer_size(4);
    let mut input = transport.publisher("depth", MaxSliceLen::const_new(4096), 0);
    let input_sub = transport.subscriber("depth");
    let output = transport.publisher("obstacle", MaxSliceLen::const_new(4096), 0);
    let mut output_sub = transport.subscriber("obstacle");
    let mut entry = NearestObstacleNodeEntry::new();
    entry
        .init(NodeContext::for_tests(
            [("obstacle".into(), AnyPublisher::Ipc(output))]
                .into_iter()
                .collect(),
            [("depth".into(), AnySubscriber::Ipc(input_sub))]
                .into_iter()
                .collect(),
        ))
        .unwrap();
    // (image, valid encoding, tick succeeds, published range).
    for (image, valid_encoding, tick_ok, expected) in [
        (
            depth_image(20, 20, 5000, Some((10, 10, 750))),
            true,
            true,
            Some(0.75),
        ),
        (
            depth_image(20, 20, 0, None),
            true,
            true,
            Some(f32::INFINITY),
        ),
        (depth_image(20, 20, 5000, None), false, false, None),
    ] {
        {
            let mut depth = input.loan_proxy::<Image>().unwrap();
            depth.width = 20;
            depth.height = 20;
            depth.step = 40;
            depth.set_header_bytes(&HEADER).unwrap();
            depth
                .set_encoding(if valid_encoding { ENCODING } else { "mono8" })
                .unwrap();
            depth.set_data(&image).unwrap();
        }
        let outcome = entry.tick();
        assert_eq!(outcome.is_ok(), tick_ok);
        let delivered = output_sub
            .try_view::<Range, _>(|range| {
                assert_eq!(range.header_bytes(), HEADER);
                assert_eq!(range.radiation_type, INFRARED);
                assert!((range.field_of_view - 8.7f32.to_radians()).abs() < 1e-6);
                assert_eq!(
                    (range.min_range, range.max_range),
                    (MIN_RANGE_M, MAX_RANGE_M)
                );
                range.range
            })
            .unwrap();
        assert_eq!(delivered, expected);
    }
    entry.shutdown().unwrap();
}
