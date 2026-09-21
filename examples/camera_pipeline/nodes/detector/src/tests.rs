use super::*;
// Test-only isolated transport; node code uses the public prelude.
use cerulion_core::testing::TestTransport;

// Hand-encoded Header: zero timestamp and "camera" at offset 16.
const HEADER: [u8; 22] = [
    0, 0, 0, 0, 0, 0, 0, 0, 16, 0, 0, 0, 6, 0, 0, 0, b'c', b'a', b'm', b'e', b'r', b'a',
];

#[test]
fn threshold_centroid_uses_pixels_and_ignores_padding() {
    let pixels = [0, 0, 0, 128, 128, 128, 255, 255, 0, 0, 0, 0, 0, 0, 255, 255];
    assert_eq!(
        bright_center(&pixels, 2, 2, 8, "rgb8").unwrap(),
        Some((0.25, -0.25))
    );
    assert_eq!(bright_center(&[127; 12], 2, 2, 6, "rgb8").unwrap(), None);
    assert_eq!(
        bright_center(&[128; 12], 2, 2, 6, "rgb8").unwrap(),
        Some((0.0, 0.0))
    );
}

#[test]
fn malformed_image_metadata_is_rejected() {
    for (data, width, height, step, encoding) in [
        (&[][..], 0, 1, 3, "rgb8"),
        (&[][..], 1, 0, 3, "rgb8"),
        (&[0; 3][..], 2, 1, 3, "rgb8"),
        (&[0; 3][..], 1, 2, 3, "rgb8"),
        (&[0; 3][..], 1, 1, 3, "mono8"),
    ] {
        assert!(bright_center(data, width, height, step, encoding).is_err());
    }
}

#[test]
fn detector_publishes_a_pose_for_bright_pixels_and_nothing_otherwise() {
    let transport = TestTransport::with_buffer_size(4);
    let mut input = transport.publisher("image", MaxSliceLen::const_new(4096), 0);
    let input_sub = transport.subscriber("image");
    let output = transport.publisher("detection", MaxSliceLen::const_new(4096), 0);
    let mut output_sub = transport.subscriber("detection");
    let mut entry = DetectorNodeEntry::new();
    entry
        .init(NodeContext::for_tests(
            [("detection".into(), AnyPublisher::Ipc(output))]
                .into_iter()
                .collect(),
            [("image".into(), AnySubscriber::Ipc(input_sub))]
                .into_iter()
                .collect(),
        ))
        .unwrap();
    // (pixels, step, valid encoding, tick succeeds, published pose). A dark
    // frame ticks cleanly and publishes NOTHING: the output is never written.
    for (pixels, step, valid_encoding, tick_ok, expected) in [
        (
            [0, 0, 0, 128, 128, 128, 0, 0, 0, 0, 0, 0],
            6,
            true,
            true,
            Some([0.25, -0.25, 1.0, 0.0, 0.0, 0.0, 1.0]),
        ),
        ([0; 12], 6, true, true, None),
        ([0; 12], 5, true, false, None),
        ([0; 12], 6, false, false, None),
    ] {
        {
            let mut image = input.loan_proxy::<Image>().unwrap();
            image.width = 2;
            image.height = 2;
            image.step = step;
            image.set_header_bytes(&HEADER).unwrap();
            if valid_encoding {
                image.set_encoding("rgb8").unwrap();
            } else {
                image.loan_encoding(1).unwrap().fill(0xff);
            }
            image.set_data(&pixels).unwrap();
        }
        let outcome = entry.tick();
        assert_eq!(outcome.is_ok(), tick_ok);
        if !valid_encoding {
            assert!(outcome
                .unwrap_err()
                .to_string()
                .contains("Invalid input 'image'"));
        }
        let delivered = output_sub
            .try_view::<PoseStamped, _>(|stamped| {
                assert_eq!(stamped.header_bytes(), HEADER);
                let (p, q) = (stamped.pose.position, stamped.pose.orientation);
                [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
            })
            .unwrap();
        assert_eq!(delivered, expected);
    }
    entry.shutdown().unwrap();
}
