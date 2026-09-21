use super::*;
// Isolated transport is test support, not part of the example's node API.
use cerulion_core::testing::TestTransport;

// Hand-encoded Header: zero timestamp and "camera" at offset 16.
const HEADER: [u8; 22] = [
    0, 0, 0, 0, 0, 0, 0, 0, 16, 0, 0, 0, 6, 0, 0, 0, b'c', b'a', b'm', b'e', b'r', b'a',
];

#[test]
fn mean_luma_reads_only_the_y_samples_and_skips_row_padding() {
    // Two rows of two pixels: [Y0 U Y1 V] per row, then two padding bytes.
    let rows = [
        10, 200, 30, 200, 255, 255, // row 0: Y = 10, 30
        50, 200, 70, 200, 255, 255, // row 1: Y = 50, 70
    ];
    assert_eq!(mean_luma(&rows, 2, 2, 6, ENCODING).unwrap(), 40.0);
    assert_eq!(mean_luma(&[0; 8], 2, 2, 4, ENCODING).unwrap(), 0.0);
    assert_eq!(mean_luma(&[255; 8], 2, 2, 4, ENCODING).unwrap(), 255.0);
}

#[test]
fn malformed_image_metadata_is_rejected() {
    for (data, width, height, step, encoding) in [
        (&[][..], 0, 1, 2, ENCODING),
        (&[][..], 1, 0, 2, ENCODING),
        (&[0; 4][..], 2, 1, 3, ENCODING),
        (&[0; 4][..], 1, 2, 4, ENCODING),
        (&[0; 4][..], 1, 2, 2, "rgb8"),
    ] {
        assert!(mean_luma(data, width, height, step, encoding).is_err());
    }
}

#[test]
fn meter_publishes_the_mean_luma_of_each_delivered_frame() {
    let transport = TestTransport::with_buffer_size(4);
    let mut input = transport.publisher("image", MaxSliceLen::const_new(4096), 0);
    let input_sub = transport.subscriber("image");
    let output = transport.publisher("brightness", MaxSliceLen::const_new(4096), 0);
    let mut output_sub = transport.subscriber("brightness");
    let mut entry = BrightnessMeterNodeEntry::new();
    entry
        .init(NodeContext::for_tests(
            [("brightness".into(), AnyPublisher::Ipc(output))]
                .into_iter()
                .collect(),
            [("image".into(), AnySubscriber::Ipc(input_sub))]
                .into_iter()
                .collect(),
        ))
        .unwrap();
    // (pixels, valid encoding, tick succeeds, published mean).
    for (pixels, valid_encoding, tick_ok, expected) in [
        ([10, 0, 30, 0, 50, 0, 70, 0], true, true, Some(40.0)),
        ([200; 8], true, true, Some(200.0)),
        ([200; 8], false, false, None),
    ] {
        {
            let mut image = input.loan_proxy::<Image>().unwrap();
            image.width = 2;
            image.height = 2;
            image.step = 4;
            image.set_header_bytes(&HEADER).unwrap();
            if valid_encoding {
                image.set_encoding(ENCODING).unwrap();
            } else {
                image.set_encoding("rgb8").unwrap();
            }
            image.set_data(&pixels).unwrap();
        }
        let outcome = entry.tick();
        assert_eq!(outcome.is_ok(), tick_ok);
        let delivered = output_sub
            .try_view::<Float32, _>(|brightness| brightness.data)
            .unwrap();
        assert_eq!(delivered, expected);
    }
    entry.shutdown().unwrap();
}
