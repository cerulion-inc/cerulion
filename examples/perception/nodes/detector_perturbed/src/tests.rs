use super::*;
// Test-only isolated transport; node code uses the public prelude.
use cerulion_core::testing::TestTransport;

const HEADER: [u8; 16] = [0, 0, 0, 0, 0, 0, 0, 0, 16, 0, 0, 0, 0, 0, 0, 0];

#[test]
fn detector_delivers_pixel_bounds_with_expected_regression_and_empty_arrays() {
    let transport = TestTransport::with_buffer_size(4);
    let mut input = transport.publisher("image", MaxSliceLen::const_new(32768), 0);
    let input_sub = transport.subscriber("image");
    let output = transport.publisher("detections", MaxSliceLen::const_new(4096), 0);
    let mut output_sub = transport.subscriber("detections");
    let mut entry = DetectorNodeEntry::new();
    entry
        .init(NodeContext::for_tests(
            [("detections".into(), AnyPublisher::Ipc(output))]
                .into_iter()
                .collect(),
            [("image_raw".into(), AnySubscriber::Ipc(input_sub))]
                .into_iter()
                .collect(),
        ))
        .unwrap();
    // Oracles are handwritten, independent of the detector's region scan.
    let expected = match DETECTION_SHIFT {
        0.0 => [
            11.0, 11.0, 20.0, 20.0, 51.0, 51.0, 20.0, 20.0, 91.0, 91.0, 20.0, 20.0,
        ],
        8.0 => [
            19.0, 11.0, 20.0, 20.0, 59.0, 51.0, 20.0, 20.0, 99.0, 91.0, 20.0, 20.0,
        ],
        _ => panic!("update the intended-regression oracle when changing the tutorial"),
    };
    for lit in [true, false, true] {
        {
            let mut image = input.loan_proxy::<Image>().unwrap();
            image.width = 128;
            image.height = 128;
            image.step = 128;
            image.set_header_bytes(&HEADER).unwrap();
            image.set_encoding("mono8").unwrap();
            let pixels = image.loan_data(128 * 128).unwrap();
            pixels.fill(0);
            if lit {
                for (start, value) in [(11, 230), (51, 204), (91, 179)] {
                    for row in start..start + 20 {
                        pixels[row * 128 + start..row * 128 + start + 20].fill(value);
                    }
                }
            }
        }
        entry.tick().unwrap();
        assert_eq!(
            output_sub
                .try_view::<DetectionArray, _>(|output| {
                    if lit {
                        assert_eq!(output.boxes(), expected);
                        assert_eq!(output.scores(), [0.9, 0.8, 0.7]);
                        assert_eq!(output.class_ids(), [0.0, 1.0, 2.0]);
                        let intersection = (31.0 - output.boxes()[0]) * 20.0;
                        let iou = intersection / (800.0 - intersection);
                        assert_eq!(
                            iou,
                            if DETECTION_SHIFT == 0.0 {
                                1.0
                            } else {
                                240.0 / 560.0
                            }
                        );
                        assert_eq!(iou >= 0.5, DETECTION_SHIFT == 0.0);
                    } else {
                        assert!(output.boxes().is_empty());
                        assert!(output.scores().is_empty());
                        assert!(output.class_ids().is_empty());
                    }
                })
                .unwrap(),
            Some(())
        );
    }
    {
        let mut image = input.loan_proxy::<Image>().unwrap();
        image.width = 1;
        image.height = 1;
        image.step = 1;
        image.set_header_bytes(&HEADER).unwrap();
        image.loan_encoding(1).unwrap().fill(0xff);
        image.loan_data(1).unwrap().fill(0);
    }
    assert!(entry
        .tick()
        .unwrap_err()
        .to_string()
        .contains("Invalid input 'image_raw'"));
    assert!(output_sub
        .try_view::<DetectionArray, _>(|_| ())
        .unwrap()
        .is_none());
    entry.shutdown().unwrap();
}
