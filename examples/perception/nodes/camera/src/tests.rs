use super::*;
// Test-only isolated transport; node code uses the public prelude.
use cerulion_core::testing::TestTransport;

#[test]
fn camera_publishes_valid_fully_initialized_moving_rectangles() {
    let transport = TestTransport::with_buffer_size(4);
    let publisher = transport.publisher("image", MaxSliceLen::const_new(32768), 0);
    let mut subscriber = transport.subscriber("image");
    let mut entry = CameraNodeEntry::new();
    entry
        .init(NodeContext::for_tests(
            [("image_raw".into(), AnyPublisher::Ipc(publisher))]
                .into_iter()
                .collect(),
            Default::default(),
        ))
        .unwrap();
    // Independent first-rectangle origins for frames 1, 7 and 35; the
    // repeat catches retained slot bytes. Frame 7 separates the x/y periods.
    for (before, origin_x, origin_y) in [(0, 11, 11), (6, 10, 12), (34, 10, 10), (0, 11, 11)] {
        entry.inner.frame = before;
        entry.tick().unwrap();
        assert_eq!(
            subscriber
                .try_view::<Image, _>(|image| {
                    assert_eq!((image.width, image.height, image.step), (128, 128, 128));
                    assert_eq!(image.encoding().unwrap(), "mono8");
                    assert_eq!(image.data().len(), 16384);
                    assert_eq!(image.header_bytes().len(), 22);
                    for (offset, &value) in image.data().iter().enumerate() {
                        let (x, y) = (offset % 128, offset / 128);
                        let expected = [230, 204, 179]
                            .into_iter()
                            .enumerate()
                            .find_map(|(index, shade)| {
                                let left = origin_x + index * 40;
                                let top = origin_y + index * 40;
                                ((left..left + 20).contains(&x) && (top..top + 20).contains(&y))
                                    .then_some(shade)
                            })
                            .unwrap_or(0);
                        assert_eq!(value, expected, "pixel ({x}, {y})");
                    }
                })
                .unwrap(),
            Some(())
        );
    }
    entry.shutdown().unwrap();
}
