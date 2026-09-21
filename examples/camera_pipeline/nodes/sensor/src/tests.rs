use super::*;
// Isolated transport is test support, not part of the example's node API.
use cerulion_core::testing::TestTransport;

#[test]
fn source_delivers_a_complete_image_and_wraps_its_shade() {
    let transport = TestTransport::with_buffer_size(4);
    let publisher = transport.publisher("image", MaxSliceLen::const_new(1_000_000), 0);
    let mut subscriber = transport.subscriber("image");
    let mut entry = SensorNodeEntry::new();
    entry
        .init(NodeContext::for_tests(
            [("image".into(), AnyPublisher::Ipc(publisher))]
                .into_iter()
                .collect(),
            Default::default(),
        ))
        .unwrap();
    for (before, expected) in [(0, 1), (127, 128), (255, 0), (u32::MAX, 0)] {
        entry.inner.frame_count = before;
        entry.tick().unwrap();
        let delivered = subscriber
            .try_view::<Image, _>(|image| {
                assert_eq!((image.width, image.height, image.step), (640, 480, 1920));
                assert_eq!(image.encoding().unwrap(), "rgb8");
                assert_eq!(image.is_bigendian, 0);
                assert_eq!(image.data().len(), 921_600);
                assert!(image.data().iter().all(|&v| v == expected));
                assert_eq!(image.header_bytes().len(), 22);
                assert!(image.header_bytes().ends_with(b"camera"));
            })
            .unwrap();
        assert_eq!(delivered, Some(()), "observe the actual published frame");
    }
    entry.shutdown().unwrap();
}
