use super::*;
// Test-only isolated transport; node code uses the public prelude.
use cerulion_core::testing::TestTransport;

#[test]
fn scan_delivers_all_beams_on_both_sides_of_the_threshold() {
    let transport = TestTransport::with_buffer_size(4);
    let publisher = transport.publisher("scan", MaxSliceLen::const_new(4096), 0);
    let mut subscriber = transport.subscriber("scan");
    let mut entry = LaserScannerNodeEntry::new();
    entry
        .init(NodeContext::for_tests(
            [("scan".into(), AnyPublisher::Ipc(publisher))]
                .into_iter()
                .collect(),
            Default::default(),
        ))
        .unwrap();
    for (before, nearest) in [(0, 0.3), (98, 0.3), (99, 5.0), (199, 0.3), (u32::MAX, 0.3)] {
        entry.inner.tick_count = before;
        entry.tick().unwrap();
        assert_eq!(
            subscriber
                .try_view::<LaserScan, _>(|scan| {
                    assert_eq!(scan.ranges(), [nearest; 180]);
                    assert!(scan.intensities().is_empty());
                    assert_eq!(scan.scan_time, 0.02);
                    assert!(
                        (scan.angle_min + 179.0 * scan.angle_increment - scan.angle_max).abs()
                            < 1e-6
                    );
                    assert!(scan.header_bytes().ends_with(b"laser"));
                })
                .unwrap(),
            Some(())
        );
    }
    entry.shutdown().unwrap();
}
