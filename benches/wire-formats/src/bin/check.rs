//! Correctness check and bytes-on-the-wire table. Every format encodes into a
//! preallocated slot, the encoded bytes are copied to an aligned buffer, and
//! every read path must return the value computed from the source.
use std::sync::Arc;
use wire_formats::*;

fn main() {
    let topic: Arc<str> = Arc::from("/bench");
    let walker = cer_walker();
    println!(
        "| message | Cerulion | FlatBuffers | Cap'n Proto (flat) | Protobuf (prost) | payload |"
    );
    println!("|---|---:|---:|---:|---:|---:|");

    // (a) IMU
    let mut slot = AlignedBuf::new(4096);
    let n = cer_encode_imu(slot.bytes_mut(), &topic);
    let cer = AlignedBuf::from_slice(&slot.bytes()[..n]);
    let mut slot2 = AlignedBuf::new(4096);
    let mut fbld = fb_builder(slot2.bytes_mut());
    let fbn = fb_encode_imu(&mut fbld);
    let fbb = AlignedBuf::from_slice(fbld.finished_data());
    let (s, e) = (0usize, fbn);
    let cflat = capnp_flat_imu();
    let cbuf = AlignedBuf::from_slice(&cflat);
    let mut slot3 = vec![0u8; 4096];
    let pn = pb_encode_imu(&mut slot3);
    let x = expect_imu();
    assert_eq!(cer_read_imu(&cer.bytes()[..n]), x);
    assert_eq!(cer_walk_imu(&walker, &cer.bytes()[..n]), x);
    assert_eq!(fb_read_imu(&fbb.bytes()[..e - s], true), x);
    assert_eq!(fb_read_imu(&fbb.bytes()[..e - s], false), x);
    assert_eq!(capnp_read_imu(&cbuf.bytes()[..cflat.len()]), x);
    assert_eq!(pb_read_imu(&slot3[..pn]), x);
    {
        let mut sc = AlignedBuf::new(4096);
        let mut a = capnp::message::ScratchSpaceHeapAllocator::new(sc.bytes_mut());
        assert_eq!(capnp_encode_imu(&mut a), cflat.len());
    }
    println!(
        "| (a) IMU sample, 96 B of content | {n} | {} | {} | {pn} | 0 |",
        e - s,
        cflat.len()
    );

    // (b) Image 1 MiB, 6 MiB
    for mib in [1usize, 6] {
        let src = image_src(mib);
        let len = slot_len(src.data.len());
        let mut slot = AlignedBuf::new(len);
        let n = cer_encode_image(slot.bytes_mut(), &src, &topic);
        let cer = AlignedBuf::from_slice(&slot.bytes()[..n]);
        let mut slot2 = AlignedBuf::new(len);
        let mut fbld = fb_builder(slot2.bytes_mut());
        let fbn = fb_encode_image(&mut fbld, &src);
        let fbb = AlignedBuf::from_slice(fbld.finished_data());
        let (s, e) = (0usize, fbn);
        let cflat = capnp_flat_image(&src);
        let cbuf = AlignedBuf::from_slice(&cflat);
        let mut slot3 = vec![0u8; len];
        let pn = pb_encode_image(&mut slot3, &src);
        let pbytes = bytes::Bytes::copy_from_slice(&slot3[..pn]);
        let x = expect_image(&src);
        assert_eq!(cer_read_image(&cer.bytes()[..n]), x);
        assert_eq!(cer_walk_image(&walker, &cer.bytes()[..n]), x);
        assert_eq!(fb_read_image(&fbb.bytes()[..e - s], true), x);
        assert_eq!(fb_read_image(&fbb.bytes()[..e - s], false), x);
        assert_eq!(capnp_read_image(&cbuf.bytes()[..cflat.len()]), x);
        assert_eq!(pb_read_image_vec(&slot3[..pn]), x);
        assert_eq!(pb_read_image_bytes(&pbytes), x);
        {
            let mut sc = AlignedBuf::new(len);
            let mut a = capnp::message::ScratchSpaceHeapAllocator::new(sc.bytes_mut());
            assert_eq!(capnp_encode_image(&mut a, &src), cflat.len());
        }
        // The payload bytes must be the source bytes in every format.
        let r = native_ros2_messages::sensor_msgs::ImageShm::from_bytes(&cer.bytes()[40..n]);
        assert_eq!(r.data(), &src.data[..]);
        assert_eq!(r.encoding().unwrap(), src.encoding);
        // FINDING: with_header is flushed when the writer commits, so the
        // header sub-frame lands AFTER encoding + data. Its start inside the
        // payload is printed here; HeaderShm::from_bytes asserts 4-byte
        // alignment and panics on this frame unless the bytes are re-aligned.
        let hb = r.header_bytes();
        let hoff = hb.as_ptr() as usize - cer.bytes()[40..].as_ptr() as usize;
        println!("  [image {mib} MiB] header sub-frame at payload offset {hoff} (mod 4 = {}); HeaderShm::from_bytes(header_bytes()) would {}",
            hoff % 4, if hoff.is_multiple_of(4) { "succeed" } else { "PANIC (misaligned)" });
        let direct = std::panic::catch_unwind(|| {
            native_ros2_messages::std_msgs::HeaderShm::from_bytes(hb)
                .stamp
                .sec
        });
        println!(
            "  [image {mib} MiB] direct HeaderShm::from_bytes(header_bytes()): {}",
            if direct.is_ok() { "ok" } else { "panicked" }
        );
        let hcopy = AlignedBuf::from_slice(hb);
        let h = native_ros2_messages::std_msgs::HeaderShm::from_bytes(&hcopy.bytes()[..hb.len()]);
        assert_eq!(h.frame_id().unwrap(), FRAME_ID);
        assert_eq!((h.stamp.sec, h.stamp.nanosec), (STAMP_SEC, STAMP_NSEC));
        let fm = flatbuffers::root::<fb::Image>(&fbb.bytes()[..e - s]).unwrap();
        assert_eq!(fm.data().unwrap().bytes(), &src.data[..]);
        assert_eq!(fm.header().unwrap().frame_id().unwrap(), FRAME_ID);
        let p = <pb::ImageV as prost::Message>::decode(&slot3[..pn]).unwrap();
        assert_eq!(p.data, src.data);
        println!(
            "| (b) Image {mib} MiB | {n} | {} | {} | {pn} | {} |",
            e - s,
            cflat.len(),
            src.data.len()
        );
    }

    // (c) PointCloud2 4 MiB
    let src = cloud_src();
    let len = slot_len(src.data.len());
    let mut slot = AlignedBuf::new(len);
    let n = cer_encode_cloud(slot.bytes_mut(), &src, &topic);
    let cer = AlignedBuf::from_slice(&slot.bytes()[..n]);
    let mut slot2 = AlignedBuf::new(len);
    let mut fbld = fb_builder(slot2.bytes_mut());
    let fbn = fb_encode_cloud(&mut fbld, &src);
    let fbb = AlignedBuf::from_slice(fbld.finished_data());
    let (s, e) = (0usize, fbn);
    let cflat = capnp_flat_cloud(&src);
    let cbuf = AlignedBuf::from_slice(&cflat);
    let mut slot3 = vec![0u8; len];
    let pn = pb_encode_cloud(&mut slot3, &src);
    let pbytes = bytes::Bytes::copy_from_slice(&slot3[..pn]);
    let x = expect_cloud(&src);
    assert_eq!(cer_read_cloud(&cer.bytes()[..n]), x);
    assert_eq!(cer_walk_cloud(&walker, &cer.bytes()[..n]), x);
    assert_eq!(fb_read_cloud(&fbb.bytes()[..e - s], true), x);
    assert_eq!(fb_read_cloud(&fbb.bytes()[..e - s], false), x);
    assert_eq!(capnp_read_cloud(&cbuf.bytes()[..cflat.len()]), x);
    assert_eq!(pb_read_cloud_vec(&slot3[..pn]), x);
    assert_eq!(pb_read_cloud_bytes(&pbytes), x);
    // The walker decodes the canonical PointField[] blob into 4 elements.
    let v = walker.walk_by_hash(&cer.bytes()[..n]).unwrap();
    match v.field("fields") {
        Some(cerulion_core::codegen::FrameValueKind::NestedArray { elements, .. }) => {
            assert_eq!(elements.len(), 4)
        }
        other => panic!("fields decoded as {other:?}"),
    }
    println!(
        "| (c) PointCloud2 4 MiB | {n} | {} | {} | {pn} | {} |",
        e - s,
        cflat.len(),
        src.data.len()
    );
    {
        let mut sl = AlignedBuf::new(len);
        let mut b = fb_builder(sl.bytes_mut());
        for _ in 0..3 {
            fb_encode_cloud(&mut b, &src);
        }
        assert!(flatbuffers::root::<fb::PointCloud2>(
            &AlignedBuf::from_slice(b.finished_data()).bytes()[..b.finished_data().len()]
        )
        .is_ok());
    }
    println!();
    println!("Cerulion schema hashes: ImuSample {:#018x}, Image {:#018x}, PointCloud2 {:#018x}",
        <ImuSample as cerulion_core::message::ShmMessage>::SCHEMA_HASH,
        <native_ros2_messages::sensor_msgs::Image as cerulion_core::message::ShmMessage>::SCHEMA_HASH,
        <native_ros2_messages::sensor_msgs::PointCloud2 as cerulion_core::message::ShmMessage>::SCHEMA_HASH);
    println!("all reads returned the source values: OK");
}
