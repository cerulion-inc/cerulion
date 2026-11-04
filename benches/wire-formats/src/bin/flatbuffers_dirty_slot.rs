//! A FlatBuffers builder over a slot that still holds an earlier message
//! (here a Cerulion frame) produces a buffer the FlatBuffers verifier rejects.
use std::sync::Arc;
use wire_formats::*;
fn main() {
    let topic: Arc<str> = Arc::from("/bench");
    let src = cloud_src();
    let len = slot_len(src.data.len());
    for dirty in [false, true] {
        let mut slot = AlignedBuf::new(len);
        if dirty {
            cer_encode_cloud(slot.bytes_mut(), &src, &topic);
        }
        let mut b = fb_builder(slot.bytes_mut());
        let n = fb_encode_cloud(&mut b, &src);
        let fbb = AlignedBuf::from_slice(b.finished_data());
        let r = flatbuffers::root::<fb::PointCloud2>(&fbb.bytes()[..n]);
        println!(
            "slot previously used={dirty} len={n} verify_ok={}",
            r.is_ok()
        );
        if let Err(x) = r {
            println!("  {x:?}");
        }
    }
}
