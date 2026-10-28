use std::hint::black_box;
use std::sync::Arc;
use std::time::Duration;

use capnp::message::ScratchSpaceHeapAllocator;
use criterion::{criterion_group, criterion_main, Criterion, Throughput};
use wire_formats::*;

fn cfg() -> Criterion {
    Criterion::default()
        .sample_size(100)
        .warm_up_time(Duration::from_secs(2))
        .measurement_time(Duration::from_secs(5))
}

fn encode(c: &mut Criterion) {
    let topic: Arc<str> = Arc::from("/bench");

    let mut g = c.benchmark_group("encode/a_imu");
    g.throughput(Throughput::Elements(1));
    let mut slot = AlignedBuf::new(4096);
    g.bench_function("cerulion", |b| {
        b.iter(|| black_box(cer_encode_imu(black_box(slot.bytes_mut()), &topic)))
    });
    {
        let mut fs = AlignedBuf::new(4096);
        let mut fbld = fb_builder(fs.bytes_mut());
        g.bench_function("flatbuffers", |b| {
            b.iter(|| black_box(fb_encode_imu(black_box(&mut fbld))))
        });
    }
    {
        let mut sc = AlignedBuf::new(4096);
        let mut a = ScratchSpaceHeapAllocator::new(sc.bytes_mut());
        g.bench_function("capnp", |b| {
            b.iter(|| black_box(capnp_encode_imu(black_box(&mut a))))
        });
    }
    g.bench_function("prost", |b| {
        b.iter(|| black_box(pb_encode_imu(black_box(slot.bytes_mut()))))
    });
    g.finish();

    for mib in [1usize, 6] {
        let src = image_src(mib);
        let len = slot_len(src.data.len());
        let mut g = c.benchmark_group(format!("encode/b_image_{mib}MiB"));
        g.throughput(Throughput::Bytes(src.data.len() as u64));
        let mut slot = AlignedBuf::new(len);
        g.bench_function("cerulion", |b| {
            b.iter(|| black_box(cer_encode_image(slot.bytes_mut(), black_box(&src), &topic)))
        });
        {
            let mut fs = AlignedBuf::new(len);
            let mut fbld = fb_builder(fs.bytes_mut());
            g.bench_function("flatbuffers", |b| {
                b.iter(|| black_box(fb_encode_image(&mut fbld, black_box(&src))))
            });
        }
        {
            let mut sc = AlignedBuf::new(len);
            let mut a = ScratchSpaceHeapAllocator::new(sc.bytes_mut());
            g.bench_function("capnp", |b| {
                b.iter(|| black_box(capnp_encode_image(&mut a, black_box(&src))))
            });
        }
        g.bench_function("prost", |b| {
            b.iter(|| black_box(pb_encode_image(slot.bytes_mut(), black_box(&src))))
        });
        g.finish();
    }

    let src = cloud_src();
    let len = slot_len(src.data.len());
    let mut g = c.benchmark_group("encode/c_cloud_4MiB");
    g.throughput(Throughput::Bytes(src.data.len() as u64));
    let mut slot = AlignedBuf::new(len);
    g.bench_function("cerulion", |b| {
        b.iter(|| black_box(cer_encode_cloud(slot.bytes_mut(), black_box(&src), &topic)))
    });
    {
        let mut fs = AlignedBuf::new(len);
        let mut fbld = fb_builder(fs.bytes_mut());
        g.bench_function("flatbuffers", |b| {
            b.iter(|| black_box(fb_encode_cloud(&mut fbld, black_box(&src))))
        });
    }
    {
        let mut sc = AlignedBuf::new(len);
        let mut a = ScratchSpaceHeapAllocator::new(sc.bytes_mut());
        g.bench_function("capnp", |b| {
            b.iter(|| black_box(capnp_encode_cloud(&mut a, black_box(&src))))
        });
    }
    g.bench_function("prost", |b| {
        b.iter(|| black_box(pb_encode_cloud(slot.bytes_mut(), black_box(&src))))
    });
    g.finish();
}

fn read(c: &mut Criterion) {
    let topic: Arc<str> = Arc::from("/bench");
    let walker = cer_walker();

    // (a)
    {
        let mut slot = AlignedBuf::new(4096);
        let n = cer_encode_imu(slot.bytes_mut(), &topic);
        let cer = AlignedBuf::from_slice(&slot.bytes()[..n]);
        let mut fs = AlignedBuf::new(4096);
        let mut fbld = fb_builder(fs.bytes_mut());
        let (s, e) = (0usize, fb_encode_imu(&mut fbld));
        let fbb = AlignedBuf::from_slice(fbld.finished_data());
        let cflat = capnp_flat_imu();
        let cbuf = AlignedBuf::from_slice(&cflat);
        let mut pbuf = vec![0u8; 4096];
        let pn = pb_encode_imu(&mut pbuf);
        pbuf.truncate(pn);
        let (cer, fbb, cbuf) = (
            &cer.bytes()[..n],
            &fbb.bytes()[..e - s],
            &cbuf.bytes()[..cflat.len()],
        );
        let mut g = c.benchmark_group("read/a_imu");
        g.bench_function("cerulion_typed", |b| {
            b.iter(|| black_box(cer_read_imu(black_box(cer))))
        });
        g.bench_function("cerulion_walker", |b| {
            b.iter(|| black_box(cer_walk_imu(&walker, black_box(cer))))
        });
        g.bench_function("flatbuffers_verify", |b| {
            b.iter(|| black_box(fb_read_imu(black_box(fbb), true)))
        });
        g.bench_function("flatbuffers_noverify", |b| {
            b.iter(|| black_box(fb_read_imu(black_box(fbb), false)))
        });
        g.bench_function("capnp", |b| {
            b.iter(|| black_box(capnp_read_imu(black_box(cbuf))))
        });
        g.bench_function("prost", |b| {
            b.iter(|| black_box(pb_read_imu(black_box(&pbuf))))
        });
        g.finish();
    }

    // (b)
    for mib in [1usize, 6] {
        let src = image_src(mib);
        let len = slot_len(src.data.len());
        let mut slot = AlignedBuf::new(len);
        let n = cer_encode_image(slot.bytes_mut(), &src, &topic);
        let cer = AlignedBuf::from_slice(&slot.bytes()[..n]);
        let mut fs = AlignedBuf::new(len);
        let mut fbld = fb_builder(fs.bytes_mut());
        let (s, e) = (0usize, fb_encode_image(&mut fbld, &src));
        let fbb = AlignedBuf::from_slice(fbld.finished_data());
        let cflat = capnp_flat_image(&src);
        let cbuf = AlignedBuf::from_slice(&cflat);
        let mut pbuf = vec![0u8; len];
        let pn = pb_encode_image(&mut pbuf, &src);
        pbuf.truncate(pn);
        let pbytes = bytes::Bytes::from(pbuf.clone());
        let (cer, fbb, cbuf) = (
            &cer.bytes()[..n],
            &fbb.bytes()[..e - s],
            &cbuf.bytes()[..cflat.len()],
        );
        let mut g = c.benchmark_group(format!("read/b_image_{mib}MiB"));
        g.bench_function("cerulion_typed", |b| {
            b.iter(|| black_box(cer_read_image(black_box(cer))))
        });
        g.bench_function("cerulion_walker", |b| {
            b.iter(|| black_box(cer_walk_image(&walker, black_box(cer))))
        });
        g.bench_function("flatbuffers_verify", |b| {
            b.iter(|| black_box(fb_read_image(black_box(fbb), true)))
        });
        g.bench_function("flatbuffers_noverify", |b| {
            b.iter(|| black_box(fb_read_image(black_box(fbb), false)))
        });
        g.bench_function("capnp", |b| {
            b.iter(|| black_box(capnp_read_image(black_box(cbuf))))
        });
        g.bench_function("prost_vec", |b| {
            b.iter(|| black_box(pb_read_image_vec(black_box(&pbuf))))
        });
        g.bench_function("prost_bytes", |b| {
            b.iter(|| black_box(pb_read_image_bytes(black_box(&pbytes))))
        });
        g.finish();
    }

    // (c)
    {
        let src = cloud_src();
        let len = slot_len(src.data.len());
        let mut slot = AlignedBuf::new(len);
        let n = cer_encode_cloud(slot.bytes_mut(), &src, &topic);
        let cer = AlignedBuf::from_slice(&slot.bytes()[..n]);
        let mut fs = AlignedBuf::new(len);
        let mut fbld = fb_builder(fs.bytes_mut());
        let (s, e) = (0usize, fb_encode_cloud(&mut fbld, &src));
        let fbb = AlignedBuf::from_slice(fbld.finished_data());
        let cflat = capnp_flat_cloud(&src);
        let cbuf = AlignedBuf::from_slice(&cflat);
        let mut pbuf = vec![0u8; len];
        let pn = pb_encode_cloud(&mut pbuf, &src);
        pbuf.truncate(pn);
        let pbytes = bytes::Bytes::from(pbuf.clone());
        let (cer, fbb, cbuf) = (
            &cer.bytes()[..n],
            &fbb.bytes()[..e - s],
            &cbuf.bytes()[..cflat.len()],
        );
        let mut g = c.benchmark_group("read/c_cloud_4MiB");
        g.bench_function("cerulion_typed", |b| {
            b.iter(|| black_box(cer_read_cloud(black_box(cer))))
        });
        g.bench_function("cerulion_walker", |b| {
            b.iter(|| black_box(cer_walk_cloud(&walker, black_box(cer))))
        });
        g.bench_function("flatbuffers_verify", |b| {
            b.iter(|| black_box(fb_read_cloud(black_box(fbb), true)))
        });
        g.bench_function("flatbuffers_noverify", |b| {
            b.iter(|| black_box(fb_read_cloud(black_box(fbb), false)))
        });
        g.bench_function("capnp", |b| {
            b.iter(|| black_box(capnp_read_cloud(black_box(cbuf))))
        });
        g.bench_function("prost_vec", |b| {
            b.iter(|| black_box(pb_read_cloud_vec(black_box(&pbuf))))
        });
        g.bench_function("prost_bytes", |b| {
            b.iter(|| black_box(pb_read_cloud_bytes(black_box(&pbytes))))
        });
        g.finish();
    }
}

criterion_group! { name = benches; config = cfg(); targets = encode, read }
criterion_main!(benches);
