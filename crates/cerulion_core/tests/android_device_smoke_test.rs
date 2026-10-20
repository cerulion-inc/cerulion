// SPDX-License-Identifier: AGPL-3.0-only
//! Build with `cargo ndk -t x86_64 --platform 30 test -p cerulion_core --test
//! android_device_smoke_test --no-run`, then run the produced test binary on an
//! Android device with `--test-threads=1`.

use std::ffi::CString;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};

use cerulion_core::transport::{
    PublisherProvisioning, TopicServiceConfig, TransportConfig, TransportManager,
};
use cerulion_core::wire::MaxSliceLen;
use native_ros2_messages::geometry_msgs::Vector3;

static NEXT_ID: AtomicU64 = AtomicU64::new(0);

fn unique_name() -> String {
    format!(
        "android_smoke_{}_{}",
        std::process::id(),
        NEXT_ID.fetch_add(1, Ordering::Relaxed)
    )
}

fn smoke_iox2_config() -> iceoryx2::config::Config {
    cerulion_core::testing::iceoryx_test_config()
}

#[test]
fn a_topic_published_in_process_is_received_in_order() {
    let config = TransportConfig {
        node_name: unique_name(),
        ..Default::default()
    };
    let manager = TransportManager::init_for_test(config, smoke_iox2_config())
        .expect("initialize isolated transport");
    let topic = format!("/android_smoke/{}", unique_name());
    let subscriber = manager
        .create_subscriber_with_buffers(
            &topic,
            TopicServiceConfig::for_topology(
                manager.default_topic_config(),
                20,
                1,
                PublisherProvisioning::SingleWriter,
                0,
                0,
            ),
            20,
        )
        .expect("create twenty-message subscriber");
    let mut publisher = manager
        .create_publisher_simple(&topic, MaxSliceLen::const_new(64))
        .expect("create publisher");

    for seq in 0..20u32 {
        let mut message = publisher.loan_proxy::<Vector3>().expect("loan message");
        message.x = f64::from(seq);
        message.y = -0.25;
        message.z = 42.0;
    }

    let expected: Vec<_> = (0..20u32)
        .map(|seq| (f64::from(seq), -0.25, 42.0))
        .collect();
    let deadline = Instant::now() + Duration::from_secs(2);
    let mut received = Vec::new();
    while received.len() < expected.len() && Instant::now() < deadline {
        subscriber
            .try_receive(|message| {
                let payload = message.payload();
                assert_eq!(payload.len(), 24, "fixed-size Vector3 payload");
                let x = f64::from_le_bytes(payload[0..8].try_into().expect("x bytes"));
                let y = f64::from_le_bytes(payload[8..16].try_into().expect("y bytes"));
                let z = f64::from_le_bytes(payload[16..24].try_into().expect("z bytes"));
                received.push((x, y, z));
            })
            .expect("receive messages");
        std::thread::yield_now();
    }
    assert_eq!(
        received, expected,
        "deliver the twenty oracle messages in order"
    );
}

#[test]
fn a_named_shm_object_round_trips_through_posix_shm() {
    let name = CString::new(format!("/{}", unique_name())).expect("valid name");
    // SAFETY: `name` is a live NUL-terminated string for the duration of the call.
    let created = unsafe {
        cerulion_core::posix_shm::shm_open(
            name.as_ptr(),
            libc::O_CREAT | libc::O_EXCL | libc::O_RDWR,
            0o600,
        )
    };
    assert!(
        created >= 0,
        "create named object: {}",
        std::io::Error::last_os_error()
    );
    // SAFETY: `created` is an open descriptor for the object just created.
    assert_eq!(unsafe { libc::ftruncate(created, 4096) }, 0);
    // SAFETY: `created` refers to a 4096-byte object; the mapping is shared,
    // writable, starts at offset zero, and has a nonzero length.
    let first = unsafe {
        libc::mmap(
            std::ptr::null_mut(),
            4096,
            libc::PROT_READ | libc::PROT_WRITE,
            libc::MAP_SHARED,
            created,
            0,
        )
    };
    assert_ne!(first, libc::MAP_FAILED, "map created object");

    let pattern: Vec<u8> = (0..=255).cycle().take(4096).collect();
    // SAFETY: `first` was checked against MAP_FAILED and spans 4096 writable
    // bytes; `pattern` has the same length and does not overlap the mapping.
    unsafe {
        std::ptr::copy_nonoverlapping(pattern.as_ptr(), first.cast::<u8>(), pattern.len());
    }
    // SAFETY: `created` is the open descriptor and is not used again.
    assert_eq!(unsafe { libc::close(created) }, 0);

    // SAFETY: `name` is a live NUL-terminated string for the duration of the call.
    let reopened = unsafe { cerulion_core::posix_shm::shm_open(name.as_ptr(), libc::O_RDWR, 0) };
    assert!(
        reopened >= 0,
        "reopen named object: {}",
        std::io::Error::last_os_error()
    );
    // SAFETY: `reopened` refers to the same 4096-byte object; the mapping is
    // shared, writable, starts at offset zero, and has a nonzero length.
    let second = unsafe {
        libc::mmap(
            std::ptr::null_mut(),
            4096,
            libc::PROT_READ | libc::PROT_WRITE,
            libc::MAP_SHARED,
            reopened,
            0,
        )
    };
    assert_ne!(second, libc::MAP_FAILED, "map reopened object");
    // SAFETY: `second` was checked against MAP_FAILED and spans 4096 mapped bytes.
    let actual = unsafe { std::slice::from_raw_parts(second.cast::<u8>(), 4096) };
    assert_eq!(actual, pattern.as_slice());

    #[cfg(target_os = "android")]
    {
        let object_path = android_object_path(&name);
        assert!(object_path.exists(), "backing file exists before unlink");
    }

    // SAFETY: `first` is the live 4096-byte mapping created above.
    assert_eq!(unsafe { libc::munmap(first, 4096) }, 0);
    // SAFETY: `second` is the live 4096-byte mapping created above.
    assert_eq!(unsafe { libc::munmap(second, 4096) }, 0);
    // SAFETY: `reopened` is the open descriptor and is not used again.
    assert_eq!(unsafe { libc::close(reopened) }, 0);
    // SAFETY: `name` is a live NUL-terminated string for the duration of the call.
    assert_eq!(
        unsafe { cerulion_core::posix_shm::shm_unlink(name.as_ptr()) },
        0
    );

    #[cfg(target_os = "android")]
    assert!(
        !android_object_path(&name).exists(),
        "backing file is gone after unlink"
    );

    // SAFETY: `name` is a live NUL-terminated string for the duration of the call.
    let missing = unsafe { cerulion_core::posix_shm::shm_open(name.as_ptr(), libc::O_RDWR, 0) };
    assert_eq!(missing, -1);
    assert_eq!(
        std::io::Error::last_os_error().raw_os_error(),
        Some(libc::ENOENT)
    );
}

#[cfg(target_os = "android")]
fn android_object_path(name: &std::ffi::CStr) -> std::path::PathBuf {
    use std::os::unix::ffi::OsStrExt;

    let configured = std::env::var_os("CERULION_SHM_DIR")
        .filter(|value| !value.is_empty() && std::path::Path::new(value).is_absolute());
    let temporary = std::env::var_os("TMPDIR")
        .filter(|value| std::path::Path::new(value).is_absolute())
        .map(|value| std::path::PathBuf::from(value).join("cerulion-shm"));
    let dir = configured
        .map(std::path::PathBuf::from)
        .or(temporary)
        .unwrap_or_else(|| std::path::PathBuf::from("/data/local/tmp/cerulion-shm"));
    let object = std::ffi::OsStr::from_bytes(&name.to_bytes()[1..]);
    dir.join(object)
}
