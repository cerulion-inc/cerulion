// SPDX-License-Identifier: AGPL-3.0-only
//! Native Android account TLS uses system roots without a Java verifier.

use super::*;

#[cfg(test)]
thread_local! {
    static TEST_ROOTS: std::cell::RefCell<Option<PathBuf>> = const { std::cell::RefCell::new(None) };
}

pub(super) fn configure(
    builder: reqwest::blocking::ClientBuilder,
) -> CliResult<reqwest::blocking::ClientBuilder> {
    #[cfg(test)]
    if let Some(path) = TEST_ROOTS.with(|roots| roots.borrow().clone()) {
        return roots_from_directories(&[&path]).map(|roots| builder.tls_certs_only(roots));
    }
    #[cfg(target_os = "android")]
    {
        // Conscrypt's updatable store takes precedence on newer Android releases.
        // Neither directory needs a JVM, and neither contains user-added roots.
        roots_from_directories(&[
            Path::new("/apex/com.android.conscrypt/cacerts"),
            Path::new("/system/etc/security/cacerts"),
        ])
        .map(|roots| builder.tls_certs_only(roots))
    }
    #[cfg(not(target_os = "android"))]
    Ok(builder)
}

fn roots_from_directories(directories: &[&Path]) -> CliResult<Vec<reqwest::Certificate>> {
    let invalid =
        |reason| CliError::Login(format!("could not build a secure HTTP client: {reason}"));
    for directory in directories {
        let entries = match std::fs::read_dir(directory) {
            Ok(entries) => entries,
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => continue,
            Err(_) => return Err(invalid("Android system certificate roots are unreadable")),
        };
        let mut paths = entries
            .map(|entry| entry.map(|entry| entry.path()))
            .collect::<Result<Vec<_>, _>>()
            .map_err(|_| invalid("Android system certificate roots are unreadable"))?;
        paths.sort();
        let mut roots = Vec::new();
        for path in paths {
            if !path.is_file() {
                continue;
            }
            let pem = std::fs::read(path)
                .map_err(|_| invalid("an Android system certificate root is unreadable"))?;
            let certificates = reqwest::Certificate::from_pem_bundle(&pem)
                .map_err(|_| invalid("an Android system certificate root is invalid"))?;
            if certificates.len() != 1 {
                return Err(invalid("an Android system certificate root is invalid"));
            }
            roots.extend(certificates);
        }
        if !roots.is_empty() {
            return Ok(roots);
        }
    }
    Err(invalid(
        "Android system certificate roots are unavailable; local topics remain available",
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::Arc;

    fn with_roots<T>(path: &Path, f: impl FnOnce() -> T) -> T {
        struct Reset;
        impl Drop for Reset {
            fn drop(&mut self) {
                TEST_ROOTS.with(|roots| *roots.borrow_mut() = None);
            }
        }
        TEST_ROOTS.with(|roots| *roots.borrow_mut() = Some(path.to_owned()));
        let _reset = Reset;
        f()
    }

    #[test]
    fn native_android_account_tls_builds_with_system_roots_without_a_jvm() {
        let _environment = crate::test_env::env_lock();
        let directory = tempfile::tempdir().unwrap();
        // A public ISRG authority certificate; no private key is present.
        std::fs::write(
            directory.path().join("root.0"),
            include_bytes!("../tests/fixtures/account-root.pem"),
        )
        .unwrap();
        assert_eq!(
            roots_from_directories(&[directory.path()]).unwrap().len(),
            1
        );
        with_roots(directory.path(), || http_client().unwrap());
        let source = include_str!("android_tls.rs");
        let implementation = source.split("#[cfg(test)]\nmod tests").next().unwrap();
        assert!(implementation.contains("builder.tls_certs_only(roots)"));
        assert!(!implementation.contains("tls_danger_accept"));
        assert!(!implementation.contains("platform_verifier::"));
    }

    #[test]
    fn absent_and_invalid_native_android_roots_return_errors_without_panicking() {
        let _environment = crate::test_env::env_lock();
        let directory = tempfile::tempdir().unwrap();
        with_roots(directory.path(), || {
            assert!(std::panic::catch_unwind(http_client)
                .unwrap()
                .unwrap_err()
                .to_string()
                .contains("Android system certificate roots are unavailable"));
        });
        std::fs::write(directory.path().join("root.0"), b"not a certificate").unwrap();
        with_roots(directory.path(), || {
            assert!(std::panic::catch_unwind(http_client)
                .unwrap()
                .unwrap_err()
                .to_string()
                .contains("Android system certificate root is invalid"));
        });
    }

    #[test]
    #[tracing_test::traced_test]
    fn local_echo_delivers_after_an_account_client_build_failure() {
        let _environment = crate::test_env::env_lock();
        // Begin a new failure regime, independently of other test ordering.
        let _working_client = http_client().unwrap();
        let home = tempfile::tempdir().unwrap();
        struct RestoreHome(Option<std::ffi::OsString>);
        impl Drop for RestoreHome {
            fn drop(&mut self) {
                match &self.0 {
                    Some(value) => std::env::set_var("CERULION_HOME", value),
                    None => std::env::remove_var("CERULION_HOME"),
                }
            }
        }
        let _home = RestoreHome(std::env::var_os("CERULION_HOME"));
        std::env::set_var("CERULION_HOME", home.path());
        auth::seed_logged_in_at(home.path(), "account-with-expired-session").unwrap();
        let auth_path = home.path().join("auth.json");
        let mut stale = auth::load_from(&auth_path).state().unwrap().clone();
        stale.expires_at_ns = 1;
        auth::write_to(&auth_path, &stale).unwrap();
        let unavailable_roots = tempfile::tempdir().unwrap();
        let running = Arc::new(AtomicBool::new(true));
        struct Delivery {
            bytes: Vec<u8>,
            running: Arc<AtomicBool>,
        }
        impl Write for Delivery {
            fn write(&mut self, bytes: &[u8]) -> std::io::Result<usize> {
                self.bytes.extend_from_slice(bytes);
                if self.bytes.windows(4).any(|window| window == b"3.25") {
                    self.running.store(false, Ordering::SeqCst);
                }
                Ok(bytes.len())
            }
            fn flush(&mut self) -> std::io::Result<()> {
                Ok(())
            }
        }
        let topic = format!("/account_client_failure_{}", std::process::id());
        let schema = cerulion_core::codegen::parse_rosmsg(
            "float64 x\nfloat64 y\nfloat64 z\n",
            "Vector3",
            Some("geometry_msgs"),
        )
        .unwrap();
        let (walker, warnings) = cerulion_core::codegen::FrameWalker::new(vec![schema]);
        assert!(warnings.is_empty());
        let hash = walker.schema_hash_for("geometry_msgs/Vector3").unwrap();
        let producing = running.clone();
        let producer_topic = topic.clone();
        let (ready, started) = std::sync::mpsc::channel();
        let producer = std::thread::spawn(move || {
            let manager = cerulion_core::TransportManager::get_or_init().unwrap();
            let mut publisher = manager
                .create_publisher_simple(
                    &producer_topic,
                    cerulion_core::wire::MaxSliceLen::const_new(256),
                )
                .unwrap();
            let header = cerulion_core::WireHeader {
                schema_hash: hash,
                total_size: 64,
                offset_table_offset: 64,
                offset_table_count: 0,
                sequence: 19,
                timestamp_ns: 123_456_789,
                step_tag: cerulion_core::wire::StepTag::UNTAGGED,
            };
            let mut frame = [0; 64];
            header.write_to_buf(&mut frame);
            for (index, value) in [1.5_f64, -2.5, 3.25].into_iter().enumerate() {
                let offset = 40 + index * 8;
                frame[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
            }
            ready.send(()).unwrap();
            let deadline = Instant::now() + Duration::from_secs(5);
            while producing.load(Ordering::SeqCst) && Instant::now() < deadline {
                publisher.publish_raw(&frame).unwrap();
                std::thread::sleep(Duration::from_millis(10));
            }
            producing.store(false, Ordering::SeqCst);
        });
        started.recv_timeout(Duration::from_secs(5)).unwrap();
        let mut delivery = Delivery {
            bytes: Vec::new(),
            running: running.clone(),
        };
        let echoed = with_roots(unavailable_roots.path(), || {
            crate::topic_cmd::topic_echo_with_selection(
                &topic,
                None,
                running,
                &mut delivery,
                16,
                crate::topic_cmd::TopicScope::Automatic,
                None,
            )
        });
        producer.join().unwrap();
        echoed.unwrap();
        with_roots(unavailable_roots.path(), || assert!(http_client().is_err()));
        let output = String::from_utf8(delivery.bytes).unwrap();
        for expected in ["x: 1.5", "y: -2.5", "z: 3.25"] {
            assert!(output.contains(expected), "{output}");
        }
        assert_eq!(auth::load_from(&auth_path).state().unwrap(), &stale);
        logs_assert(|lines| {
            let failures = lines
                .iter()
                .filter(|line| {
                    line.contains(" WARN ") && line.contains("could not build a secure HTTP client")
                })
                .count();
            if failures == 1 {
                Ok(())
            } else {
                Err(format!("client build failures flooded: {failures}"))
            }
        });
    }
}
