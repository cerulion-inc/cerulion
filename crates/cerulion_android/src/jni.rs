// SPDX-License-Identifier: AGPL-3.0-only
//! JNI entry points for the Java app facade.

use std::borrow::Cow;
use std::panic::{catch_unwind, AssertUnwindSafe};

use jni::objects::{JClass, JObject, JString, JValue};
use jni::strings::JNIString;
use jni::sys::{jint, jlong, jstring};
use jni::{Env, EnvUnowned, Outcome};

use crate::error::codes;

#[no_mangle]
pub extern "system" fn Java_com_cerulion_android_CerulionRuntime_nativeInit<'caller>(
    mut env: EnvUnowned<'caller>,
    _class: JClass<'caller>,
    files_dir: JString<'caller>,
    robot_name: JString<'caller>,
) -> jint {
    guarded_with_env(&mut env, |env| {
        let files_dir = match java_string(env, &files_dir) {
            Ok(value) => value,
            Err(code) => return code,
        };
        let robot_name = match java_string(env, &robot_name) {
            Ok(value) => value,
            Err(code) => return code,
        };
        crate::host::init(&files_dir, &robot_name)
    })
}

#[no_mangle]
pub extern "system" fn Java_com_cerulion_android_CerulionRuntime_nativeStart<'caller>(
    mut env: EnvUnowned<'caller>,
    _class: JClass<'caller>,
    config_json: JString<'caller>,
) -> jint {
    guarded_with_env(&mut env, |env| match java_string(env, &config_json) {
        Ok(value) => crate::host::start(&value),
        Err(code) => code,
    })
}

#[no_mangle]
pub extern "system" fn Java_com_cerulion_android_CerulionRuntime_nativePublish<'caller>(
    mut env: EnvUnowned<'caller>,
    _class: JClass<'caller>,
    topic: JString<'caller>,
    json: JString<'caller>,
) -> jint {
    guarded_with_env(&mut env, |env| publish(env, &topic, &json, None))
}

#[no_mangle]
pub extern "system" fn Java_com_cerulion_android_CerulionRuntime_nativePublishStamped<'caller>(
    mut env: EnvUnowned<'caller>,
    _class: JClass<'caller>,
    topic: JString<'caller>,
    json: JString<'caller>,
    stamp_ns: jlong,
) -> jint {
    guarded_with_env(&mut env, |env| {
        // A stamp is nanoseconds on the runtime clock's base, so a negative
        // one names no time.
        let Ok(stamp_ns) = u64::try_from(stamp_ns) else {
            return codes::INVALID_ARGUMENT;
        };
        publish(env, &topic, &json, Some(stamp_ns))
    })
}

/// Publish `json` on `topic`, stamped `stamp_ns` or, when it is `None`, with
/// the runtime clock's time.
fn publish(
    env: &mut Env<'_>,
    topic: &JString<'_>,
    json: &JString<'_>,
    stamp_ns: Option<u64>,
) -> jint {
    let topic = match java_string(env, topic) {
        Ok(value) => value,
        Err(code) => return code,
    };
    // `GetStringLength` below dereferences `json` raw; a null message
    // must return the invalid-argument code, not crash the app.
    if json.is_null() {
        return codes::INVALID_ARGUMENT;
    }
    // Refuse an oversized message before `java_string` copies it: each
    // UTF-16 unit is at least one UTF-8 byte, so a UTF-16 length over the
    // publisher's JSON limit can never encode. The host runs this check
    // and the publish under one runtime lock, so the limit is the active
    // run's limit (a stop or restart cannot land in between), and only
    // when the publish could proceed (BUSY, BAD_STATE and the
    // unknown-topic code come first). The exact byte check in
    // `encode_json` still applies after conversion.
    let prepare = |limit: usize| {
        // SAFETY: `env` is the live JNIEnv for this call, `json` is a
        // non-null local ref from its arguments, and no Java exception is
        // pending: the only JNI calls before this one read `topic`, and any
        // failure there returns before `prepare` runs.
        let utf16_len = unsafe {
            let raw = env.get_raw();
            ((**raw).v1_1.GetStringLength)(raw, json.as_raw())
        };
        if utf16_len < 0 || utf16_len as usize > limit {
            return Err(codes::ENCODE);
        }
        java_string(env, json)
    };
    match stamp_ns {
        None => crate::host::publish_prepared(&topic, prepare),
        Some(stamp_ns) => crate::host::publish_stamped_prepared(&topic, stamp_ns, prepare),
    }
}

#[no_mangle]
pub extern "system" fn Java_com_cerulion_android_CerulionRuntime_nativePoll<'caller>(
    mut env: EnvUnowned<'caller>,
    _class: JClass<'caller>,
    topic: JString<'caller>,
    out: JObject<'caller>,
) -> jint {
    guarded_with_env(&mut env, |env| {
        let topic = match java_string(env, &topic) {
            Ok(value) => value,
            Err(code) => return code,
        };
        if out.is_null() {
            return codes::INVALID_ARGUMENT;
        }
        // The message is appended while the host holds the runtime lock,
        // because it borrows the subscription's buffer.
        crate::host::poll_with(&topic, |json| {
            let appended = env.new_string(json).and_then(|text| {
                env.call_method(
                    &out,
                    jni::jni_str!("append"),
                    jni::jni_sig!("(Ljava/lang/String;)Ljava/lang/StringBuilder;"),
                    &[JValue::Object(&text)],
                )
            });
            match appended {
                Ok(_) => codes::OK,
                // Out of memory: the Java exception is pending and is what
                // the caller sees when this call returns.
                Err(_) => {
                    throw_out_of_memory_unless_pending(env);
                    codes::PANIC
                }
            }
        })
    })
}

#[no_mangle]
pub extern "system" fn Java_com_cerulion_android_CerulionRuntime_nativeStatus<'caller>(
    mut env: EnvUnowned<'caller>,
    _class: JClass<'caller>,
) -> jstring {
    match catch_unwind(AssertUnwindSafe(|| {
        env.with_env_no_catch(|env| {
            Ok::<_, jni::errors::Error>(match env.new_string(crate::host::status()) {
                Ok(status) => status.into_raw(),
                // Out of memory: the Java exception is pending and is what
                // the caller sees when this call returns.
                Err(_) => {
                    throw_out_of_memory_unless_pending(env);
                    std::ptr::null_mut()
                }
            })
        })
        .into_outcome()
    })) {
        Ok(Outcome::Ok(status)) => status,
        // The closure above returns `Ok` on every path.
        Ok(Outcome::Err(_)) => std::ptr::null_mut(),
        Ok(Outcome::Panic(_)) | Err(_) => {
            crate::host::panic_code();
            std::ptr::null_mut()
        }
    }
}

#[no_mangle]
pub extern "system" fn Java_com_cerulion_android_CerulionRuntime_nativeStop<'caller>(
    _env: EnvUnowned<'caller>,
    _class: JClass<'caller>,
) -> jint {
    guarded_code(crate::host::stop)
}

fn guarded_code(operation: impl FnOnce() -> jint) -> jint {
    match catch_unwind(AssertUnwindSafe(operation)) {
        Ok(code) => code,
        Err(_) => crate::host::panic_code(),
    }
}

/// [`guarded_code`] with the call's [`Env`]. `operation` maps every JNI
/// failure to a status code itself, so the only outcome it can leave is `Ok`;
/// `with_env_no_catch` leaves panics to the `catch_unwind` here, which reports
/// them through the host as before.
fn guarded_with_env<'caller>(
    env: &mut EnvUnowned<'caller>,
    operation: impl FnOnce(&mut Env<'caller>) -> jint,
) -> jint {
    guarded_code(|| {
        match env
            .with_env_no_catch(|env| Ok::<_, jni::errors::Error>(operation(env)))
            .into_outcome()
        {
            Outcome::Ok(code) => code,
            Outcome::Err(_) | Outcome::Panic(_) => crate::host::panic_code(),
        }
    })
}

/// jni 0.22 clears the `OutOfMemoryError` a failed string or method lookup
/// raised and returns an error instead. Throw one again, unless an exception
/// is already pending, so the Java caller still sees the failure when the
/// native call returns.
fn throw_out_of_memory_unless_pending(env: &mut Env<'_>) {
    if !env.exception_check() {
        // A throw that fails leaves this path nothing else to report.
        let _ = env.throw_new(
            jni::jni_str!("java/lang/OutOfMemoryError"),
            jni::jni_str!("cerulion: out of memory"),
        );
    }
}

fn java_string(env: &Env<'_>, value: &JString<'_>) -> Result<String, jint> {
    if value.is_null() {
        return Err(codes::INVALID_ARGUMENT);
    }
    let java_value = value
        .mutf8_chars(env)
        .map_err(|_| codes::INVALID_ARGUMENT)?;
    let original_bytes = java_value.to_bytes().to_vec();
    let value: String = Cow::from(&java_value).into_owned();
    let round_trip = JNIString::from(value.as_str());
    if round_trip.to_bytes() != original_bytes {
        return Err(codes::INVALID_ARGUMENT);
    }
    Ok(value)
}
