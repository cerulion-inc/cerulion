// SPDX-License-Identifier: AGPL-3.0-only
//! JNI entry points for the Java app facade.

use std::borrow::Cow;
use std::panic::{catch_unwind, AssertUnwindSafe};

use jni::objects::{JClass, JString};
use jni::strings::JNIString;
use jni::sys::{jint, jstring};
use jni::JNIEnv;

use crate::error::codes;

#[no_mangle]
pub extern "system" fn Java_com_cerulion_android_CerulionRuntime_nativeInit(
    mut env: JNIEnv,
    _class: JClass,
    files_dir: JString,
    robot_name: JString,
) -> jint {
    guarded_code(|| {
        let files_dir = match java_string(&mut env, &files_dir) {
            Ok(value) => value,
            Err(code) => return code,
        };
        let robot_name = match java_string(&mut env, &robot_name) {
            Ok(value) => value,
            Err(code) => return code,
        };
        crate::host::init(&files_dir, &robot_name)
    })
}

#[no_mangle]
pub extern "system" fn Java_com_cerulion_android_CerulionRuntime_nativeStart(
    mut env: JNIEnv,
    _class: JClass,
    config_json: JString,
) -> jint {
    guarded_code(|| match java_string(&mut env, &config_json) {
        Ok(value) => crate::host::start(&value),
        Err(code) => code,
    })
}

#[no_mangle]
pub extern "system" fn Java_com_cerulion_android_CerulionRuntime_nativePublish(
    mut env: JNIEnv,
    _class: JClass,
    topic: JString,
    json: JString,
) -> jint {
    guarded_code(|| {
        let topic = match java_string(&mut env, &topic) {
            Ok(value) => value,
            Err(code) => return code,
        };
        // Refuse an oversized message before `java_string` copies it: each
        // UTF-16 unit is at least one UTF-8 byte, so a UTF-16 length over the
        // publisher's JSON limit can never encode. The exact byte check in
        // `encode_json` still applies after conversion.
        if let Some(limit) = crate::host::publish_json_limit(&topic) {
            // SAFETY: `env` is the live JNIEnv for this call and `json` is a
            // local ref from its arguments.
            let utf16_len = unsafe {
                let raw = env.get_raw();
                let get_length = (**raw)
                    .GetStringLength
                    .expect("GetStringLength is in JNI 1.0");
                get_length(raw, json.as_raw())
            };
            if utf16_len < 0 || utf16_len as usize > limit {
                return codes::ENCODE;
            }
        }
        let json = match java_string(&mut env, &json) {
            Ok(value) => value,
            Err(code) => return code,
        };
        crate::host::publish(&topic, &json)
    })
}

#[no_mangle]
pub extern "system" fn Java_com_cerulion_android_CerulionRuntime_nativeStatus(
    env: JNIEnv,
    _class: JClass,
) -> jstring {
    match catch_unwind(AssertUnwindSafe(|| {
        env.new_string(crate::host::status())
            .map(JString::into_raw)
            .unwrap_or(std::ptr::null_mut())
    })) {
        Ok(value) => value,
        Err(_) => {
            crate::host::panic_code();
            std::ptr::null_mut()
        }
    }
}

#[no_mangle]
pub extern "system" fn Java_com_cerulion_android_CerulionRuntime_nativeStop(
    _env: JNIEnv,
    _class: JClass,
) -> jint {
    guarded_code(crate::host::stop)
}

fn guarded_code(operation: impl FnOnce() -> jint) -> jint {
    match catch_unwind(AssertUnwindSafe(operation)) {
        Ok(code) => code,
        Err(_) => crate::host::panic_code(),
    }
}

fn java_string(env: &mut JNIEnv, value: &JString) -> Result<String, jint> {
    if value.is_null() {
        return Err(codes::INVALID_ARGUMENT);
    }
    let java_value = env.get_string(value).map_err(|_| codes::INVALID_ARGUMENT)?;
    let original_bytes = java_value.to_bytes().to_vec();
    let value: String = Cow::from(&java_value).into_owned();
    let round_trip = JNIString::from(value.as_str());
    if round_trip.to_bytes() != original_bytes {
        return Err(codes::INVALID_ARGUMENT);
    }
    Ok(value)
}
