// SPDX-License-Identifier: AGPL-3.0-only
package com.cerulion.android;

/** Nonblocking process-global runtime facade for Android apps. */
public final class CerulionRuntime {
    public static final int OK = 0;
    public static final int NOT_INITIALIZED = 1;
    public static final int ALREADY_INITIALIZED = 2;
    public static final int INVALID_ARGUMENT = 3;
    public static final int BAD_STATE = 4;
    public static final int GRAPH = 5;
    public static final int NETWORK = 6;
    public static final int RECORDER = 7;
    public static final int UNKNOWN_TOPIC = 8;
    public static final int ENCODE = 9;
    public static final int PUBLISH = 10;
    public static final int BUSY = 11;
    public static final int PANIC = 12;

    static {
        System.loadLibrary("cerulion_android");
    }

    private CerulionRuntime() {}

    /** Initialize the process runtime. Returns {@link #OK}, or a stable error code. */
    public static int init(String filesDir, String robotName) {
        return nativeInit(filesDir, robotName);
    }

    /** Queue startup and return immediately. Returns an accepted or stable error code. */
    public static int start(String configJson) {
        return nativeStart(configJson);
    }

    /** Publish without waiting for startup or shutdown. Returns a stable error code. */
    public static int publish(String topic, String json) {
        return nativePublish(topic, json);
    }

    /** Return the latest JSON status snapshot without waiting for runtime work. */
    public static String status() {
        return nativeStatus();
    }

    /**
     * Queue shutdown and return immediately. A stop issued while startup is
     * still in flight is accepted and runs once startup finishes. Returns an
     * accepted or stable error code.
     */
    public static int stop() {
        return nativeStop();
    }

    private static native int nativeInit(String filesDir, String robotName);

    private static native int nativeStart(String configJson);

    private static native int nativePublish(String topic, String json);

    private static native String nativeStatus();

    private static native int nativeStop();
}
