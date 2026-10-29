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
    public static final int SPAWN = 13;
    public static final int DECODE = 14;
    public static final int EMPTY = 15;
    public static final int RECEIVE = 16;

    private static String loadedLibrary;

    private CerulionRuntime() {}

    /**
     * Load the app's native runtime library, the one library that links
     * Cerulion and the app's graph nodes, for example
     * {@code loadLibrary("myapp_runtime")} for {@code libmyapp_runtime.so}.
     * Call it once, before any other method. A second call with the same
     * name does nothing; a call with a different name throws
     * {@link IllegalStateException}, because one process holds one runtime.
     */
    public static synchronized void loadLibrary(String name) {
        if (loadedLibrary != null) {
            if (!loadedLibrary.equals(name)) {
                throw new IllegalStateException(
                        "CerulionRuntime is bound to lib" + loadedLibrary + ".so");
            }
            return;
        }
        System.loadLibrary(name);
        loadedLibrary = name;
    }

    private static synchronized void ensureLoaded() {
        if (loadedLibrary == null) {
            throw new IllegalStateException(
                    "call CerulionRuntime.loadLibrary(<your runtime library>) first");
        }
    }

    /**
     * Initialize the process runtime. Returns {@link #OK}, or a stable error
     * code. Throws {@link IllegalStateException} before {@link #loadLibrary}.
     */
    public static int init(String filesDir, String robotName) {
        ensureLoaded();
        return nativeInit(filesDir, robotName);
    }

    /**
     * Queue startup and return immediately. Returns an accepted or stable
     * error code. {@link #BUSY} means another start or stop is in flight, so
     * retry. A retry answered {@link #BAD_STATE} means a runtime is already
     * running, owned by whoever started it; this caller must not stop it.
     * A start that fails after it was queued, for example a graph node type
     * that is not linked into the library, shows in {@link #status()} as
     * state {@code failed} with {@code last_error.code}.
     */
    public static int start(String configJson) {
        ensureLoaded();
        return nativeStart(configJson);
    }

    /** Publish without waiting for startup or shutdown. Returns a stable error code. */
    public static int publish(String topic, String json) {
        ensureLoaded();
        return nativePublish(topic, json);
    }

    /**
     * Take the oldest message of a {@code subscribe} topic without waiting
     * and append it to {@code out} as JSON object text, for example
     * {@code {"data":"hello"}}. Returns {@link #OK} after appending, or
     * {@link #EMPTY} when the queue holds no message; any other value is a
     * stable error code, and only {@link #OK} appends. {@link #BUSY} means a
     * start or stop is in flight, so retry. {@link #DECODE} means the message
     * did not decode; it was still taken, so the next poll reads the one
     * after it. A full queue drops its oldest message; {@link #status()}
     * counts it under {@code subscribed}. A run whose state is
     * {@code finished} or {@code failed} can still be polled until
     * {@link #stop()}.
     */
    public static int poll(String topic, StringBuilder out) {
        ensureLoaded();
        return nativePoll(topic, out);
    }

    /** Return the latest JSON status snapshot without waiting for runtime work. */
    public static String status() {
        ensureLoaded();
        return nativeStatus();
    }

    /**
     * Queue shutdown and return immediately. A stop issued while startup is
     * still in flight is accepted and runs once startup finishes. Returns an
     * accepted or stable error code.
     */
    public static int stop() {
        ensureLoaded();
        return nativeStop();
    }

    private static native int nativeInit(String filesDir, String robotName);

    private static native int nativeStart(String configJson);

    private static native int nativePublish(String topic, String json);

    private static native int nativePoll(String topic, StringBuilder out);

    private static native String nativeStatus();

    private static native int nativeStop();
}
