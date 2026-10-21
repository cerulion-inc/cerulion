// SPDX-License-Identifier: AGPL-3.0-only
package com.cerulion.sample;

import android.app.Activity;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;

import com.cerulion.android.CerulionRuntime;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;

public final class MainActivity extends Activity {
    private static final String TAG = "CerulionSample";

    private final Handler handler = new Handler(Looper.getMainLooper());
    private final Runnable startRetry = this::startRuntime;
    private final Runnable stopRetry = this::stopRuntime;
    private boolean stopRequested;
    private boolean ownsRuntime;
    private long tick;
    private String lastLoggedState;

    private final Runnable heartbeat = new Runnable() {
        @Override
        public void run() {
            if (stopRequested) {
                return;
            }
            // App-private stop control: only the app (or run-as on a
            // debuggable build) can create files/stop, and only the instance
            // that started the runtime may stop it.
            File stopFile = new File(getFilesDir(), "stop");
            if (stopFile.isFile() && stopFile.delete() && ownsRuntime) {
                stopRuntime();
                return;
            }
            String status = CerulionRuntime.status();
            // status() reports null before init or after a teardown race.
            if (status == null) {
                return;
            }
            try {
                String state = new JSONObject(status).getString("state");
                if ("starting".equals(state)) {
                    lastLoggedState = null;
                    handler.postDelayed(this, 500);
                    return;
                }
                if (!"running".equals(state)) {
                    if (!state.equals(lastLoggedState)) {
                        Log.e(TAG, "heartbeat not running state=" + state + " status=" + status);
                        lastLoggedState = state;
                    }
                    if (("finished".equals(state) || "failed".equals(state)) && ownsRuntime) {
                        stopRuntime();
                    }
                    return;
                }
                lastLoggedState = null;
                int code = CerulionRuntime.publish(
                        "/app/heartbeat", "{\"data\":\"tick " + tick + "\"}");
                if (code == CerulionRuntime.BUSY) {
                    handler.postDelayed(this, 50);
                    return;
                }
                if (code == CerulionRuntime.OK && (tick + 1) % 4 == 0) {
                    Log.i(TAG, "publish=" + code + " status=" + status);
                }
                if (code == CerulionRuntime.OK) {
                    tick++;
                } else {
                    Log.e(TAG, "publish=" + code + " status=" + status);
                }
                handler.postDelayed(this, 500);
            } catch (JSONException error) {
                if (!"status_parse_error".equals(lastLoggedState)) {
                    Log.e(TAG, "heartbeat status parse failed status=" + status, error);
                    lastLoggedState = "status_parse_error";
                }
            }
        }
    };

    @Override
    protected void onCreate(Bundle state) {
        super.onCreate(state);
        int init = CerulionRuntime.init(getFilesDir().getAbsolutePath(), "android-app");
        if (init != CerulionRuntime.OK) {
            Log.e(TAG, "init=" + init);
            return;
        }
        startRuntime();
    }

    private void startRuntime() {
        try {
            int start = CerulionRuntime.start(configJson());
            Log.i(TAG, "start=" + start);
            if (start == CerulionRuntime.OK) {
                ownsRuntime = true;
                handler.postDelayed(heartbeat, 500);
            } else if (start == CerulionRuntime.BUSY) {
                // Another start or stop is still in flight; retry unless
                // this activity is already going away.
                if (!isFinishing() && !isDestroyed()) {
                    handler.postDelayed(startRetry, 100);
                }
            } else {
                // BAD_STATE means a runtime is already running, owned by
                // whoever started it; this activity must not stop it.
                Log.e(TAG, "start refused code=" + start
                        + " status=" + CerulionRuntime.status());
            }
        } catch (Exception error) {
            Log.e(TAG, "configuration failed", error);
        }
    }

    @Override
    protected void onDestroy() {
        handler.removeCallbacks(startRetry);
        handler.removeCallbacks(stopRetry);
        // Clear a stale stop-control marker so the next instance is not
        // stopped by a file this activity left behind.
        new File(getFilesDir(), "stop").delete();
        if (ownsRuntime) {
            stopRuntime();
        }
        super.onDestroy();
    }

    private void stopRuntime() {
        if (stopRequested) {
            return;
        }
        int code = CerulionRuntime.stop();
        Log.i(TAG, "stop=" + code);
        if (code == CerulionRuntime.BUSY) {
            handler.removeCallbacks(stopRetry);
            handler.postDelayed(stopRetry, 100);
            return;
        }
        if (code == CerulionRuntime.OK || code == CerulionRuntime.BAD_STATE) {
            stopRequested = true;
            handler.removeCallbacks(heartbeat);
        } else {
            Log.e(TAG, "stop failed code=" + code
                    + " status=" + CerulionRuntime.status());
            handler.removeCallbacks(stopRetry);
            handler.postDelayed(stopRetry, 500);
        }
    }

    private String configJson() throws IOException, JSONException {
        JSONObject config = new JSONObject();
        // Loopback: `adb forward` reaches the device's own loopback, and the
        // gateway is unreachable from the device LAN.
        config.put("listen", new JSONArray().put("tcp/127.0.0.1:7683"));
        config.put("node_lib_dir", getApplicationInfo().nativeLibraryDir);
        // Timestamp + random suffix: a restart inside the same millisecond
        // cannot reopen the previous run's bag.
        config.put("record", new JSONObject().put("file",
                "app-" + System.currentTimeMillis() + "-"
                        + java.util.UUID.randomUUID().toString().substring(0, 8) + ".mcap"));
        config.put(
                "publish",
                new JSONArray().put(new JSONObject()
                        .put("topic", "/app/heartbeat")
                        .put("schema", "std_msgs/String")));
        File graph = new File(getFilesDir(), "graph.yaml");
        // An absent graph.yaml means a publisher-only run; a path that exists
        // but is not a regular file is a broken setup, so fail loudly instead
        // of silently running without the intended graph.
        if (graph.exists() && !graph.isFile()) {
            throw new IOException(graph + " exists but is not a regular file");
        }
        if (graph.isFile()) {
            config.put(
                    "graph_yaml",
                    new String(Files.readAllBytes(graph.toPath()), StandardCharsets.UTF_8));
        }
        return config.toString();
    }
}
