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
    private boolean stopRequested;
    private boolean ownsRuntime;
    private long tick;

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
            try {
                String state = new JSONObject(status).getString("state");
                if ("starting".equals(state)) {
                    handler.postDelayed(this, 500);
                    return;
                }
                if (!"running".equals(state)) {
                    Log.e(TAG, "heartbeat stopped status=" + status);
                    return;
                }
                int code = CerulionRuntime.publish(
                        "/app/heartbeat", "{\"data\":\"tick " + tick++ + "\"}");
                if (tick % 4 == 0) {
                    Log.i(TAG, "publish=" + code + " status=" + status);
                }
                handler.postDelayed(this, 500);
            } catch (JSONException error) {
                Log.e(TAG, "heartbeat stopped status=" + status, error);
            }
        }
    };

    @Override
    protected void onCreate(Bundle state) {
        super.onCreate(state);
        int init = CerulionRuntime.init(getFilesDir().getAbsolutePath(), "android-app");
        if (init != CerulionRuntime.OK && init != CerulionRuntime.ALREADY_INITIALIZED) {
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
            handler.postDelayed(this::stopRuntime, 100);
            return;
        }
        stopRequested = true;
        handler.removeCallbacks(heartbeat);
    }

    private String configJson() throws IOException, JSONException {
        JSONObject config = new JSONObject();
        config.put("listen", new JSONArray().put("tcp/0.0.0.0:7683"));
        config.put("node_lib_dir", getApplicationInfo().nativeLibraryDir);
        config.put("record", new JSONObject().put("file",
                "app-" + System.currentTimeMillis() + ".mcap"));
        config.put(
                "publish",
                new JSONArray().put(new JSONObject()
                        .put("topic", "/app/heartbeat")
                        .put("schema", "std_msgs/String")));
        File graph = new File(getFilesDir(), "graph.yaml");
        if (graph.isFile()) {
            config.put(
                    "graph_yaml",
                    new String(Files.readAllBytes(graph.toPath()), StandardCharsets.UTF_8));
        }
        return config.toString();
    }
}
