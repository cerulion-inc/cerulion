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
    private boolean stopRequested;
    private long tick;

    private final Runnable heartbeat = new Runnable() {
        @Override
        public void run() {
            if (stopRequested) {
                return;
            }
            int code = CerulionRuntime.publish(
                    "/app/heartbeat", "{\"data\":\"tick " + tick++ + "\"}");
            if (tick % 4 == 0) {
                Log.i(TAG, "publish=" + code + " status=" + CerulionRuntime.status());
            }
            handler.postDelayed(this, 500);
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
        try {
            int start = CerulionRuntime.start(configJson());
            Log.i(TAG, "start=" + start);
            handler.postDelayed(heartbeat, 500);
        } catch (Exception error) {
            Log.e(TAG, "configuration failed", error);
        }
    }

    @Override
    protected void onNewIntent(android.content.Intent intent) {
        super.onNewIntent(intent);
        setIntent(intent);
        if (intent.getBooleanExtra("stop", false)) {
            stopRuntime();
        }
    }

    @Override
    protected void onDestroy() {
        stopRuntime();
        super.onDestroy();
    }

    private void stopRuntime() {
        if (stopRequested) {
            return;
        }
        stopRequested = true;
        handler.removeCallbacks(heartbeat);
        Log.i(TAG, "stop=" + CerulionRuntime.stop());
    }

    private String configJson() throws IOException, JSONException {
        JSONObject config = new JSONObject();
        config.put("listen", new JSONArray().put("tcp/0.0.0.0:7683"));
        config.put("node_lib_dir", getApplicationInfo().nativeLibraryDir);
        config.put("record", new JSONObject().put("file", "app.mcap"));
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
