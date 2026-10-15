// SPDX-License-Identifier: AGPL-3.0-only
package com.cerulion.sample;

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.content.pm.ApplicationInfo;
import android.net.wifi.WifiManager;
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
    private WifiManager.MulticastLock multicastLock;
    private final Runnable startRetry = this::startRuntime;
    private boolean stopRequested;
    private boolean ownsRuntime;
    private boolean restartRequested;
    private boolean destroyed;
    private long tick;

    private final Runnable heartbeat = new Runnable() {
        @Override
        public void run() {
            if (destroyed || stopRequested) {
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
                    // start() returns before the runtime is up; a start that
                    // fails later reports "failed" here. Stop every component
                    // before releasing the multicast lock so no surviving
                    // sibling can outlive discovery.
                    if ("failed".equals(state)) {
                        stopRuntime();
                    }
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

    @Override
    protected void onNewIntent(Intent intent) {
        super.onNewIntent(intent);
        if (destroyed) {
            return;
        }
        setIntent(intent);
        if (ownsRuntime && !stopRequested) {
            if (!isFailed(CerulionRuntime.status())) {
                Log.i(TAG, "runtime is already running");
                return;
            }
            restartRequested = true;
            handler.removeCallbacks(heartbeat);
            stopRuntime();
            return;
        }
        handler.removeCallbacks(startRetry);
        stopRequested = false;
        ownsRuntime = false;
        startRuntime();
    }

    private void startRuntime() {
        if (destroyed) {
            return;
        }
        try {
            String config = configJson();
            acquireMulticastLock();
            int start = CerulionRuntime.start(config);
            Log.i(TAG, "start=" + start);
            if (start == CerulionRuntime.OK) {
                ownsRuntime = true;
                handler.postDelayed(heartbeat, 500);
            } else if (start == CerulionRuntime.BUSY) {
                // Another start or stop is still in flight; retry unless
                // this activity is already going away. The retry takes the
                // multicast lock again, so release this one first.
                releaseMulticastLock();
                if (!isFinishing() && !isDestroyed()) {
                    handler.postDelayed(startRetry, 100);
                }
            } else {
                // BAD_STATE means a runtime is already running, owned by
                // whoever started it; this activity must not stop it.
                Log.e(TAG, "start refused code=" + start
                        + " status=" + CerulionRuntime.status());
                releaseMulticastLock();
            }
        } catch (Exception error) {
            releaseMulticastLock();
            Log.e(TAG, "configuration failed", error);
        }
    }

    @Override
    protected void onDestroy() {
        destroyed = true;
        restartRequested = false;
        handler.removeCallbacks(startRetry);
        handler.removeCallbacks(heartbeat);
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
        if (code == CerulionRuntime.OK || code == CerulionRuntime.BAD_STATE) {
            stopRequested = true;
            handler.removeCallbacks(heartbeat);
            releaseMulticastLock();
            if (restartRequested && !destroyed) {
                restartRequested = false;
                stopRequested = false;
                ownsRuntime = false;
                startRuntime();
            }
            return;
        }
        Log.e(TAG, "stop failed code=" + code
                + " status=" + CerulionRuntime.status());
        handler.postDelayed(this::stopRuntime, 1000);
    }

    // Android drops inbound multicast to save power unless an app holds this lock;
    // without it desks cannot see the mDNS beacon or the zenoh scouting replies.
    private void acquireMulticastLock() {
        releaseMulticastLock();
        WifiManager wifi = (WifiManager) getApplicationContext()
                .getSystemService(Context.WIFI_SERVICE);
        if (wifi == null) {
            Log.w(TAG, "no Wi-Fi service: discovery is limited to typed addresses");
            return;
        }
        multicastLock = wifi.createMulticastLock("cerulion-discovery");
        multicastLock.setReferenceCounted(false);
        multicastLock.acquire();
        Log.i(TAG, "multicast lock held=" + multicastLock.isHeld());
    }

    private void releaseMulticastLock() {
        if (multicastLock != null && multicastLock.isHeld()) {
            multicastLock.release();
            Log.i(TAG, "multicast lock released");
        }
        multicastLock = null;
    }

    private static boolean isFailed(String status) {
        try {
            return "failed".equals(new JSONObject(status).optString("state"));
        } catch (JSONException error) {
            return false;
        }
    }

    private String configJson() throws IOException, JSONException {
        JSONObject config = new JSONObject();
        config.put("listen", new JSONArray().put("tcp/0.0.0.0:7683"));
        // The LAN-access development opt-out is honored only by debuggable builds.
        boolean debuggable =
                (getApplicationInfo().flags & ApplicationInfo.FLAG_DEBUGGABLE) != 0;
        if (debuggable) {
            String lanAccess = getIntent().getStringExtra("lan_access");
            if (lanAccess != null) {
                config.put("lan_access", lanAccess);
            }
        } else if (getIntent().hasExtra("lan_access")) {
            Log.w(TAG, "ignoring lan_access extra in a release build; LAN access remains paired");
        }
        config.put("node_lib_dir", getApplicationInfo().nativeLibraryDir);
        config.put("record", new JSONObject().put("file",
                recordingFileName()));
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

    private String recordingFileName() {
        File bagsDir = new File(getFilesDir(), "bags");
        String stem = "app-" + System.currentTimeMillis();
        String name = stem + ".mcap";
        for (int suffix = 1; new File(bagsDir, name).exists(); suffix++) {
            name = stem + "-" + suffix + ".mcap";
        }
        return name;
    }
}
