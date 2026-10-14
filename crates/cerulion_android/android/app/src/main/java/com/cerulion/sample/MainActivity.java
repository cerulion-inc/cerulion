// SPDX-License-Identifier: AGPL-3.0-only
package com.cerulion.sample;

import android.app.Activity;
import android.content.Context;
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
    private boolean stopRequested;
    private long tick;

    private final Runnable heartbeat = new Runnable() {
        @Override
        public void run() {
            if (stopRequested) {
                return;
            }
            // App-private stop control: only the app (or run-as on a
            // debuggable build) can create files/stop.
            File stopFile = new File(getFilesDir(), "stop");
            if (stopFile.isFile() && stopFile.delete()) {
                stopRuntime();
                return;
            }
            int code = CerulionRuntime.publish(
                    "/app/heartbeat", "{\"data\":\"tick " + tick++ + "\"}");
            String status = CerulionRuntime.status();
            if (tick % 4 == 0) {
                Log.i(TAG, "publish=" + code + " status=" + status);
            }
            // start() returns before the runtime is up; a start that fails later
            // reports "failed" here. Stop every component before releasing the
            // multicast lock so no surviving sibling can outlive discovery.
            if (isFailed(status)) {
                stopRuntime();
                return;
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
            String config = configJson();
            acquireMulticastLock();
            int start = CerulionRuntime.start(config);
            Log.i(TAG, "start=" + start);
            if (start == CerulionRuntime.OK) {
                handler.postDelayed(heartbeat, 500);
            } else {
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
        stopRuntime();
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
        releaseMulticastLock();
    }

    // Android drops inbound multicast to save power unless an app holds this lock;
    // without it desks cannot see the mDNS beacon or the zenoh scouting replies.
    private void acquireMulticastLock() {
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
