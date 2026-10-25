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

    private static final String EXP = "CerExp";

    private void execProbe(String path) {
        try {
            Process p = new ProcessBuilder(path).redirectErrorStream(true).start();
            java.io.ByteArrayOutputStream buf = new java.io.ByteArrayOutputStream();
            byte[] chunk = new byte[4096];
            int n;
            while ((n = p.getInputStream().read(chunk)) > 0) {
                buf.write(chunk, 0, n);
            }
            String out = new String(buf.toByteArray(), StandardCharsets.UTF_8).trim();
            int code = p.waitFor();
            Log.i(EXP, "E3 exec " + path + " exit=" + code + " out=" + out);
        } catch (Exception error) {
            Log.e(EXP, "E3 exec " + path + " failed: " + error);
        }
    }

    private void runExperiments() {
        String libDir = getApplicationInfo().nativeLibraryDir;
        Log.i(EXP, "nativeLibraryDir=" + libDir + " targetSdk=" + getApplicationInfo().targetSdkVersion);
        execProbe(libDir + "/libexecprobe.so");
        try {
            File copy = new File(getFilesDir(), "execprobe");
            Files.copy(new File(libDir, "libexecprobe.so").toPath(), copy.toPath(),
                    java.nio.file.StandardCopyOption.REPLACE_EXISTING);
            Log.i(EXP, "E3 setExecutable=" + copy.setExecutable(true, true));
            execProbe(copy.getAbsolutePath());
        } catch (Exception error) {
            Log.e(EXP, "E3 copy failed: " + error);
        }
        try {
            String yaml = "prefix: exp\nnodes:\n  - id: ticker\n    type: test_node_macro_period_cdylib\n"
                    + "    outputs:\n      - name: cmd\n        schema: geometry_msgs/Vector3\n";
            Files.write(new File(getFilesDir(), "graph.yaml").toPath(), yaml.getBytes(StandardCharsets.UTF_8));
        } catch (IOException error) {
            Log.e(EXP, "graph.yaml write failed: " + error);
        }
        handler.postDelayed(this::expStop, 8000);
    }

    private void expStop() {
        Log.i(EXP, "E4 status before stop=" + CerulionRuntime.status());
        stopRuntime();
        handler.postDelayed(this::expPollStopped, 200);
    }

    private void expPollStopped() {
        String status = CerulionRuntime.status();
        try {
            String state = new JSONObject(status).getString("state");
            if ("stopping".equals(state)) {
                handler.postDelayed(this::expPollStopped, 200);
                return;
            }
        } catch (JSONException ignored) {
        }
        Log.i(EXP, "E4 status after stop=" + status);
    }

    @Override
    protected void onCreate(Bundle state) {
        super.onCreate(state);
        runExperiments();
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
        } else {
            Log.e(TAG, "stop failed code=" + code
                    + " status=" + CerulionRuntime.status());
            handler.postDelayed(this::stopRuntime, 500);
        }
    }

    private String configJson() throws IOException, JSONException {
        JSONObject config = new JSONObject();
        config.put("listen", new JSONArray().put("tcp/0.0.0.0:7684"));
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
