// SPDX-License-Identifier: AGPL-3.0-only
import com.cerulion.android.CerulionRuntime;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.function.IntSupplier;

public final class JniSmoke {
    private static String state(String status) {
        String marker = "\"state\":\"";
        int start = status.indexOf(marker);
        if (start < 0) {
            return "missing";
        }
        start += marker.length();
        int end = status.indexOf('"', start);
        return status.substring(start, end);
    }

    private static int errorCode(String status) {
        String marker = "\"last_error\":{\"code\":";
        int start = status.indexOf(marker);
        if (start < 0) {
            return -1;
        }
        start += marker.length();
        int end = status.indexOf(',', start);
        return Integer.parseInt(status.substring(start, end));
    }

    private static String field(String status, String name) {
        String marker = "\"" + name + "\":";
        int start = status.indexOf(marker);
        if (start < 0) {
            return "missing";
        }
        start += marker.length();
        int end = status.charAt(start) == '['
                ? status.indexOf(']', start) + 1
                : status.indexOf(',', start);
        return status.substring(start, end);
    }

    // A JSON string literal for the graph text: quotes, backslashes and
    // control characters escaped.
    private static String jsonString(String text) {
        StringBuilder out = new StringBuilder("\"");
        for (char c : text.toCharArray()) {
            switch (c) {
                case '"': out.append("\\\""); break;
                case '\\': out.append("\\\\"); break;
                case '\n': out.append("\\n"); break;
                case '\r': out.append("\\r"); break;
                case '\t': out.append("\\t"); break;
                default:
                    if (c < 0x20) {
                        out.append(String.format("\\u%04x", (int) c));
                    } else {
                        out.append(c);
                    }
            }
        }
        return out.append('"').toString();
    }

    private static String waitForState(String expected) throws Exception {
        String status = null;
        for (int attempt = 0; attempt < 500; attempt++) {
            status = CerulionRuntime.status();
            if (expected.equals(state(status))) {
                return status;
            }
            Thread.sleep(10);
        }
        throw new AssertionError("timed out waiting for state " + expected + ": " + status);
    }

    // The error code a call returns while a lifecycle transition is in
    // flight. The status can report the new state before the host's start
    // or stop worker finishes, so a call made right after a state change
    // retries until the transition settles.
    private static final int BUSY = 11;

    private static int whenSettled(IntSupplier call) throws Exception {
        for (int attempt = 0; attempt < 500; attempt++) {
            int code = call.getAsInt();
            if (code != BUSY) {
                return code;
            }
            Thread.sleep(10);
        }
        throw new AssertionError("timed out waiting for the host lifecycle to settle");
    }

    private static String illegalState(Runnable call) {
        try {
            call.run();
            return "no-exception";
        } catch (IllegalStateException error) {
            return error.getMessage();
        }
    }

    public static void main(String[] args) throws Exception {
        System.out.println("status-before-load="
                + illegalState(() -> CerulionRuntime.status()));
        CerulionRuntime.loadLibrary(args[1]);
        System.out.println("load-again=" + illegalState(() -> CerulionRuntime.loadLibrary(args[1])));
        System.out.println("load-other="
                + illegalState(() -> CerulionRuntime.loadLibrary("other_runtime")));
        System.out.println("status-before-init=" + state(CerulionRuntime.status()));
        System.out.println("init-bad=" + CerulionRuntime.init(args[0] + "/missing", "jni-test"));
        System.out.println("init-ok=" + CerulionRuntime.init(args[0], "jni-test"));
        String initStatus = CerulionRuntime.status();
        System.out.println("linked-nodes=" + field(initStatus, "linked_nodes"));
        System.out.println("publish-before-start="
                + CerulionRuntime.publish("/jni/heartbeat", "{\"data\":\"before\"}"));

        int invalidStart = CerulionRuntime.start("{");
        String invalidStatus = waitForState("failed");
        System.out.println("start-invalid=" + invalidStart
                + " state=" + state(invalidStatus)
                + " error=" + errorCode(invalidStatus));

        String config = "{\"publish\":[{\"topic\":\"/jni/heartbeat\","
                + "\"schema\":\"std_msgs/String\"}]}";
        int validStart = whenSettled(() -> CerulionRuntime.start(config));
        String runningStatus = waitForState("running");
        System.out.println("start-ok=" + validStart + " state=" + state(runningStatus));
        System.out.println("publish=" + whenSettled(() -> CerulionRuntime.publish(
                "/jni/heartbeat", "{\"data\":\"hello\"}")));
        System.out.println("publish-unknown=" + whenSettled(() -> CerulionRuntime.publish(
                "/jni/missing", "{\"data\":\"hello\"}")));

        int stop = whenSettled(CerulionRuntime::stop);
        String stoppedStatus = waitForState("stopped");
        System.out.println("stop=" + stop + " state=" + state(stoppedStatus));

        // The sample app's graph: every node type in it is linked into the
        // loaded library, so the start reaches running with all three nodes.
        String graph = new String(Files.readAllBytes(Paths.get(args[2])), StandardCharsets.UTF_8);
        String graphConfig = "{\"graph_yaml\":" + jsonString(graph) + "}";
        int graphStart = whenSettled(() -> CerulionRuntime.start(graphConfig));
        String graphStatus = waitForState("running");
        System.out.println("start-graph=" + graphStart + " state=" + state(graphStatus)
                + " nodes=" + field(graphStatus, "nodes"));
        int graphStop = whenSettled(CerulionRuntime::stop);
        String graphStopped = waitForState("stopped");
        System.out.println("stop-graph=" + graphStop + " state=" + state(graphStopped));
    }
}
