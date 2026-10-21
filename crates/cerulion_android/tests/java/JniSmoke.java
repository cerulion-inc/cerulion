// SPDX-License-Identifier: AGPL-3.0-only
import com.cerulion.android.CerulionRuntime;
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

    private static String waitForState(String expected) throws Exception {
        for (int attempt = 0; attempt < 500; attempt++) {
            String status = CerulionRuntime.status();
            if (expected.equals(state(status))) {
                return status;
            }
            Thread.sleep(10);
        }
        throw new AssertionError("timed out waiting for state " + expected);
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

    public static void main(String[] args) throws Exception {
        System.out.println("status-before-init=" + state(CerulionRuntime.status()));
        System.out.println("init-bad=" + CerulionRuntime.init(args[0] + "/missing", "jni-test"));
        System.out.println("init-ok=" + CerulionRuntime.init(args[0], "jni-test"));
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
    }
}
