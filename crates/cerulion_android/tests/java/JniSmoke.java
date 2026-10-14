// SPDX-License-Identifier: AGPL-3.0-only
import com.cerulion.android.CerulionRuntime;

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
        int validStart = CerulionRuntime.start(config);
        String runningStatus = waitForState("running");
        System.out.println("start-ok=" + validStart + " state=" + state(runningStatus));
        System.out.println("publish=" + CerulionRuntime.publish(
                "/jni/heartbeat", "{\"data\":\"hello\"}"));
        System.out.println("publish-unknown=" + CerulionRuntime.publish(
                "/jni/missing", "{\"data\":\"hello\"}"));

        int stop = CerulionRuntime.stop();
        String stoppedStatus = waitForState("stopped");
        System.out.println("stop=" + stop + " state=" + state(stoppedStatus));
    }
}
