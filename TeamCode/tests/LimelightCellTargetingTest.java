package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.json.JSONArray;
import org.json.JSONObject;
import java.net.InetAddress;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import static org.firstinspires.ftc.teamcode.LimelightCellTargeting.*;

/**
 * Real SDK parser, synthetic corners projected from a 3D HIVE model (game manual 9.6/9.9).
 * World frame in inches: x toward the blue side, y away from the audience, z up.
 * Hive angle: audience cell's outer-edge elevation, +30 = audience up, -30 = scoring-table up.
 */
public final class LimelightCellTargetingTest {
    private static int checks;
    static final double F = 600, CX = 320, CY = 240;
    // Assumed detector limits for the synthetic scene (not measured).
    static final double MAX_VIEW_DEGREES = 70, MAX_RANGE = 150;

    static class Frame extends LLResult {
        long stale;
        Frame(double ts, JSONObject... tags) throws Exception {
            super(new JSONObject().put("v", 1).put("ts", ts).put("Fiducial", new JSONArray(tags)));
        }
        @Override public long getStaleness() { return stale; }
        @Override public double getParseLatency() { return 0; }
    }

    static final class Scene {
        double x = -12.75, y = -60, z = 10, yawDeg = 0, pitchDeg = 35, rollDeg = 0;
        double redAngle = 30, blueAngle = 30, noise = 0;
        Random random = new Random(1);
        Scene at(double x, double y, double yawDeg) { this.x = x; this.y = y; this.yawDeg = yawDeg; return this; }
        Scene red(double angle) { redAngle = angle; return this; }
        Scene blue(double angle) { blueAngle = angle; return this; }

        double[] project(double[] w) {
            double yaw = Math.toRadians(yawDeg), pitch = Math.toRadians(pitchDeg);
            double[] fwd = {Math.sin(yaw) * Math.cos(pitch), Math.cos(yaw) * Math.cos(pitch), Math.sin(pitch)};
            double[] level = {Math.cos(yaw), -Math.sin(yaw), 0}, levelDown = cross(fwd, level);
            double roll = Math.toRadians(rollDeg); // Positive: camera's right side lower.
            double[] right = new double[3];
            for (int i = 0; i < 3; i++) right[i] = level[i] * Math.cos(roll) + levelDown[i] * Math.sin(roll);
            double[] down = cross(fwd, right);
            double[] d = {w[0] - x, w[1] - y, w[2] - z};
            double zc = dot(d, fwd);
            if (zc <= 0) return null;
            return new double[]{CX + F * dot(d, right) / zc, CY + F * dot(d, down) / zc};
        }

        /** Detected tag, or null if its printed face is out of the assumed detection envelope. */
        JSONObject tag(int id) {
            int first = id < 34 ? 30 : id < 38 ? 34 : id < 42 ? 38 : 42;
            boolean audience = first == 34 || first == 38;
            double hive = first < 38 ? redAngle : blueAngle;
            double outward = audience ? -1 : 1;
            double t = Math.toRadians(audience ? hive : -hive); // this cell's outer-edge elevation
            // Tag frame (manual 9.9): faces the TILES, top edge toward the cell's outer edge.
            double[] yAxis = {0, outward * Math.cos(t), Math.sin(t)};
            double[] xAxis = {-outward, 0, 0};
            // Cells ride an ~18 in. arm from the pivot (43.95 in. above the TILES).
            double[] origin = {first < 38 ? -12.75 : 12.75, outward * 16 * Math.cos(t), 43 + 18 * Math.sin(t)};
            double[] toCamera = {x - origin[0], y - origin[1], z - origin[2]};
            double range = Math.sqrt(dot(toCamera, toCamera));
            if (dot(cross(xAxis, yAxis), toCamera) < Math.cos(Math.toRadians(MAX_VIEW_DEGREES)) * range
                    || range > MAX_RANGE) return null;
            double memberX = new double[]{-6.5, -2.75, 2.75, 6.5}[id - first];
            JSONArray points = new JSONArray();
            for (double[] c : new double[][]{{-1, -1}, {1, -1}, {1, 1}, {-1, 1}}) {
                double[] p = project(onTag(origin, xAxis, yAxis, memberX + c[0] * 1.625, c[1] * 1.625));
                if (p == null) return null;
                points.put(new JSONArray().put(p[0] + noise * (random.nextDouble() * 2 - 1))
                        .put(p[1] + noise * (random.nextDouble() * 2 - 1)));
            }
            double[] center = project(onTag(origin, xAxis, yAxis, memberX, 0));
            return new JSONObject().put("fID", id).put("pts", points)
                    .put("tx_nocross", Math.toDegrees(Math.atan((center[0] - CX) / F)))
                    .put("ty_nocross", Math.toDegrees(Math.atan(-(center[1] - CY) / F)));
        }

        JSONObject[] tags(int... ids) {
            List<JSONObject> tags = new ArrayList<>();
            for (int id : ids) {
                JSONObject tag = tag(id);
                if (tag != null) tags.add(tag);
            }
            return tags.toArray(new JSONObject[0]);
        }
    }

    static double[] onTag(double[] o, double[] xAxis, double[] yAxis, double px, double py) {
        return new double[]{o[0] + xAxis[0] * px + yAxis[0] * py,
                o[1] + xAxis[1] * px + yAxis[1] * py, o[2] + xAxis[2] * px + yAxis[2] * py};
    }
    static double dot(double[] a, double[] b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
    static double[] cross(double[] a, double[] b) {
        return new double[]{a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    }

    static LimelightCellTargeting tracker(Alliance alliance, double pitch) throws Exception {
        return new LimelightCellTargeting(new Limelight3A(null, "test", InetAddress.getLoopbackAddress()),
                alliance, pitch);
    }
    static LimelightCellTargeting tracker(Alliance alliance) throws Exception { return tracker(alliance, 35); }

    /** Feeds frames 30 ms apart; owns the frame clock so tests read as sequences. */
    static final class Run {
        final LimelightCellTargeting tracker;
        long now;
        int ts;
        Observation o;
        Run(LimelightCellTargeting tracker) { this.tracker = tracker; }
        Observation frame(JSONObject... tags) throws Exception {
            now += 30;
            return o = tracker.evaluate(new Frame(++ts, tags), now);
        }
        Observation frames(int n, Scene s, int... ids) throws Exception {
            for (int i = 0; i < n; i++) frame(s.tags(ids));
            return o;
        }
        /** Moves the red HIVE from one angle to another over the given time, one frame per 30 ms. */
        Observation flipRed(Scene s, double from, double to, long millis, int... ids) throws Exception {
            for (long t = 0; t <= millis; t += 30) frame(s.red(from + (to - from) * t / millis).tags(ids));
            return o;
        }
    }

    static void expect(Observation o, State s, String label) {
        if (o.state != s) {
            throw new AssertionError(String.format("%s: %s/%s P=%.3f angle %.1f +/- %.1f", label, o.state,
                    o.reason, o.probabilityAudienceUp, o.hiveAngleDegrees, o.hiveAngleSigmaDegrees));
        }
        checks++;
    }
    static void expectReason(Observation o, Reason r, String label) {
        if (o.reason != r) throw new AssertionError(label + ": " + o.reason);
        checks++;
    }
    static void expectNear(double actual, double expected, double tolerance, String label) {
        if (!(Math.abs(actual - expected) <= tolerance)) throw new AssertionError(label + ": " + actual);
        checks++;
    }

    static final int[] RED = {30, 31, 32, 33, 34, 35, 36, 37};

    public static void main(String[] args) throws Exception {
        // Every view that can see the HIVE in a given position converges to it.
        double[][] views = {{-12.75, -60, 0}, {12.75, -40, 0}, {-12.75, 70, 180}, {-50, -40, 40},
                {40, 60, 220}, {-12.75, -30, 0}};
        int hidden = 0, framesToConfirm = 0, confirmed = 0;
        Run run;
        Scene s;
        for (double noise : new double[]{0, 1}) {
            for (double[] view : views) {
                for (double angle : new double[]{30, -30}) {
                    State want = angle > 0 ? State.AUDIENCE_UP : State.SCORING_TABLE_UP;
                    for (int[] ids : new int[][]{RED, {34, 35}, {32, 33}}) {
                        s = new Scene().at(view[0], view[1], view[2]).red(angle);
                        s.noise = noise;
                        if (s.tags(ids).length < 2) { hidden++; continue; }
                        run = new Run(tracker(Alliance.RED));
                        int n = 0;
                        while (n < 20 && run.frame(s.tags(ids)).state != want) n++;
                        expect(run.frames(10, s, ids), want, String.format("view %.0f,%.0f,%.0f angle %.0f noise %.0f",
                                view[0], view[1], view[2], angle, noise));
                        framesToConfirm += n + 1;
                        confirmed++;
                    }
                }
            }
        }

        if (confirmed < 40 || hidden > 40) {
            throw new AssertionError("convergence coverage: " + confirmed + " checked, " + hidden + " hidden");
        }

        // Moving gate holds across re-polls of the same frame and right after tags drop out.
        run = new Run(tracker(Alliance.RED));
        s = new Scene().at(-12.75, -45, 0);
        run.frames(10, s.red(30), RED);
        run.frame(s.red(8).tags(RED));
        run.frame(s.red(6).tags(RED));
        expect(run.o, State.UNKNOWN, "moving gate");
        expect(run.tracker.evaluate(new Frame(run.ts, s.tags(RED)), run.now + 10), State.UNKNOWN, "moving gate re-poll");
        expect(run.frame(), State.UNKNOWN, "moving gate without tags");
        run.frames(10, s.red(30), RED);
        expect(run.o, State.AUDIENCE_UP, "moving gate clears when settled");

        // A level HIVE (or one held mid-tilt) never reaches either side.
        run = new Run(tracker(Alliance.RED));
        s = new Scene().at(-12.75, -30, 0).red(0);
        expect(run.frames(30, s, RED), State.UNKNOWN, "level HIVE");

        // Hold: tags out of view keep the estimate for a while, then it relaxes to UNKNOWN.
        run = new Run(tracker(Alliance.RED));
        s = new Scene().red(30);
        expect(run.frames(10, s, RED), State.AUDIENCE_UP, "settled");
        for (int i = 0; i < 13; i++) run.frame();
        expect(run.o, State.AUDIENCE_UP, "held 0.4 s without tags");
        expectReason(run.o, Reason.NO_TAGS, "held reason");
        for (int i = 0; i < 20; i++) run.frame();
        expect(run.o, State.UNKNOWN, "relaxed after 1 s without tags");

        // Bumped but not flipped: dips to +12 and returns; never leaves AUDIENCE_UP.
        run = new Run(tracker(Alliance.RED));
        s = new Scene().at(-12.75, -45, 0);
        run.frames(10, s.red(30), RED);
        run.flipRed(s, 30, 12, 150, RED);
        expect(run.o, State.AUDIENCE_UP, "bump down");
        run.flipRed(s, 12, 30, 150, RED);
        expect(run.frames(5, s, RED), State.AUDIENCE_UP, "bump settled");

        // Flip watched from nearby: the camera sees the HIVE cross level before tags vanish.
        run = new Run(tracker(Alliance.RED));
        s = new Scene().at(-12.75, -45, 0);
        run.frames(10, s.red(30), RED);
        run.flipRed(s, 30, -30, 600, RED);
        run.frames(10, s, RED);
        expect(run.o, State.SCORING_TABLE_UP, "near flip");

        // Flip watched from far away: tags vanish before level. Must not keep AUDIENCE_UP.
        run = new Run(tracker(Alliance.RED, 15));
        s = new Scene().at(-12.75, -130, 0);
        s.pitchDeg = 15;
        run.frames(10, s.red(30), RED);
        expect(run.o, State.AUDIENCE_UP, "far settled");
        run.flipRed(s, 30, -30, 600, RED);
        run.frames(10, s, RED);
        expect(run.o, State.UNKNOWN, "far flip suspected");
        expectReason(run.o, Reason.FLIP_SUSPECTED, "far flip reason");

        // Turning away from a settled HIVE is not a suspected flip.
        run = new Run(tracker(Alliance.RED));
        run.frames(10, new Scene().red(30), RED);
        for (int i = 0; i < 10; i++) run.frame();
        expect(run.o, State.AUDIENCE_UP, "looked away");

        // Duplicate frames add no evidence.
        LimelightCellTargeting dup = tracker(Alliance.RED);
        Frame one = new Frame(1, new Scene().tags(34, 35));
        Observation firstObservation = dup.evaluate(one, 0);
        expectReason(firstObservation, Reason.MEASURED, "duplicate setup measured");
        double first = firstObservation.probabilityAudienceUp;
        if (!(first > 0.6)) throw new AssertionError("duplicate setup P " + first);
        for (int i = 1; i <= 5; i++) dup.evaluate(one, i * 10);
        expectNear(dup.evaluate(one, 60).probabilityAudienceUp, first, 0.01, "duplicate frames");
        // Stale or disconnected camera: no evidence.
        Frame old = new Frame(2, new Scene().tags(34, 35));
        old.stale = 500;
        expectReason(dup.evaluate(old, 70), Reason.STALE, "stale");
        expectReason(dup.evaluate(null, 80), Reason.STALE, "disconnected");
        dup.reset();
        expectNear(dup.evaluate(null, 90).probabilityAudienceUp, 0.5, 1e-9, "reset");

        // Mount self-calibration: start 15 degrees off and drive past the HIVE at varied headings.
        LimelightCellTargeting cal = tracker(Alliance.RED, 20);
        run = new Run(cal);
        double[][] drive = {{-12.75, -45, 0}, {-40, -40, 35}, {15, -45, -30}, {-50, -25, 60}, {-12.75, -30, 0}};
        for (double[] v : drive) {
            s = new Scene().at(v[0], v[1], v[2]).red(30);
            s.noise = 1;
            run.frames(40, s, RED);
        }
        expectNear(cal.mount().pitchDegrees(), 35, 2, "self-calibrated pitch");
        expectNear(cal.mount().rollDegrees(), 0, 2, "self-calibrated roll");
        expect(run.o, State.AUDIENCE_UP, "state after calibration");
        cal.mount().reset();
        expectNear(cal.mount().pitchDegrees(), 20, 1e-9, "mount reset");

        // Corner list order does not matter.
        JSONObject[] arbitrary = new Scene().tags(34, 35);
        for (JSONObject t : arbitrary) {
            JSONArray c = t.getJSONArray("pts");
            t.put("pts", new JSONArray().put(c.get(2)).put(c.get(0)).put(c.get(3)).put(c.get(1)));
        }
        run = new Run(tracker(Alliance.RED));
        for (int i = 0; i < 10; i++) run.frame(arbitrary);
        expect(run.o, State.AUDIENCE_UP, "arbitrary corner order");

        // Missing tag angles: focal length cannot be fitted.
        JSONObject[] noAngles = new Scene().tags(34, 35);
        for (JSONObject t : noAngles) { t.remove("tx_nocross"); t.remove("ty_nocross"); }
        expectReason(new Run(tracker(Alliance.RED)).frame(noAngles), Reason.UNRELIABLE, "no angles");

        // Each alliance reads only its own HIVE from a shared frame, and both feed one mount.
        CameraMount sharedMount = new CameraMount(35, 0);
        Limelight3A camera = new Limelight3A(null, "test", InetAddress.getLoopbackAddress());
        LimelightCellTargeting red = new LimelightCellTargeting(camera, Alliance.RED, sharedMount);
        LimelightCellTargeting blue = new LimelightCellTargeting(camera, Alliance.BLUE, sharedMount);
        s = new Scene().at(0, -40, 0).red(30).blue(30);
        Observation redState = null, blueState = null;
        for (int i = 1; i <= 15; i++) {
            Frame shared = new Frame(i, s.tags(34, 35, 36, 37, 30, 31, 38, 39, 40, 41, 42, 43));
            redState = red.evaluate(shared, i * 30);
            blueState = blue.evaluate(shared, i * 30);
        }
        expect(redState, State.AUDIENCE_UP, "shared view red");
        expect(blueState, State.AUDIENCE_UP, "shared view blue");
        // 15 frames x 2 red cells + 15 frames x 2 blue cells of pivot constraints, plus settled ones.
        if (sharedMount.samples() < 60) throw new AssertionError("shared mount samples " + sharedMount.samples());
        checks++;
        expectReason(red.evaluate(new Frame(16, s.tags(38, 39)), 16 * 30), Reason.NO_TAGS, "blue tags ignored by red");

        System.out.printf("PASS: %d checks (%d hidden views; mean %.1f frames to reach 95%%)%n",
                checks, hidden, (double) framesToConfirm / confirmed);
    }
}
