package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import org.json.JSONArray;
import org.json.JSONObject;
import static org.firstinspires.ftc.teamcode.LimelightCellTargeting.*;
import static org.firstinspires.ftc.teamcode.LimelightCellTargetingTest.*;

/**
 * Monte Carlo effectiveness estimate: 20 two-minute matches, 30 ms frames. Every input below is
 * an assumption for comparing tuning choices, not a measured field condition.
 * Run with LimelightCellTargetingTest compiled on the classpath. Argument "handheld" makes the
 * camera pitch wander +/-13 degrees and roll +/-6 degrees. Mid-flip, either the departing or the
 * arriving side counts as correct.
 */
public final class LimelightCellTargetingSimulation {
    static final double FLIP_SECONDS = 0.6;          // Assumed HIVE flip duration.
    static final double MIN_FLIP_GAP = 8, MAX_FLIP_GAP = 28;
    static final double OCCLUSION = 0.1;             // Chance each tag is blocked in a frame.
    static final double SPEED = 30;                  // Robot speed, in./s.
    static final double CAMERA_PITCH = 30, CONFIGURED_PITCH = 25;

    public static void main(String[] args) throws Exception {
        boolean handheld = args.length > 0 && args[0].equals("handheld");
        long correct = 0, unknown = 0, wrong = 0, measured = 0;
        for (int match = 0; match < 20; match++) {
            Random random = new Random(100 + match);
            Scene s = new Scene();
            s.noise = 1;
            s.random = random;
            s.pitchDeg = CAMERA_PITCH;
            LimelightCellTargeting tracker = tracker(Alliance.RED, CONFIGURED_PITCH);
            double angle = random.nextBoolean() ? 30 : -30, target = angle;
            double nextFlip = 5 + random.nextDouble() * 15, flipStart = -1;
            double x = -40, y = -50, wx = x, wy = y, yawOffset = 0;
            for (int f = 0; f < 4000; f++) {
                double t = f * 0.03;
                if (flipStart < 0 && t >= nextFlip) { flipStart = t; target = -angle; }
                if (flipStart >= 0) {
                    double k = Math.min(1, (t - flipStart) / FLIP_SECONDS);
                    angle = -target + 2 * target * k;
                    if (k >= 1) {
                        flipStart = -1;
                        nextFlip = t + MIN_FLIP_GAP + random.nextDouble() * (MAX_FLIP_GAP - MIN_FLIP_GAP);
                    }
                }
                // Drive between random waypoints, usually facing the red HIVE, sometimes turned away.
                if (Math.hypot(wx - x, wy - y) < 3) {
                    wx = -66 + random.nextDouble() * 132;
                    wy = -66 + random.nextDouble() * 132;
                    if (Math.abs(wx + 12.75) < 15 && Math.abs(wy) < 25) wx += 30;
                    yawOffset = (random.nextDouble() - 0.5) * (random.nextDouble() < 0.3 ? 240 : 20);
                }
                double d = Math.hypot(wx - x, wy - y), step = Math.min(d, SPEED * 0.03);
                x += (wx - x) / d * step;
                y += (wy - y) / d * step;
                if (handheld) {
                    s.pitchDeg = CAMERA_PITCH + 10 * Math.sin(t * 0.7 + match) + 3 * Math.sin(t * 2.3);
                    s.rollDeg = 6 * Math.sin(t * 0.5 + 2 * match);
                }
                s.at(x, y, Math.toDegrees(Math.atan2(-12.75 - x, -y)) + yawOffset).red(angle);
                List<JSONObject> seen = new ArrayList<>();
                for (JSONObject tag : s.tags(RED)) {
                    if (random.nextDouble() < OCCLUSION || !inImage(tag)) continue;
                    seen.add(tag);
                }
                Observation o = tracker.evaluate(new Frame(f + 1, seen.toArray(new JSONObject[0])), (f + 1) * 30L);
                if (o.reason == Reason.MEASURED) measured++;
                // Mid-flip, either the departing or arriving side counts as correct.
                State truth = angle > 0 ? State.AUDIENCE_UP : State.SCORING_TABLE_UP;
                State arriving = target > 0 ? State.AUDIENCE_UP : State.SCORING_TABLE_UP;
                if (o.state == State.UNKNOWN) unknown++;
                else if (o.state == truth || (flipStart >= 0 && o.state == arriving)) correct++;
                else wrong++;
            }
        }
        long n = correct + unknown + wrong;
        System.out.printf("%s, frames %d: correct %.1f%%, unknown %.1f%%, wrong %.2f%% (measured in %.1f%%)%n",
                handheld ? "hand-held" : "fixed mount", n, 100.0 * correct / n, 100.0 * unknown / n,
                100.0 * wrong / n, 100.0 * measured / n);
    }

    static boolean inImage(JSONObject tag) {
        JSONArray points = tag.getJSONArray("pts");
        for (int c = 0; c < 4; c++) {
            double u = points.getJSONArray(c).getDouble(0), v = points.getJSONArray(c).getDouble(1);
            if (u < 0 || u > 2 * CX || v < 0 || v > 2 * CY) return false;
        }
        return true;
    }
}
