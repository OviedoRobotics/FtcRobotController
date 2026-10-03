package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import java.util.HashSet;
import java.util.List;
import java.util.Objects;
import java.util.Set;

/**
 * Probabilistic HIVE-state estimate for one alliance. Odometry handles aiming/range.
 *
 * Game manual 9.6/9.9: each HIVE is a rigid, bistable seesaw tipped 30 degrees either way. The
 * AprilTag cluster sits on the bottom of each CELL, facing the TILES, with the tags' bottom edges
 * toward the field center. The "hive angle" is the audience cell's outer-edge elevation:
 * +30 = audience cell up, -30 = scoring-table cell up. Each frame jointly fits the visible
 * cluster corners, measures that angle against gravity, and updates P(audience up):
 *
 * - Settled readings near +/-30 are strong evidence; readings between are "moving" and only say
 *   which side of level the HIVE is on (it cannot cross level without completing a flip).
 *   Two consecutive readings within 15 degrees of level report UNKNOWN (the HIVE is in motion)
 *   until a settled reading arrives or the tags have been gone for 150 ms.
 * - Without tags the estimate is held and slowly relaxes toward 50/50 (unseen flips).
 * - Losing the tags while the HIVE was tilting away from its last settled side, before it
 *   crossed level, leans the estimate toward a flip.
 * - Camera pitch/roll are refined online in a {@link CameraMount}: the tag row (pivot axis) is
 *   always horizontal. Share one mount between estimators that use the same camera.
 *
 * Tuning constants marked "placeholder" are not yet measured. Single-threaded; caller owns
 * pipeline selection and Limelight start/stop. Call {@link CameraMount#reset()} after moving the camera.
 */
public final class LimelightCellTargeting {
    public enum Alliance { RED, BLUE }
    public enum Side { AUDIENCE, SCORING_TABLE }
    public enum State { AUDIENCE_UP, SCORING_TABLE_UP, UNKNOWN }
    /**
     * MEASURED: this frame's tags updated the estimate. NO_TAGS: fresh frame without two usable
     * tags of a cell; estimate held. FLIP_SUSPECTED: tags were lost mid-tilt. UNRELIABLE: tags
     * present but the fit was rejected. STALE: no fresh camera frame.
     */
    public enum Reason { MEASURED, NO_TAGS, FLIP_SUSPECTED, UNRELIABLE, STALE }

    /** Game manual Figure 9-10: each stable position is tipped 30 degrees. */
    public static final double SETTLED_DEGREES = 30;
    /** RMS corner reprojection error allowed for the joint cluster fit. */
    public static final double MAX_REPROJECTION_PIXELS = 4;
    // Placeholder: spread of settled readings beyond fit noise (hive play, field flatness).
    private static final double SETTLED_SIGMA_DEGREES = 3;
    // Placeholder: camera pitch/roll uncertainty added to every reading.
    private static final double MOUNT_SIGMA_DEGREES = 5;
    // Placeholder: chance that a frame catches the HIVE moving rather than settled.
    private static final double MOVING_PROBABILITY = 0.1;
    private static final double OUTLIER_PROBABILITY = 0.01;
    // Placeholder: a moving reading may cross level by this much and still return.
    private static final double BASIN_OVERLAP_DEGREES = 5;
    // Placeholder: unseen flips per second; sets how fast a held estimate relaxes. 0.1 holds a
    // confident estimate about 0.5 s (simulated tradeoff in LIMELIGHT_TARGETING.md).
    private static final double UNSEEN_FLIP_RATE = 0.1;
    // Consecutive frames see the same scene; count each as half an independent sample.
    private static final double FRAME_EVIDENCE_WEIGHT = 0.5;
    private static final double MAX_LOG_ODDS = Math.log(999);
    // Placeholder: losing tags within this angle of level suggests a flip in progress.
    private static final double FLIP_SUSPECT_DEGREES = 22;
    private static final long LOST_MILLIS = 150;
    // Two consecutive readings closer to level than this mean the HIVE is moving: report UNKNOWN.
    private static final double MOVING_GATE_DEGREES = 15;
    private static final double MAX_FIT_SIGMA_DEGREES = 20;
    // Mount fit: each axis constraint is good to about 2 degrees; the configured mount to about 15.
    private static final double MOUNT_CONSTRAINT_SIGMA = Math.sin(Math.toRadians(2));
    private static final double MOUNT_PRIOR_SIGMA = Math.toRadians(15);
    // SDK 12 getBioBuzzCluster(): member centers along the cluster x axis, 3.25 in. tags.
    private static final double[] MEMBER_X = {-6.5, -2.75, 2.75, 6.5};
    private static final double HALF_TAG = 3.25 / 2;
    // Tag centers need at least 0.5 degree RMS spread (1 degree for two tags) to fit the focal length.
    private static final double MIN_ANGLE_SPREAD = Math.pow(Math.tan(Math.toRadians(0.5)), 2);

    public static final class Observation {
        public final State state;
        public final Reason reason;
        public final double probabilityAudienceUp;
        /** This frame's hive angle (+30 audience up) and its 1-sigma; NaN unless MEASURED. */
        public final double hiveAngleDegrees, hiveAngleSigmaDegrees;
        /** This frame's outer-edge tilt per cell (+30 raised, -30 lowered); NaN if unmeasured. */
        public final double audienceTiltDegrees, scoringTableTiltDegrees;
        public final int supportingTags;
        public final double ageMillis;
        private Observation(State state, Reason reason, double probability, double[] measurement,
                            int tags, double age) {
            this.state = state; this.reason = reason; probabilityAudienceUp = probability;
            hiveAngleDegrees = measurement[0]; hiveAngleSigmaDegrees = measurement[1];
            audienceTiltDegrees = measurement[2]; scoringTableTiltDegrees = measurement[3];
            supportingTags = tags; ageMillis = age;
        }
        /** Vision gate only; caller must additionally check odometry aim and shooter readiness. */
        public boolean isUp(Side side) {
            Objects.requireNonNull(side, "side");
            return side == Side.AUDIENCE ? state == State.AUDIENCE_UP
                    : state == State.SCORING_TABLE_UP;
        }
    }

    /**
     * Camera pitch/roll, starting from configured values and refined from every fitted cell.
     * One camera, one mount: pass the same instance to each estimator it feeds. Each estimator
     * only adds its own alliance's cells, so a red and a blue estimator never double count.
     */
    public static final class CameraMount {
        private final double priorPitch, priorRoll; // Radians, as configured.
        private double pitch, roll;                 // Radians, refined online.
        private double[] up;                        // World up in camera coordinates.
        private double[][] normal = new double[3][3];
        private double[] target = new double[3];
        private int samples;

        /**
         * pitchDegrees: optical axis above horizontal (positive = looking up).
         * rollDegrees: rotation about the optical axis (positive = camera's right side lower).
         */
        public CameraMount(double pitchDegrees, double rollDegrees) {
            if (!Double.isFinite(pitchDegrees) || Math.abs(pitchDegrees) >= 90
                    || !Double.isFinite(rollDegrees) || Math.abs(rollDegrees) >= 90) {
                throw new IllegalArgumentException("Camera pitch and roll must be within (-90, 90) degrees");
            }
            priorPitch = Math.toRadians(pitchDegrees);
            priorRoll = Math.toRadians(rollDegrees);
            reset();
        }

        /** Forget the learned pitch/roll; call after moving the camera. */
        public void reset() {
            normal = new double[3][3];
            target = new double[3];
            samples = 0;
            pitch = priorPitch;
            roll = priorRoll;
            up = up(pitch, roll);
        }

        public double pitchDegrees() { return Math.toDegrees(pitch); }
        public double rollDegrees() { return Math.toDegrees(roll); }
        /** Cell observations that have contributed since the last reset. */
        public int samples() { return samples; }

        /** World up in camera coordinates (x right, y down, z forward). */
        private static double[] up(double pitch, double roll) {
            return new double[]{-Math.cos(pitch) * Math.sin(roll), -Math.cos(pitch) * Math.cos(roll), Math.sin(pitch)};
        }

        /** Adds the constraint up . axis = value, then refits pitch and roll. */
        private void add(double[] axis, double value) {
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) normal[i][j] += axis[i] * axis[j];
                target[i] += axis[i] * value;
            }
            samples++;
            // Newton steps on (pitch, roll) for sum((axis . up - value)^2) plus the prior.
            double h = 1e-4;
            for (int iteration = 0; iteration < 3; iteration++) {
                double f = cost(pitch, roll);
                double fp = cost(pitch + h, roll), fm = cost(pitch - h, roll);
                double fr = cost(pitch, roll + h), fl = cost(pitch, roll - h);
                double fpr = cost(pitch + h, roll + h);
                double gp = (fp - fm) / (2 * h), gr = (fr - fl) / (2 * h);
                double hpp = (fp - 2 * f + fm) / (h * h), hrr = (fr - 2 * f + fl) / (h * h);
                double hpr = (fpr - fp - fr + f) / (h * h);
                double det = hpp * hrr - hpr * hpr;
                if (!(hpp > 0 && det > 0)) break;
                double dp = -(hrr * gp - hpr * gr) / det, dr = -(hpp * gr - hpr * gp) / det;
                pitch = clamp(pitch + clamp(dp, 0.1), Math.toRadians(89));
                roll = clamp(roll + clamp(dr, 0.1), Math.toRadians(89));
            }
            up = up(pitch, roll);
        }

        private double cost(double pitch, double roll) {
            double[] u = up(pitch, roll);
            double cost = 0;
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) cost += u[i] * normal[i][j] * u[j];
                cost -= 2 * target[i] * u[i];
            }
            cost /= MOUNT_CONSTRAINT_SIGMA * MOUNT_CONSTRAINT_SIGMA;
            return cost + (Math.pow(pitch - priorPitch, 2) + Math.pow(roll - priorRoll, 2))
                    / (MOUNT_PRIOR_SIGMA * MOUNT_PRIOR_SIGMA);
        }
    }

    private static final double[] NOT_MEASURED = {Double.NaN, Double.NaN, Double.NaN, Double.NaN};

    private final Limelight3A limelight;
    private final Alliance alliance;
    private final double threshold;
    private final long maxAgeMillis;
    private final CameraMount mount;
    private double logOdds;
    private double lastAngle = Double.NaN;
    private int settledSide; // Sign of the last settled reading; 0 if none since reset.
    private int movingFrames; // Consecutive measured frames within MOVING_GATE_DEGREES of level.
    private long lastMeasuredMillis;
    private boolean lostHandled = true, flipSuspected;
    private Observation last;
    private double frameTimestamp = Double.NaN;
    private long firstSeenMillis, lastCallMillis = Long.MIN_VALUE;

    /** Level camera with its own mount estimate; see {@link CameraMount}. */
    public LimelightCellTargeting(Limelight3A limelight, Alliance alliance, double cameraPitchDegrees) {
        this(limelight, alliance, new CameraMount(cameraPitchDegrees, 0));
    }

    /** Use one mount for every estimator fed by the same camera. */
    public LimelightCellTargeting(Limelight3A limelight, Alliance alliance, CameraMount mount) {
        this(limelight, alliance, mount, 0.95, 200);
    }

    /**
     * threshold: P required to report a side. Intrinsics are derived each frame from the
     * Limelight's own per-tag angles.
     */
    public LimelightCellTargeting(Limelight3A limelight, Alliance alliance, CameraMount mount,
                                 double threshold, long maxAgeMillis) {
        this.limelight = Objects.requireNonNull(limelight, "limelight");
        this.alliance = Objects.requireNonNull(alliance, "alliance");
        this.mount = Objects.requireNonNull(mount, "mount");
        if (!(threshold > 0.5 && threshold < 1) || maxAgeMillis <= 0) {
            throw new IllegalArgumentException("threshold must be in (0.5, 1); maxAgeMillis positive");
        }
        this.threshold = threshold;
        this.maxAgeMillis = maxAgeMillis;
        last = observation(Reason.STALE, NOT_MEASURED, 0, 0);
    }

    public CameraMount mount() { return mount; }

    public Observation update() {
        return evaluate(limelight.getLatestResult(), System.nanoTime() / 1_000_000);
    }

    /** nowMillis is monotonic time in milliseconds (injectable for deterministic tests). */
    public Observation evaluate(LLResult result, long nowMillis) {
        if (lastCallMillis != Long.MIN_VALUE && nowMillis > lastCallMillis) {
            relax(nowMillis - lastCallMillis);
        }
        lastCallMillis = nowMillis;
        if (result == null) return unmeasured(Reason.STALE, 0, nowMillis);
        double timestamp = result.getTimestamp();
        if (!Double.isFinite(timestamp) || timestamp <= 0) return unmeasured(Reason.STALE, 0, nowMillis);
        if (timestamp < frameTimestamp) frameTimestamp = Double.NaN; // Camera restart.
        boolean newFrame = timestamp != frameTimestamp;
        if (newFrame) { frameTimestamp = timestamp; firstSeenMillis = nowMillis; }
        double age = Math.max(Math.max(0, result.getStaleness()), nowMillis - firstSeenMillis)
                + result.getCaptureLatency() + result.getTargetingLatency() + result.getParseLatency();
        if (!Double.isFinite(age) || age < 0 || age > maxAgeMillis) return unmeasured(Reason.STALE, age, nowMillis);
        if (!newFrame) {
            // Same frame polled again: no new evidence, but report the relaxed probability.
            last = new Observation(stateFor(probability()), last.reason, probability(),
                    new double[]{last.hiveAngleDegrees, last.hiveAngleSigmaDegrees,
                            last.audienceTiltDegrees, last.scoringTableTiltDegrees},
                    last.supportingTags, age);
            return last;
        }
        if (!result.isValid()) return unmeasured(Reason.NO_TAGS, age, nowMillis);

        // corners[cell][member] in pixels; member index follows SDK cluster order (ID - first ID).
        double[][][][] corners = new double[2][4][][];
        int[] tags = new int[2];
        Set<Integer> seen = new HashSet<>();
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        for (LLResultTypes.FiducialResult tag : fiducials) {
            int id = tag.getFiducialId();
            int cell = cellIndex(id);
            if (cell < 0) continue;
            if (!seen.add(id)) return unmeasured(Reason.UNRELIABLE, age, nowMillis);
            double[][] points = corners(tag.getTargetCorners());
            if (points == null) return unmeasured(Reason.UNRELIABLE, age, nowMillis);
            int firstId = alliance == Alliance.RED ? (cell == 0 ? 34 : 30) : (cell == 0 ? 38 : 42);
            corners[cell][id - firstId] = points;
            tags[cell]++;
        }
        if (tags[0] < 2 && tags[1] < 2) return unmeasured(Reason.NO_TAGS, age, nowMillis);
        double[] camera = intrinsics(fiducials);
        Fit[] fits = new Fit[2];
        double[] measurement = NOT_MEASURED.clone();
        double weightedSum = 0, weight = 0;
        int used = 0;
        for (int cell = 0; cell < 2; cell++) {
            if (tags[cell] < 2 || camera == null) continue;
            Fit fit = fit(corners[cell], camera);
            if (fit == null) continue;
            // The tag row is parallel to the pivot axis, which is horizontal: up . xAxis = 0.
            mount.add(fit.xAxis, 0);
            double tilt = Math.toDegrees(Math.asin(clamp(dot(fit.yAxis, mount.up), 1)));
            double fitSigma = fit.tiltSigmaDegrees(mount.up);
            if (!(fitSigma <= MAX_FIT_SIGMA_DEGREES)) continue;
            fits[cell] = fit;
            measurement[2 + cell] = tilt;
            double sigma = Math.hypot(fitSigma, MOUNT_SIGMA_DEGREES);
            // The HIVE is rigid: the scoring-table cell's tilt is the hive angle negated.
            weightedSum += (cell == 0 ? tilt : -tilt) / (sigma * sigma);
            weight += 1 / (sigma * sigma);
            used += tags[cell];
        }
        if (weight == 0) return unmeasured(Reason.UNRELIABLE, age, nowMillis);
        double angle = weightedSum / weight, sigma = Math.sqrt(1 / weight);
        measurement[0] = angle;
        measurement[1] = sigma;
        logOdds = clamp(logOdds + FRAME_EVIDENCE_WEIGHT
                * Math.log(likelihood(angle, 1, sigma) / likelihood(angle, -1, sigma)), MAX_LOG_ODDS);
        lastAngle = angle;
        if (Math.abs(angle) >= FLIP_SUSPECT_DEGREES) settledSide = angle > 0 ? 1 : -1;
        lastMeasuredMillis = nowMillis;
        lostHandled = false;
        flipSuspected = false;
        // Settled readings of a confidently known state also constrain the mount's pitch.
        double p = probability();
        if (Math.abs(angle) >= FLIP_SUSPECT_DEGREES && (p > 0.99 || p < 0.01) && (angle > 0) == (p > 0.5)) {
            for (int cell = 0; cell < 2; cell++) {
                if (fits[cell] != null) {
                    // A settled cell's outer edge sits at +/-30 degrees: up . yAxis = +/-sin 30.
                    mount.add(fits[cell].yAxis, ((angle > 0) == (cell == 0) ? 1 : -1)
                            * Math.sin(Math.toRadians(SETTLED_DEGREES)));
                }
            }
        }
        movingFrames = Math.abs(angle) < MOVING_GATE_DEGREES ? movingFrames + 1 : 0;
        last = observation(Reason.MEASURED, measurement, used, age);
        return last;
    }

    /** Forget the HIVE estimate (for example, between matches). */
    public void reset() {
        logOdds = 0;
        lastAngle = Double.NaN;
        settledSide = 0;
        movingFrames = 0;
        lostHandled = true;
        flipSuspected = false;
        frameTimestamp = Double.NaN;
        last = observation(Reason.STALE, NOT_MEASURED, 0, 0);
    }

    private double probability() { return 1 / (1 + Math.exp(-logOdds)); }

    /** UNKNOWN while the HIVE was last seen moving; otherwise thresholded P. */
    private State stateFor(double p) {
        if (movingFrames >= 2) return State.UNKNOWN;
        return p >= threshold ? State.AUDIENCE_UP : p <= 1 - threshold ? State.SCORING_TABLE_UP : State.UNKNOWN;
    }

    private Observation observation(Reason reason, double[] measurement, int tags, double age) {
        double p = probability();
        return new Observation(stateFor(p), reason, p, measurement, tags, age);
    }

    /** No usable measurement this call: hold the estimate, checking for a flip in progress. */
    private Observation unmeasured(Reason reason, double age, long nowMillis) {
        if (!lostHandled && nowMillis - lastMeasuredMillis >= LOST_MILLIS) {
            lostHandled = true;
            movingFrames = 0; // The moving gate lapses here; the flip lean below takes over.
            // Tilting away from the settled side but not yet past level: lean toward a flip, up to
            // 50% at level. Readings past level have already moved the estimate themselves.
            if (settledSide != 0 && Math.abs(lastAngle) < FLIP_SUSPECT_DEGREES && lastAngle * settledSide >= 0) {
                double lean = 0.5 * (1 - Math.abs(lastAngle) / SETTLED_DEGREES);
                double p = probability();
                p = settledSide > 0 ? p * (1 - lean) : p + lean * (1 - p);
                logOdds = clamp(Math.log(p / (1 - p)), MAX_LOG_ODDS);
                flipSuspected = true;
            }
        }
        last = observation(flipSuspected && reason == Reason.NO_TAGS ? Reason.FLIP_SUSPECTED : reason,
                NOT_MEASURED, 0, age);
        return last;
    }

    /** Unseen flips: a symmetric two-state chain relaxes P toward 0.5. */
    private void relax(long dtMillis) {
        double q = 0.5 * (1 - Math.exp(-2 * UNSEEN_FLIP_RATE * dtMillis / 1000.0));
        double p = probability();
        p = p * (1 - q) + (1 - p) * q;
        logOdds = Math.log(p / (1 - p));
    }

    /** Density of a hive-angle reading given the HIVE belongs to side (+1 audience, -1 scoring). */
    private static double likelihood(double angle, int side, double sigma) {
        double s = Math.hypot(sigma, SETTLED_SIGMA_DEGREES);
        double settled = Math.exp(-0.5 * Math.pow((angle - side * SETTLED_DEGREES) / s, 2))
                / (s * Math.sqrt(2 * Math.PI));
        // Moving: anywhere on this side of level (a bistable HIVE cannot cross without flipping).
        double lo = side > 0 ? -BASIN_OVERLAP_DEGREES : -SETTLED_DEGREES - 5;
        double hi = side > 0 ? SETTLED_DEGREES + 5 : BASIN_OVERLAP_DEGREES;
        double moving = (phi((hi - angle) / sigma) - phi((lo - angle) / sigma)) / (hi - lo);
        return (1 - MOVING_PROBABILITY - OUTLIER_PROBABILITY) * settled
                + MOVING_PROBABILITY * moving + OUTLIER_PROBABILITY / 180;
    }

    /** Standard normal CDF (Abramowitz and Stegun 7.1.26, error under 1e-7). */
    private static double phi(double z) {
        double x = Math.abs(z) / Math.sqrt(2), t = 1 / (1 + 0.3275911 * x);
        double erf = 1 - t * (0.254829592 + t * (-0.284496736 + t * (1.421413741
                + t * (-1.453152027 + t * 1.061405429)))) * Math.exp(-x * x);
        return 0.5 * (1 + (z < 0 ? -erf : erf));
    }

    private int cellIndex(int id) {
        if (alliance == Alliance.RED) {
            if (id >= 34 && id <= 37) return 0;
            if (id >= 30 && id <= 33) return 1;
        } else {
            if (id >= 38 && id <= 41) return 0;
            if (id >= 42 && id <= 45) return 1;
        }
        return -1;
    }

    /** Four finite, distinct corners in pixels; no semantic corner ordering is assumed. */
    private static double[][] corners(List<List<Double>> corners) {
        if (corners == null || corners.size() != 4) return null;
        double[][] points = new double[4][];
        for (int i = 0; i < 4; i++) {
            List<Double> c = corners.get(i);
            if (c == null || c.size() != 2 || c.get(0) == null || c.get(1) == null
                    || !Double.isFinite(c.get(0)) || !Double.isFinite(c.get(1))) return null;
            points[i] = new double[]{c.get(0), c.get(1)};
            for (int j = 0; j < i; j++) {
                if (Math.hypot(points[i][0] - points[j][0], points[i][1] - points[j][1]) < 1) return null;
            }
        }
        return points;
    }

    private static double[] mean(double[][] points) {
        double x = 0, y = 0;
        for (double[] p : points) { x += p[0] / points.length; y += p[1] / points.length; }
        return new double[]{x, y};
    }

    /**
     * Pinhole {f, cx, cy} in the corner pixel frame, fitted from every tag's corner-mean pixel
     * and the Limelight's calibrated principal-point angles (tx/ty_nocross). Null if degenerate.
     */
    private static double[] intrinsics(List<LLResultTypes.FiducialResult> fiducials) {
        int n = 0;
        double[] u = new double[fiducials.size()], v = new double[u.length];
        double[] a = new double[u.length], b = new double[u.length];
        for (LLResultTypes.FiducialResult tag : fiducials) {
            double[][] points = corners(tag.getTargetCorners());
            double tx = tag.getTargetXDegreesNoCrosshair(), ty = tag.getTargetYDegreesNoCrosshair();
            if (points == null || !Double.isFinite(tx) || !Double.isFinite(ty)) continue;
            double[] center = mean(points);
            u[n] = center[0]; v[n] = center[1];
            a[n] = Math.tan(Math.toRadians(tx)); b[n] = Math.tan(Math.toRadians(ty));
            n++;
        }
        if (n < 2) return null;
        double mu = 0, mv = 0, ma = 0, mb = 0;
        for (int i = 0; i < n; i++) { mu += u[i] / n; mv += v[i] / n; ma += a[i] / n; mb += b[i] / n; }
        // u = cx + f*tan(tx); v = cy - f*tan(ty) (ty is positive up, pixel y is down).
        double num = 0, den = 0;
        for (int i = 0; i < n; i++) {
            num += (u[i] - mu) * (a[i] - ma) - (v[i] - mv) * (b[i] - mb);
            den += (a[i] - ma) * (a[i] - ma) + (b[i] - mb) * (b[i] - mb);
        }
        if (den < n * MIN_ANGLE_SPREAD) return null;
        double f = num / den;
        if (!Double.isFinite(f) || f <= 0) return null;
        return new double[]{f, mu - f * ma, mv + f * mb};
    }

    /** Cluster axes in camera coordinates plus the rotation covariance of the fit. */
    private static final class Fit {
        final double[] xAxis, yAxis;
        final double[][] rotationCovariance;
        Fit(double[] xAxis, double[] yAxis, double[][] rotationCovariance) {
            this.xAxis = xAxis; this.yAxis = yAxis; this.rotationCovariance = rotationCovariance;
        }
        /** 1-sigma of the yAxis elevation against the given up vector. */
        double tiltSigmaDegrees(double[] up) {
            double[] g = cross(yAxis, up); // d(yAxis . up) / d(rotation vector)
            double variance = 0;
            for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) variance += g[i] * rotationCovariance[i][j] * g[j];
            double cos = Math.sqrt(Math.max(0, 1 - Math.pow(dot(yAxis, up), 2)));
            return Math.toDegrees(Math.sqrt(Math.max(0, variance)) / Math.max(cos, 0.2));
        }
    }

    /**
     * Joint pose fit of all visible member corners: homography initialization, then a rigid
     * 6-DOF Gauss-Newton refinement. Null when the fit is unreliable.
     */
    private static Fit fit(double[][][] members, double[] camera) {
        int lo = -1, hi = -1;
        for (int k = 0; k < 4; k++) if (members[k] != null) { if (lo < 0) lo = k; hi = k; }
        double[] cLo = mean(members[lo]), cHi = mean(members[hi]);
        double dx = cHi[0] - cLo[0], dy = cHi[1] - cLo[1], length = Math.hypot(dx, dy);
        if (length < 4) return null;
        dx /= length; dy /= length;
        // Viewing a tag's front, its +y axis is the image +x direction rotated 90 degrees toward
        // image-up. Assign each corner to its tag-frame quadrant; corner list order is irrelevant.
        double px = dy, py = -dx;
        int count = 0;
        double[][] plane = new double[16][], image = new double[16][];
        for (int k = 0; k < 4; k++) {
            if (members[k] == null) continue;
            double[] center = mean(members[k]);
            int quadrants = 0;
            for (double[] c : members[k]) {
                double sx = Math.signum((c[0] - center[0]) * dx + (c[1] - center[1]) * dy);
                double sy = Math.signum((c[0] - center[0]) * px + (c[1] - center[1]) * py);
                if (sx == 0 || sy == 0) return null;
                quadrants |= 1 << ((sx > 0 ? 1 : 0) + (sy > 0 ? 2 : 0));
                plane[count] = new double[]{MEMBER_X[k] + sx * HALF_TAG, sy * HALF_TAG};
                image[count] = new double[]{(c[0] - camera[1]) / camera[0], (c[1] - camera[2]) / camera[0]};
                count++;
            }
            if (quadrants != 15) return null;
        }
        double[] h = homography(plane, image, count);
        if (h == null) return null;
        // H ~ [r1 r2 t] in normalized camera coordinates (x right, y down, z forward);
        // h33 = 1 keeps the cluster in front of the camera (t.z > 0).
        double[] h1 = {h[0], h[3], h[6]}, h2 = {h[1], h[4], h[7]}, h3 = {h[2], h[5], 1};
        double n1 = norm(h1), n2 = norm(h2);
        if (n1 == 0 || n2 == 0) return null;
        double[] r1 = times(h1, 1 / n1);
        double[] r2 = minus(h2, times(r1, dot(r1, h2)));
        r2 = times(r2, 1 / norm(r2));
        double[][] pose = {r1, r2, cross(r1, r2), times(h3, 2 / (n1 + n2))};
        // Refine as a rigid 6-DOF pose. The homography's two extra degrees of freedom absorb
        // corner noise that would otherwise appear as tilt error on a single row of tags.
        for (int iteration = 0; iteration < 10; iteration++) {
            double[][] normal = normalEquations(pose, plane, image, count);
            double[] step = normal == null ? null : solve(normal);
            if (step == null) return null;
            pose = moved(pose, step);
            if (norm(new double[]{step[0], step[1], step[2]}) < 1e-7) break;
        }
        double[] residual = residuals(pose, plane, image, count);
        if (residual == null) return null;
        double rmsPixels = camera[0] * Math.sqrt(dot(residual, residual) / count);
        if (rmsPixels > MAX_REPROJECTION_PIXELS) return null;
        // The printed face (r3) must point toward the camera; otherwise corner assignment is wrong.
        if (dot(pose[2], pose[3]) >= 0) return null;
        // Rotation covariance: (corner noise)^2 (J^T J)^-1, corner noise at least 0.5 px.
        double[][] normal = normalEquations(pose, plane, image, count);
        if (normal == null) return null;
        double noise = Math.max(rmsPixels, 0.5) / camera[0];
        double[][] covariance = new double[3][3];
        for (int c = 0; c < 3; c++) {
            double[][] system = new double[6][7];
            for (int i = 0; i < 6; i++) {
                System.arraycopy(normal[i], 0, system[i], 0, 6);
                system[i][6] = i == c ? 1 : 0;
            }
            double[] column = solve(system);
            if (column == null) return null;
            for (int i = 0; i < 3; i++) covariance[i][c] = noise * noise * column[i];
        }
        return new Fit(pose[0], pose[1], covariance);
    }

    /** Least-squares DLT with h33 = 1; returns row-major h11..h32 or null if singular. */
    private static double[] homography(double[][] plane, double[][] image, int count) {
        double[][] normal = new double[8][9];
        for (int i = 0; i < count; i++) {
            double X = plane[i][0], Y = plane[i][1], x = image[i][0], y = image[i][1];
            accumulate(normal, new double[]{X, Y, 1, 0, 0, 0, -x * X, -x * Y}, x);
            accumulate(normal, new double[]{0, 0, 0, X, Y, 1, -y * X, -y * Y}, y);
        }
        return solve(normal);
    }

    /**
     * Gauss-Newton normal equations [J^T J | -J^T r] for {rotation vector (camera frame),
     * translation}, or null if the cluster is behind the camera.
     */
    private static double[][] normalEquations(double[][] pose, double[][] plane, double[][] image, int count) {
        double[] base = residuals(pose, plane, image, count);
        if (base == null) return null;
        double[][] jacobian = new double[6][];
        for (int p = 0; p < 6; p++) {
            double[] delta = new double[6];
            delta[p] = 1e-6;
            double[] moved = residuals(moved(pose, delta), plane, image, count);
            if (moved == null) return null;
            jacobian[p] = new double[base.length];
            for (int i = 0; i < base.length; i++) jacobian[p][i] = (moved[i] - base[i]) / 1e-6;
        }
        double[][] normal = new double[6][7];
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) normal[i][j] = dot(jacobian[i], jacobian[j]);
            normal[i][6] = -dot(jacobian[i], base);
        }
        return normal;
    }

    /** Pose rotated by step[0..2] (camera-frame rotation vector) and translated by step[3..5]. */
    private static double[][] moved(double[][] pose, double[] step) {
        double[] w = {step[0], step[1], step[2]};
        return new double[][]{rotate(pose[0], w), rotate(pose[1], w), rotate(pose[2], w),
                {pose[3][0] + step[3], pose[3][1] + step[4], pose[3][2] + step[5]}};
    }

    /** Normalized-image reprojection residuals (x, y per point), or null if behind the camera. */
    private static double[] residuals(double[][] pose, double[][] plane, double[][] image, int count) {
        double[] r = new double[2 * count];
        for (int i = 0; i < count; i++) {
            double[] p = new double[3];
            for (int a = 0; a < 3; a++) p[a] = plane[i][0] * pose[0][a] + plane[i][1] * pose[1][a] + pose[3][a];
            if (p[2] <= 0) return null;
            r[2 * i] = p[0] / p[2] - image[i][0];
            r[2 * i + 1] = p[1] / p[2] - image[i][1];
        }
        return r;
    }

    /** Rodrigues rotation of v by rotation vector w. */
    private static double[] rotate(double[] v, double[] w) {
        double theta = norm(w);
        if (theta < 1e-12) return v.clone();
        double[] k = times(w, 1 / theta), kv = cross(k, v);
        double c = Math.cos(theta), s = Math.sin(theta), d = dot(k, v) * (1 - c);
        return new double[]{v[0] * c + kv[0] * s + k[0] * d, v[1] * c + kv[1] * s + k[1] * d,
                v[2] * c + kv[2] * s + k[2] * d};
    }

    /** Gauss-Jordan solve of an n x (n+1) augmented system (modified in place); null if singular. */
    private static double[] solve(double[][] a) {
        int n = a.length;
        for (int col = 0; col < n; col++) {
            int pivot = col;
            for (int r = col + 1; r < n; r++) if (Math.abs(a[r][col]) > Math.abs(a[pivot][col])) pivot = r;
            if (Math.abs(a[pivot][col]) < 1e-12) return null;
            double[] swap = a[col]; a[col] = a[pivot]; a[pivot] = swap;
            for (int r = 0; r < n; r++) {
                if (r == col) continue;
                double factor = a[r][col] / a[col][col];
                for (int c = col; c <= n; c++) a[r][c] -= factor * a[col][c];
            }
        }
        double[] x = new double[n];
        for (int i = 0; i < n; i++) {
            x[i] = a[i][n] / a[i][i];
            if (!Double.isFinite(x[i])) return null;
        }
        return x;
    }

    private static void accumulate(double[][] normal, double[] row, double value) {
        for (int i = 0; i < 8; i++) {
            for (int j = 0; j < 8; j++) normal[i][j] += row[i] * row[j];
            normal[i][8] += row[i] * value;
        }
    }

    private static double clamp(double value, double limit) { return Math.max(-limit, Math.min(limit, value)); }
    private static double dot(double[] a, double[] b) {
        double sum = 0;
        for (int i = 0; i < a.length; i++) sum += a[i] * b[i];
        return sum;
    }
    private static double norm(double[] a) { return Math.sqrt(dot(a, a)); }
    private static double[] times(double[] a, double s) { return new double[]{a[0] * s, a[1] * s, a[2] * s}; }
    private static double[] minus(double[] a, double[] b) { return new double[]{a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
    private static double[] cross(double[] a, double[] b) {
        return new double[]{a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    }
}
