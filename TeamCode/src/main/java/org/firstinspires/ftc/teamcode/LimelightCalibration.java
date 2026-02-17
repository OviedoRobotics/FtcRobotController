package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.apache.commons.math3.optim.InitialGuess;
import org.apache.commons.math3.optim.MaxEval;
import org.apache.commons.math3.optim.PointValuePair;
import org.apache.commons.math3.optim.SimpleBounds;
import org.apache.commons.math3.optim.nonlinear.scalar.GoalType;
import org.apache.commons.math3.optim.nonlinear.scalar.ObjectiveFunction;
import org.apache.commons.math3.optim.nonlinear.scalar.noderiv.NelderMeadSimplex;
import org.apache.commons.math3.optim.nonlinear.scalar.noderiv.SimplexOptimizer;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.json.JSONObject;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Limelight Quick Optimizer - Optimizes AprilTag detection stability
 * <p>
 * This OpMode automatically calibrates Limelight camera settings for optimal
 * AprilTag detection using Apache Commons Math optimization algorithms.
 * It uses the Nelder-Mead simplex method to efficiently search the parameter
 * space for optimal exposure, gain, and image processing settings.
 * <p>
 * Usage:
 * 1. Position robot so a target AprilTag (ID 20 or 24) is visible
 * 2. Run this OpMode
 * 3. Wait for optimization to complete (~1-2 minutes)
 * 4. Press [triangle] to save settings permanently or [cross] to exit
 * <p>
 * The optimization happens in phases:
 * Phase 1: Detect tag and establish baseline
 * Phase 2: Optimize exposure & gain (Nelder-Mead)
 * Phase 3: Optimize image parameters (Nelder-Mead)
 * Phase 4: Test discrete parameters (refine method, black level)
 * Phase 5: Verification
 * <p>
 * Inspired by: https://github.com/6603GuildofGears/Limelight-AprilTag-Calibration
 */
@TeleOp(name = "Limelight: Calibration", group = "Test")
public class LimelightCalibration extends LinearOpMode {

    // Tags to look for (will auto-detect)
    private static final int[] TARGET_TAGS = {20, 24};

    // Safe baseline settings
    private static final int DEFAULT_EXPOSURE = 3200;
    private static final double DEFAULT_GAIN = 8.2;
    private static final int DEFAULT_REFINE = 1;
    private static final int DEFAULT_BLACK_LEVEL = 20;
    private static final double DEFAULT_SHARPENING = 0.0;
    private static final int DEFAULT_RED_BALANCE = 1280;
    private static final int DEFAULT_BLUE_BALANCE = 1500;

    private Limelight3A limelight;
    private Method updatePipelineMethod;
    private Map<String, Object> bestSettings = new HashMap<>();
    private double bestStability = 999.0;
    private int detectedTagId = -1;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.stop(); // make sure we're stopped...
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(5);
        limelight.start();

        // Get private updatePipeline method via reflection
        try {
            updatePipelineMethod = Limelight3A.class.getDeclaredMethod("updatePipeline", JSONObject.class, boolean.class);
            updatePipelineMethod.setAccessible(true);
        } catch (NoSuchMethodException e) {
            telemetry.addLine("ERROR: Cannot access updatePipeline method");
            telemetry.update();
            throw new RuntimeException(e);
        }

        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Limelight Calibration");
        telemetry.addLine("Ready to optimize. Press PLAY to start.");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            runOptimization();
        }

        limelight.stop();
    }

    private void runOptimization() {
        long startTime = System.currentTimeMillis();

        telemetry.addLine("========================================");
        telemetry.addLine("LIMELIGHT QUICK OPTIMIZER");
        telemetry.addLine("========================================");
        telemetry.update();

        // Initialize safe settings
        initializeSafeSettings();

        // Phase 1: Detect tag and establish baseline
        telemetry.addLine("\nPhase 1: Detecting tag...");
        telemetry.update();

        applySettings(bestSettings);
        MeasurementResult baseline = measure(50, -1);

        if (baseline.detectionRate < 30 || baseline.detectedTag == -1) {
            telemetry.addLine("  Low detection. Trying alternatives...");
            telemetry.update();
            bestSettings.put("exposure", 2000);
            applySettings(bestSettings);
            baseline = measure(50, -1);
        }

        if (baseline.detectionRate < 30 || baseline.detectedTag == -1) {
            telemetry.addLine("  ERROR: Cannot detect tag!");
            telemetry.addLine("  Check camera view.");
            telemetry.update();
            sleep(5000);
            return;
        }

        detectedTagId = baseline.detectedTag;
        bestStability = baseline.stability;

        telemetry.addLine(String.format("  Detected Tag %d", detectedTagId));
        telemetry.addLine(String.format("  Baseline: %.2fmm (%.0f%% detect)",
                baseline.stability, baseline.detectionRate));
        telemetry.update();
        sleep(1500);

        // Phase 2: Optimize exposure and gain using Nelder-Mead
        telemetry.addLine("\nPhase 2: Optimizing exposure & gain...");
        telemetry.update();

        double[] optimizedExpGain = optimizeExposureAndGain();
        bestSettings.put("exposure", (int) Math.round(optimizedExpGain[0]));
        bestSettings.put("sensor_gain", optimizedExpGain[1]);

        MeasurementResult phase2Result = measure(30, detectedTagId);
        bestStability = phase2Result.stability;

        telemetry.addLine(String.format("  Optimized: exp=%d gain=%.1f (%.2fmm, %.1fms)",
                bestSettings.get("exposure"), bestSettings.get("sensor_gain"),
                bestStability, phase2Result.avgLatency));
        telemetry.update();
        sleep(1500);

        // Phase 3: Optimize secondary continuous parameters
        telemetry.addLine("\nPhase 3: Optimizing image parameters...");
        telemetry.update();

        double[] optimizedImageParams = optimizeImageParameters();
        bestSettings.put("sharpening", optimizedImageParams[0]);
        bestSettings.put("red_balance", (int) Math.round(optimizedImageParams[1]));
        bestSettings.put("blue_balance", (int) Math.round(optimizedImageParams[2]));

        MeasurementResult phase3Result = measure(30, detectedTagId);
        double phase3Cost = phase3Result.stability + (phase3Result.avgLatency * 0.1);
        double currentBestCost = bestStability; // Approximate cost from Phase 2

        if (phase3Cost < currentBestCost) {
            bestStability = phase3Result.stability;
        }

        telemetry.addLine(String.format("  Optimized image params (%.2fmm, %.1fms)",
                bestStability, phase3Result.avgLatency));
        telemetry.update();
        sleep(1500);

        // Phase 4: Test discrete parameters
        telemetry.addLine("\nPhase 4: Testing discrete parameters...");
        telemetry.update();

        // Refine method
        int[] refineMethods = {0, 1, 2, 3};
        optimizeDiscreteParameter("fiducial_refine_method", refineMethods);

        // Black level
        int[] blackLevels = {0, 3, 6, 9, 12, 15, 18, 20, 22, 25};
        optimizeDiscreteParameter("black_level", blackLevels);

        double currentLatency = bestSettings.containsKey("avgLatency") ?
                ((Number) bestSettings.get("avgLatency")).doubleValue() : 0;
        telemetry.addLine(String.format("  Discrete params optimized (%.2fmm, %.1fms)",
                bestStability, currentLatency));
        telemetry.update();
        sleep(1500);

        // Phase 5: Verification
        telemetry.addLine("\nPhase 5: Verification...");
        telemetry.update();

        applySettings(bestSettings);
        sleep(300);

        List<MeasurementResult> verifyResults = new ArrayList<>();
        for (int i = 0; i < 3; i++) {
            if (!opModeIsActive()) return;
            MeasurementResult result = measure(50, detectedTagId);
            verifyResults.add(result);
            telemetry.addLine(String.format("  Round %d: %.2fmm (%.0f%%) lat: %.1f/%.1f/%.1fms",
                    i + 1, result.stability, result.detectionRate,
                    result.minLatency, result.avgLatency, result.maxLatency));
            telemetry.update();
        }

        double finalStability = 0;
        double finalDetection = 0;
        double finalAvgLatency = 0;
        double finalMinLatency = 0;
        double finalMaxLatency = 0;
        for (MeasurementResult r : verifyResults) {
            finalStability += r.stability;
            finalDetection += r.detectionRate;
            finalAvgLatency += r.avgLatency;
            finalMinLatency += r.minLatency;
            finalMaxLatency += r.maxLatency;
        }
        finalStability /= verifyResults.size();
        finalDetection /= verifyResults.size();
        finalAvgLatency /= verifyResults.size();
        finalMinLatency /= verifyResults.size();
        finalMaxLatency /= verifyResults.size();

        long elapsed = (System.currentTimeMillis() - startTime) / 1000;

        // Final report
        telemetry.clear();
        telemetry.addLine("========================================");
        telemetry.addLine("OPTIMIZATION COMPLETE");
        telemetry.addLine("========================================");
        telemetry.addLine(String.format("\nTime: %d seconds", elapsed));
        telemetry.addLine(String.format("Tag: %d", detectedTagId));
        telemetry.addLine(String.format("\nFinal Stability: %.2fmm (%.0f%% detect)",
                finalStability, finalDetection));
        telemetry.addLine(String.format("Camera Latency: %.1f/%.1f/%.1fms (min/avg/max)",
                finalMinLatency, finalAvgLatency, finalMaxLatency));
        telemetry.addLine("\nOptimal Settings:");
        telemetry.addLine(String.format("  exposure: %d", bestSettings.get("exposure")));
        telemetry.addLine(String.format("  sensor_gain: %.1f", bestSettings.get("sensor_gain")));
        telemetry.addLine(String.format("  refine: %d", bestSettings.get("fiducial_refine_method")));
        telemetry.addLine(String.format("  black_level: %d", bestSettings.get("black_level")));
        telemetry.addLine(String.format("  sharpening: %.2f", bestSettings.get("sharpening")));
        telemetry.addLine(String.format("  red_balance: %d", bestSettings.get("red_balance")));
        telemetry.addLine(String.format("  blue_balance: %d", bestSettings.get("blue_balance")));
        telemetry.addLine("\nSettings applied to camera!");
        telemetry.addLine("========================================");
        telemetry.addLine("\nPress [triangle] to SAVE settings permanently");
        telemetry.addLine("Press [cross] to exit without saving");
        telemetry.addLine("(Settings are currently temporary)");
        telemetry.update();

        // Wait for user to save or exit
        boolean settingsSaved = false;
        while (opModeIsActive() && !settingsSaved) {
            if (gamepad1.triangleWasPressed()) {
                telemetry.addLine("\nSaving settings permanently...");
                telemetry.update();
                flushSettings(bestSettings);
                settingsSaved = true;
                telemetry.addLine("Settings SAVED to camera!");
                telemetry.addLine("Press [cross] to exit");
                telemetry.update();
            } else if (gamepad1.crossWasPressed()) {
                telemetry.addLine("\nExiting without saving.");
                telemetry.addLine("Settings were temporary only.");
                telemetry.update();
                sleep(2000);
                break;
            }
            sleep(100);
        }

        // Wait for final exit
        if (settingsSaved) {
            while (opModeIsActive() && !gamepad1.crossWasPressed()) {
                sleep(100);
            }
        }
    }

    private void initializeSafeSettings() {
        bestSettings.put("exposure", DEFAULT_EXPOSURE);
        bestSettings.put("sensor_gain", DEFAULT_GAIN);
        bestSettings.put("fiducial_refine_method", DEFAULT_REFINE);
        bestSettings.put("black_level", DEFAULT_BLACK_LEVEL);
        bestSettings.put("sharpening", DEFAULT_SHARPENING);
        bestSettings.put("red_balance", DEFAULT_RED_BALANCE);
        bestSettings.put("blue_balance", DEFAULT_BLUE_BALANCE);
    }

    /**
     * Optimize exposure and gain using Nelder-Mead simplex algorithm
     * Returns: [exposure, gain]
     */
    private double[] optimizeExposureAndGain() {
        // Parameter vector: [exposure, gain]
        double[] initialGuess = {
                ((Number) bestSettings.get("exposure")).doubleValue(),
                ((Number) bestSettings.get("sensor_gain")).doubleValue()
        };

        // Bounds: exposure [800, 3300], gain [4, 30]
        double[] lowerBounds = {800.0, 4.0};
        double[] upperBounds = {3300.0, 30.0};

        // Objective function - balance stability and latency
        ObjectiveFunction objectiveFunction = new ObjectiveFunction(point -> {
            if (!opModeIsActive()) return 999.0; // Early exit

            int exposure = (int) Math.round(point[0]);
            double gain = point[1];

            Map<String, Object> test = new HashMap<>(bestSettings);
            test.put("exposure", exposure);
            test.put("sensor_gain", gain);
            applySettings(test);

            MeasurementResult result = measure(20, detectedTagId);

            // Penalize low detection rate
            if (result.detectionRate < 60) {
                return 999.0;
            }

            // Cost function: balance stability (mm) and latency (ms)
            // Weight latency by 0.1 so 10ms latency = 1mm stability
            double cost = result.stability + (result.avgLatency * 0.1);

            telemetry.addLine(String.format("  Testing exp=%d gain=%.1f: %.1fmm %.1fms (cost=%.1f)",
                    exposure, gain, result.stability, result.avgLatency, cost));
            telemetry.update();

            return cost;
        });

        try {
            SimplexOptimizer optimizer = new SimplexOptimizer(1e-3, 1e-6);
            PointValuePair result = optimizer.optimize(
                    new MaxEval(30),
                    objectiveFunction,
                    GoalType.MINIMIZE,
                    new InitialGuess(initialGuess),
                    new SimpleBounds(lowerBounds, upperBounds),
                    new NelderMeadSimplex(2)
            );

            return result.getPoint();
        } catch (Exception e) {
            telemetry.addLine("Optimization failed, using defaults");
            telemetry.update();
            return initialGuess;
        }
    }

    /**
     * Optimize image processing parameters: sharpening, red_balance, blue_balance
     * Returns: [sharpening, red_balance, blue_balance]
     */
    private double[] optimizeImageParameters() {
        // Parameter vector: [sharpening, red_balance, blue_balance]
        double[] initialGuess = {
                ((Number) bestSettings.get("sharpening")).doubleValue(),
                ((Number) bestSettings.get("red_balance")).doubleValue(),
                ((Number) bestSettings.get("blue_balance")).doubleValue()
        };

        // Bounds
        double[] lowerBounds = {0.0, 1000.0, 1200.0};
        double[] upperBounds = {0.2, 1500.0, 1800.0};

        ObjectiveFunction objectiveFunction = new ObjectiveFunction(point -> {
            if (!opModeIsActive()) return 999.0;

            double sharpening = point[0];
            int redBalance = (int) Math.round(point[1]);
            int blueBalance = (int) Math.round(point[2]);

            Map<String, Object> test = new HashMap<>(bestSettings);
            test.put("sharpening", sharpening);
            test.put("red_balance", redBalance);
            test.put("blue_balance", blueBalance);
            applySettings(test);

            MeasurementResult result = measure(20, detectedTagId);

            if (result.detectionRate < 60) {
                return 999.0;
            }

            // Include latency in cost (weight: 10ms latency = 1mm stability)
            return result.stability + (result.avgLatency * 0.1);
        });

        try {
            SimplexOptimizer optimizer = new SimplexOptimizer(1e-3, 1e-6);
            PointValuePair result = optimizer.optimize(
                    new MaxEval(25),
                    objectiveFunction,
                    GoalType.MINIMIZE,
                    new InitialGuess(initialGuess),
                    new SimpleBounds(lowerBounds, upperBounds),
                    new NelderMeadSimplex(3)
            );

            return result.getPoint();
        } catch (Exception e) {
            telemetry.addLine("Image param optimization failed, using defaults");
            telemetry.update();
            return initialGuess;
        }
    }

    /**
     * Test discrete parameter values and select the best (considering stability + latency)
     */
    private void optimizeDiscreteParameter(String paramName, int[] values) {
        double bestCost = bestStability + (bestSettings.containsKey("avgLatency") ?
                ((Number) bestSettings.get("avgLatency")).doubleValue() * 0.1 : 0);

        for (int val : values) {
            if (!opModeIsActive()) return;

            Map<String, Object> test = new HashMap<>(bestSettings);
            test.put(paramName, val);
            applySettings(test);

            MeasurementResult result = measure(25, detectedTagId);

            // Calculate cost including latency
            double cost = result.stability + (result.avgLatency * 0.1);

            if (result.detectionRate > 60 && cost < bestCost) {
                bestCost = cost;
                bestStability = result.stability;
                bestSettings.put(paramName, val);
                bestSettings.put("avgLatency", result.avgLatency);
                telemetry.addLine(String.format("  %s=%d improved: %.1fmm %.1fms (cost=%.1f)",
                        paramName, val, result.stability, result.avgLatency, cost));
                telemetry.update();
            }
        }
    }

    private void applySettings(Map<String, Object> settings) {
        try {
            JSONObject json = new JSONObject(settings);

            // Call private updatePipeline method via reflection
            // Method signature: private boolean updatePipeline(JSONObject profileJson, boolean flush)
            updatePipelineMethod.invoke(limelight, json, false);

            sleep(150); // Allow settings to apply
        } catch (Exception e) {
            telemetry.addLine("Warning: Failed to apply settings - " + e.getMessage());
            telemetry.update();
        }
    }

    private void flushSettings(Map<String, Object> settings) {
        try {
            JSONObject json = new JSONObject(settings);

            // Call private updatePipeline method with flush=true to save permanently
            // Method signature: private boolean updatePipeline(JSONObject profileJson, boolean flush)
            updatePipelineMethod.invoke(limelight, json, true);

            sleep(150); // Allow settings to flush
        } catch (Exception e) {
            telemetry.addLine("ERROR: Failed to flush settings - " + e.getMessage());
            telemetry.update();
        }
    }

    private MeasurementResult measure(int samples, int targetTag) {
        List<Double> zValues = new ArrayList<>();
        List<Double> captureTimestamps = new ArrayList<>();
        int detectedTag = -1;
        double lastCaptureTime = -1;

        while (zValues.size() < samples && opModeIsActive()) {
            try {
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {

                    double captureTime = result.getTimestamp();
                    // only count if we have a newer result.
                    if (lastCaptureTime > 0 && lastCaptureTime >= captureTime) {
                        sleep(1);
                        continue;
                    }

                    // Track capture timestamp for latency calculation
                    captureTimestamps.add(captureTime - lastCaptureTime);
                    lastCaptureTime = captureTime;

                    for (LLResultTypes.FiducialResult fid : result.getFiducialResults()) {
                        int fidId = fid.getFiducialId();

                        // Accept specific tag or any target tag
                        if (targetTag != -1 && fidId != targetTag) {
                            continue;
                        }
                        if (targetTag == -1 && !isTargetTag(fidId)) {
                            continue;
                        }

                        // Get camera-space translation (t6t_cs)
                        Pose3D targetPose = fid.getTargetPoseCameraSpace();
                        if (targetPose != null && targetPose.getPosition() != null) {
                            double z = targetPose.getPosition().z;
                            zValues.add(z);
                            detectedTag = fidId;
                            break; // One detection per sample
                        }
                    }
                }
                sleep(2);
            } catch (Exception e) {
                // Skip this sample
            }
        }

        if (zValues.size() < 5) {
            return new MeasurementResult(999.0, 0, detectedTag, 0, 0, 0);
        }

        // Calculate standard deviation
        double mean = 0;
        for (double z : zValues) {
            mean += z;
        }
        mean /= zValues.size();

        double variance = 0;
        for (double z : zValues) {
            variance += Math.pow(z - mean, 2);
        }
        variance /= zValues.size();
        double stdDev = Math.sqrt(variance);

        double detectionRate = (double) zValues.size() / samples * 100;

        // Calculate latency statistics (convert from seconds to milliseconds)
        double avgLatency = 0;
        double minLatency = 0;
        double maxLatency = 0;

        if (!captureTimestamps.isEmpty()) {
            double sum = 0;
            double min = Double.MAX_VALUE;
            double max = Double.MIN_VALUE;

            for (double latency : captureTimestamps) {
                sum += latency;
                if (latency < min) min = latency;
                if (latency > max) max = latency;
            }

            avgLatency = (sum / captureTimestamps.size()) * 1000.0;  // Convert to ms
            minLatency = min * 1000.0;  // Convert to ms
            maxLatency = max * 1000.0;  // Convert to ms
        }

        return new MeasurementResult(stdDev * 1000, detectionRate, detectedTag,
                avgLatency, minLatency, maxLatency);
    }

    private boolean isTargetTag(int tagId) {
        for (int target : TARGET_TAGS) {
            if (tagId == target) return true;
        }
        return false;
    }

    private static class MeasurementResult {
        double stability;      // Z standard deviation in mm
        double detectionRate;  // Detection percentage
        int detectedTag;       // Tag ID that was detected
        double avgLatency;     // Average latency between camera updates (ms)
        double minLatency;     // Minimum latency (ms)
        double maxLatency;     // Maximum latency (ms)

        MeasurementResult(double stability, double detectionRate, int detectedTag,
                          double avgLatency, double minLatency, double maxLatency) {
            this.stability = stability;
            this.detectionRate = detectionRate;
            this.detectedTag = detectedTag;
            this.avgLatency = avgLatency;
            this.minLatency = minLatency;
            this.maxLatency = maxLatency;
        }
    }
}
