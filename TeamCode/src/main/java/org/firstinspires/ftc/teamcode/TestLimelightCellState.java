package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Live, vision-only HIVE-state test. Requires only a configured Limelight 3A.
 * Triangle: forget the HIVE estimates and the learned camera mount (after moving the camera).
 * Dpad up/down: change the starting camera pitch (also resets).
 */
@TeleOp(name = "Test: Limelight Cell State", group = "Test")
public class TestLimelightCellState extends LinearOpMode {
    // Configure this slot for AprilTag 36h11, IDs 30-45, with corner output enabled.
    private static final int APRILTAG_PIPELINE = 0;
    // Starting camera pitch above horizontal; refined online while the HIVE is in view.
    private static final double DEFAULT_CAMERA_PITCH_DEGREES = 25;

    private Limelight3A limelight;
    private double pitch = DEFAULT_CAMERA_PITCH_DEGREES;
    private LimelightCellTargeting.CameraMount mount;
    private LimelightCellTargeting red, blue;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        rebuild();
        boolean wasUp = false, wasDown = false, wasTriangle = false;
        telemetry.setMsTransmissionInterval(100);

        try {
            limelight.pipelineSwitch(APRILTAG_PIPELINE);
            limelight.start();
            // Preview during INIT, then continue updating until STOP.
            while (!isStopRequested() && (opModeInInit() || opModeIsActive())) {
                if (gamepad1.dpad_up && !wasUp || gamepad1.dpad_down && !wasDown) {
                    pitch = Math.max(-80, Math.min(80, pitch + (gamepad1.dpad_up ? 1 : -1)));
                    rebuild();
                }
                if (gamepad1.triangle && !wasTriangle) rebuild();
                wasUp = gamepad1.dpad_up;
                wasDown = gamepad1.dpad_down;
                wasTriangle = gamepad1.triangle;

                // Both estimators evaluate the same camera frame and monotonic time.
                LLResult result = limelight.getLatestResult();
                long nowMillis = System.nanoTime() / 1_000_000;
                LimelightCellTargeting.Observation redState = red.evaluate(result, nowMillis);
                LimelightCellTargeting.Observation blueState = blue.evaluate(result, nowMillis);

                telemetry.addData("Mode", isStarted() ? "RUNNING" : "INIT preview - press Play");
                telemetry.addData("Pipeline requested", APRILTAG_PIPELINE);
                telemetry.addData("Start pitch (dpad up/down)", "%.0f deg", pitch);
                telemetry.addData("Learned mount", "pitch %.1f, roll %.1f deg (%d samples)",
                        mount.pitchDegrees(), mount.rollDegrees(), mount.samples());
                showState("RED", redState);
                showState("BLUE", blueState);
                telemetry.addData("IDs in latest result", tagIds(result));
                telemetry.addLine("Hive angle: +30 audience cell up, -30 scoring-table cell up.");
                telemetry.addLine("Triangle: reset estimates and learned mount (after moving camera).");
                telemetry.update();
                sleep(20);
            }
        } finally {
            limelight.stop();
        }
    }

    /** Fresh estimates for both alliances, sharing one newly started camera mount. */
    private void rebuild() {
        mount = new LimelightCellTargeting.CameraMount(pitch, 0);
        red = new LimelightCellTargeting(limelight, LimelightCellTargeting.Alliance.RED, mount);
        blue = new LimelightCellTargeting(limelight, LimelightCellTargeting.Alliance.BLUE, mount);
    }

    private void showState(String alliance, LimelightCellTargeting.Observation state) {
        telemetry.addData(alliance + " cell UP", "%s (P audience up %.2f)", state.state, state.probabilityAudienceUp);
        telemetry.addData(alliance + " evidence", "%s | tags %d | age %.0f ms",
                state.reason, state.supportingTags, state.ageMillis);
        telemetry.addData(alliance + " hive angle", Double.isNaN(state.hiveAngleDegrees) ? "--"
                : String.format("%+.0f +/- %.0f deg", state.hiveAngleDegrees, state.hiveAngleSigmaDegrees));
    }

    private static String tagIds(LLResult result) {
        if (result == null || !result.isValid()) return "none";
        StringBuilder ids = new StringBuilder();
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            if (ids.length() > 0) ids.append(", ");
            ids.append(tag.getFiducialId());
        }
        return ids.length() == 0 ? "none" : ids.toString();
    }
}
