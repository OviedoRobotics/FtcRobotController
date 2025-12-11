package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TestSpindexerContinuous", group = "Test")
public class TestSpindexerContinuous extends LinearOpMode {

    // Hardware
    private CRServo axleServo;
    private AnalogInput positionSensor;  // 0–3.3V → 0–360°

    // Configuration
    private static final String SERVO_NAME = "spinServo";
    private static final String ANALOG_NAME = "spinServoPos";
    private static final double VOLTAGE_MAX = 3.3;
    private static final double POSITION_TOLERANCE_DEG = 3.0;

    // Desired positions
    public enum TargetPosition {
        P1(60),
        P2(180),
        P3(300);

        public final double degrees;

        TargetPosition(double degrees) {
            this.degrees = degrees;
        }
    }

    // Control modes
    public enum ControlMode {
        FIXED_POWER("Fixed Power"),
        PROPORTIONAL("Proportional Power"),
        TRAPEZOIDAL("Trapezoidal Power");

        public final String displayName;
        ControlMode(String displayName) { this.displayName = displayName; }
    }

    private TargetPosition currentTarget = TargetPosition.P1;
    private ControlMode currentMode = ControlMode.PROPORTIONAL;

    // Proportional & Trapezoidal constants
    private static final double P_GAIN = 0.004;        // Power per degree of error (tune this!)
    private static final double MAX_POWER = 0.90;
    private static final double MIN_POWER_TO_MOVE = 0.08;

    // Trapezoidal profile
    private static final double ACCEL_DISTANCE_DEG = 60;   // Start slowing when within 60°
    private static final double DECEL_DISTANCE_DEG = 40;   // Full deceleration zone

    @Override
    public void runOpMode() {

        // Map hardware
        axleServo = hardwareMap.get(CRServo.class, SERVO_NAME);
        positionSensor = hardwareMap.get(AnalogInput.class, ANALOG_NAME);

        telemetry.addData("Status", "Initialized – Axon Max+ Position Control");
        telemetry.addLine("\nREADY... press START");
        telemetry.update();

        waitForStart();

        Gamepad previousGamepad = new Gamepad();

        while( opModeIsActive()) {
            // === Button handling for mode & target selection ===
            if (gamepad1.square && !previousGamepad.square) {
                currentMode = ControlMode.FIXED_POWER;
            } else if (gamepad1.cross && !previousGamepad.cross) {
                currentMode = ControlMode.PROPORTIONAL;
            } else if (gamepad1.circle && !previousGamepad.circle) {
                currentMode = ControlMode.TRAPEZOIDAL;
            }

            if (gamepad1.left_bumper && !previousGamepad.left_bumper) {
                cycleTarget(-1);
            } else if (gamepad1.right_bumper && !previousGamepad.right_bumper) {
                cycleTarget(+1);
            }

            previousGamepad.copy(gamepad1);

            // === Read current position (0–360°) ===
            double currentDegrees = voltageToDegrees(positionSensor.getVoltage());

            // === Compute error ===
            double error = angularError(currentTarget.degrees, currentDegrees);

            // === Main control loop ===
            double power = 0.0;

            switch (currentMode) {
                case FIXED_POWER:
                    power = fixedPowerControl(error);
                    break;
                case PROPORTIONAL:
                    power = proportionalControl(error);
                    break;
                case TRAPEZOIDAL:
                    power = trapezoidalControl(error, Math.abs(error));
                    break;
            }

            axleServo.setPower(power);

            // === Telemetry ===
            telemetry.addData("Mode", "%s", currentMode.displayName);
            telemetry.addData("Target", "%s → %.0f°", currentTarget.name(), currentTarget.degrees);
            telemetry.addData("Current Angle", "%.1f°", currentDegrees);
            telemetry.addData("Error", "%.1f°", error);
            telemetry.addData("Servo Power", "%.3f", power);
            telemetry.addData("Within Tolerance", Math.abs(error) <= POSITION_TOLERANCE_DEG ? "YES" : "NO");
            telemetry.addLine("\nSquare = Fixed | Cross = Proportional | Circle = Trapezoidal");
            telemetry.addLine("Left/Right Bumper = Change target (P1/P2/P3)");
            telemetry.update();
        }

        // Stop servo on exit
        axleServo.setPower(0);
    }

    // Convert analog voltage → 0–360 degrees
    private double voltageToDegrees(double voltage) {
        return (voltage / VOLTAGE_MAX) * 360.0;
    }

    // Shortest angular error considering wrap-around at 360°
    private double angularError(double targetDeg, double actualDeg) {
        double diff = targetDeg - actualDeg;
        // Normalize to -180..+180
        while (diff > 180) diff -= 360;
        while (diff <= -180) diff += 360;
        return diff;
    }

    private void cycleTarget(int direction) {
        TargetPosition[] values = TargetPosition.values();
        int index = currentTarget.ordinal() + direction;
        if (index < 0) index = values.length - 1;
        if (index >= values.length) index = 0;
        currentTarget = values[index];
    }

    // Mode 1: Fixed ±10% power until in tolerance
    // (can easily overshoot both directions, so much keep low power to avoid oscillations)
    private double fixedPowerControl(double errorDeg) {
        if (Math.abs(errorDeg) <= POSITION_TOLERANCE_DEG) {
            return 0.0;
        }
        return errorDeg > 0 ? 0.10 : -0.10;
    }

    // Mode 2: Simple proportional control
    private double proportionalControl(double errorDeg) {
        if (Math.abs(errorDeg) <= POSITION_TOLERANCE_DEG) {
            return 0.0;
        }
        double rawPower = errorDeg * P_GAIN;
        return Range.clip(rawPower, -MAX_POWER, MAX_POWER);
    }

    // Mode 3: Trapezoidal – proportional but with ramp-down near target
    private double trapezoidalControl(double errorDeg, double errorAbs) {
        if (errorAbs <= POSITION_TOLERANCE_DEG) {
            return 0.0;
        }

        // Scale gain down as we approach target
        double distanceZone = Math.max(ACCEL_DISTANCE_DEG, DECEL_DISTANCE_DEG);
        double scale = 1.0;
        if (errorAbs < distanceZone) {
            scale = Math.max(0.10, errorAbs / distanceZone);  // minimum 10% power
        }

        double rawPower = errorDeg * P_GAIN * scale;

        // Ensure minimum power to overcome stiction
        if (Math.abs(rawPower) < MIN_POWER_TO_MOVE && Math.abs(rawPower) > 0) {
            rawPower = Math.signum(rawPower) * MIN_POWER_TO_MOVE;
        }

        return Range.clip(rawPower, -MAX_POWER, MAX_POWER);
    }
}