package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Test Spindexer PID Control", group = "Test")
public class TestSpindexerPID extends LinearOpMode {

    // Hardware
    private DcMotorEx spindexerMotor = null;

    // Motion profile parameters (tunable)
    private static final double TICKS_PER_DEGREE = 28.0 / 360.0; // Adjust to your motor/gearbox

    // Motion limits (tunable)
    private static final double MAX_VELOCITY = 120.0;     // deg/s
    private static final double MAX_ACCELERATION = 300.0; // deg/s²
    private static final double MAX_JERK = 1000.0;        // deg/s³

    // Profile storage
    private static final int MAX_PROFILE_STEPS = 3000; // Supports up to 3 sec move
    private final double[] spindexerPosProfile = new double[MAX_PROFILE_STEPS];
    private final double[] spindexerVelProfile = new double[MAX_PROFILE_STEPS];
    private double spindexerRotationSeconds;
    private final ElapsedTime profileTimer = new ElapsedTime();
    private boolean isMoving = false;

    // PID control (tunable)
    private double kP = 0.02;
    private double kI = 0.01;
    private double kD = 0.001;
    private double integralError = 0.0;
    private final ElapsedTime pidTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize hardware
        spindexerMotor = hardwareMap.get(DcMotorEx.class, "spindexerMotor");
        spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spindexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized - D-pad: Right=+90°, Left=-90°, Up=0°");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Trigger motion with D-pad (rising edge)
            if (gamepad1.dpad_right && !isMoving) {
                initMotion(90.0);
            } else if (gamepad1.dpad_left && !isMoving) {
                initMotion(-90.0);
            } else if (gamepad1.dpad_up && !isMoving) {
                initMotion(0.0); // Return to zero
            }

            if (isMoving) {
                processSpindexerMotionProfile();
            }

            // Telemetry
            double current = getCurrentAngle();
            telemetry.addData("Current Angle", "%.2f deg", current);
            telemetry.addData("Is Moving", isMoving);
            if (isMoving) {
                double t = profileTimer.seconds();
                int idx = (int) (t / spindexerRotationSeconds * (spindexerPosProfile.length - 1));
                idx = Math.min(idx, spindexerPosProfile.length - 1);
                telemetry.addData("Target Pos", "%.2f deg",     spindexerPosProfile[idx]);
                telemetry.addData("Target Vel", "%.2f deg/sec", spindexerVelProfile[idx]);
            }
            telemetry.update();
        }
    }

    //-------------------------------------------------------------------------------------------------
    private void initMotion(double targetAngle) {
        double v0 = spindexerMotor.getVelocity() / TICKS_PER_DEGREE; // deg/s
        spindexerRotationSeconds = computeSpindexerMotionProfile(
                targetAngle, v0, spindexerPosProfile, spindexerVelProfile);

        profileTimer.reset();
        pidTimer.reset();
        integralError = 0.0;
        isMoving = true;
    } // initSpindexerMotion

    //-------------------------------------------------------------------------------------------------
    // Pre-computes POSITION and VELOCITY values as a function of elapsed movement time for a given 
    // change in Spindexer position (CW or CCW distance from current position) given the current
    // Spindexer velocity v0 (which can be either positive, 0, or negative).  The desired profile is
    // a smooth S-curve ramp-up, coast at max velocity, and then ramp-down to zero at the target angle.
    // NOTE: The no-load Axon MAX+ servo speed is 60deg in 0.115 sec (or 521 deg/sec)
    private double computeSpindexerMotionProfile( double distance, double v0,
                                                  double[] positionProfile, double[] velocityProfile) {
        // Extract direction and use absolute distance for timing
        double direction = Math.signum(distance);
        double absDistance = Math.abs(distance);

        // Phase durations (based on absolute motion)
        double t1 = MAX_ACCELERATION / MAX_JERK;
        double t2 = MAX_VELOCITY / MAX_ACCELERATION;  // the full time
        double tCruise = (absDistance - 2 * MAX_VELOCITY * t2) / MAX_VELOCITY;
        // Check whether the distance to rotate is so short that we never achieve max velocity
        if (tCruise < 0.0 ) {
            tCruise = 0.0;   // can't have negative time; just zero it
            t2 = Math.sqrt(absDistance / MAX_ACCELERATION);  // shorten t2
        }
        // Combine these times into an overall profile duration (time-to-target)
        double profileDuration = 2 * t1 + 2 * t2 + tCruise;
        // At 1000Hz (1 msec) resolution, how many entries will our tables have for this movement?
        int profileSteps = (int) (profileDuration * 1000);
        int stepsToUse = Math.min(profileSteps, MAX_PROFILE_STEPS);

        // Initialize variables for the computation
        double time, pos = 0.0, vel = v0, acc = 0.0;

        // Populate the VELOCITY and POSITION arrays for this movement
        for (int i = 0; i < stepsToUse; i++) {
            time = i * profileDuration / profileSteps;

            if (time < t1) {
                // Jerk-up phase
                acc = direction * MAX_JERK * time;
                vel = v0 + direction * MAX_JERK * time * time / 2;
                pos = v0 * time + direction * MAX_JERK * time * time * time / 6;

            } else if (time < t1 + t2) {
                // Constant acceleration
                double dt = time - t1;
                acc = direction * MAX_ACCELERATION;
                vel = v0 + direction * (MAX_ACCELERATION * dt + MAX_JERK * t1 * t1 / 2);
                pos = v0 * time
                        + direction * (MAX_ACCELERATION * dt * dt / 2
                        + MAX_JERK * t1 * t1 * dt / 2
                        + MAX_JERK * t1 * t1 * t1 / 6);

            } else if (time < t1 + t2 + tCruise) {
                // Cruise phase
                double dt = time - t1 - t2;
                acc = 0.0;
                vel = direction * MAX_VELOCITY;
                pos = v0 * (t1 + t2)
                        + direction * (MAX_VELOCITY * dt
                        + MAX_ACCELERATION * t2 * t2 / 2
                        + MAX_JERK * t1 * t1 * t2 / 2
                        + MAX_JERK * t1 * t1 * t1 / 6);

            } else if (time < t1 + t2 + tCruise + t2) {
                // Constant deceleration
                double dt = time - t1 - t2 - tCruise;
                acc = -direction * MAX_ACCELERATION;
                vel = direction * MAX_VELOCITY - direction * MAX_ACCELERATION * dt;
                pos = v0 * (t1 + t2)
                        + direction * (MAX_VELOCITY * tCruise
                        + MAX_ACCELERATION * t2 * t2 / 2
                        + MAX_JERK * t1 * t1 * t2 / 2
                        + MAX_JERK * t1 * t1 * t1 / 6
                        - MAX_ACCELERATION * dt * dt / 2);

            } else {
                // Jerk-down phase (symmetric)
                double dt_total = time - (t1 + t2 + tCruise + t2);
                acc = -direction * MAX_ACCELERATION - direction * MAX_JERK * dt_total;
                vel = direction * MAX_VELOCITY
                        - direction * MAX_ACCELERATION * t2
                        - direction * MAX_JERK * dt_total * dt_total / 2;
                // Final position approximation (smooth enough)
                pos = distance - direction * vel * (profileDuration - time) / 3;
            }
            // Store the computed values in our arrays
            positionProfile[i] = pos;
            velocityProfile[i] = vel;
        } // profileSteps

        return profileDuration;  // number of seconds
    } // computeSpindexerMotionProfile

 //-------------------------------------------------------------------------------------------------
    private void processSpindexerMotionProfile() {
        // Where are we in the motion profile?
        double time = profileTimer.seconds();
        // Are we done?
        if (time >= spindexerRotationSeconds) {
            isMoving = false;
            integralError = 0.0;
            spindexerMotor.setPower(0);
            return;
        }

        // Smooth index lookup
        int index = (int) (time / spindexerRotationSeconds * (spindexerPosProfile.length - 1));
        index = Math.min(index, spindexerPosProfile.length - 1);

        double targetPos = spindexerPosProfile[index];
        double targetVel = spindexerVelProfile[index];

        double currentPos = getCurrentAngle();
        double currentVel = spindexerMotor.getVelocity() / TICKS_PER_DEGREE;

        double error = targetPos - currentPos;
        double dt = pidTimer.seconds();
        pidTimer.reset();

        integralError += error * dt;
        double derivative = targetVel - currentVel;

        double output = kP * error + kI * integralError + kD * derivative;
        output = Range.clip(output, -1.0, 1.0);
        // Set motor power
        spindexerMotor.setPower(output);
    } // processSpindexerMotionProfile

    //-------------------------------------------------------------------------------------------------
    private double getCurrentAngle() {
        return spindexerMotor.getCurrentPosition() / TICKS_PER_DEGREE;
    }
    
} // TestSpindexerPID