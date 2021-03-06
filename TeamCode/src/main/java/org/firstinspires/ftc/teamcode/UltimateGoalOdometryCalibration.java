package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HelperClasses.WayPoint;
import org.firstinspires.ftc.teamcode.RobotUtilities.MyPosition;

import static java.lang.Math.atan2;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

/**
 * Created by 7592 Roarbots.
 */

//@TeleOp(name="UltimateGoal: OdometryCalTeleOp", group ="TeleOp")
public class UltimateGoalOdometryCalibration extends OpMode {

    public UltimateGoalRobot robot = new UltimateGoalRobot();
    protected boolean aligning = false;

    @Override
    public void init() {
        telemetry.addLine("Calling robot.init");
        updateTelemetry(telemetry);
        robot.init(hardwareMap);
//        robot.disableDriveEncoders();
        robot.setShooterFlapPowerShot();
        robot.setInputShaping(true);
        // Let's try to tweak the PIDs
        robot.shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400,
                1, 1, 0, MotorControlAlgorithm.PIDF));
        telemetry.addLine("Ready");
        updateTelemetry(telemetry);
    }

    @Override
    public void init_loop() {
        telemetry.addLine("Press X on driver controller to reset encoders.");
        telemetry.addLine("Press O on driver controller to disable Driver Centric Mode.");
        if(gamepad1.cross) {
            robot.forceReset = true;
        }
        if(gamepad1.circle) {
            robot.disableDriverCentric = true;
        }
    }

    private WayPoint calibrationTarget = new WayPoint(0, 150.0, toRadians(90.0), 1.0);
    private double driverAngle = 0.0;
    private final double MAX_SPEED = 1.0;
    private final double MAX_SPIN = 1.0;
    private double speedMultiplier = MAX_SPEED;
    private double spinMultiplier = MAX_SPIN;
    private boolean crossHeld = false;
    private boolean circleHeld = false;
    private boolean triangleHeld = false;
    private boolean upHeld = false;
    private boolean downHeld = false;
    private boolean leftHeld = false;
    private boolean rightHeld = false;
    private boolean leftBumperHeld = false;
    private boolean rightBumperHeld = false;
    private boolean cross2Held = false;
    private boolean circle2Held = false;
    private boolean triangle2Held = false;
    private boolean square2Held = false;
    private boolean up2Held = false;
    private boolean down2Held = false;
    private boolean left2Held = false;
    private boolean right2Held = false;
    private boolean leftBumper2Held = false;
    private boolean rightBumper2Held = false;
    private boolean crossPressed;
    private boolean circlePressed;
    private boolean trianglePressed;
    private boolean leftPressed;
    private boolean rightPressed;
    private boolean leftBumperPressed;
    private boolean rightBumperPressed;
    private boolean upPressed;
    private boolean downPressed;
    private boolean cross2Pressed;
    private boolean circle2Pressed;
    private boolean triangle2Pressed;
    private boolean square2Pressed;
    private boolean up2Pressed;
    private boolean down2Pressed;
    private boolean left2Pressed;
    private boolean right2Pressed;
    private boolean leftBumper2Pressed;
    private boolean rightBumper2Pressed;
    private boolean fingersUp = true;
    private double rightTriggerPower;
    private double leftTriggerPower;
    private double yPower;
    private double xPower;
    private double spin;
    private ElapsedTime loopTime = new ElapsedTime();
    private boolean runWithEncoders = true;
    protected double forwardPower = 0.0;
    protected double strafePower = 0.0;

    @Override
    public void start() {
        robot.resetEncoders();

        //give MyPosition our current positions so that it saves the last positions of the wheels
        //this means we won't teleport when we start the match. Just in case, run this twice
        for(int i = 0; i < 2 ; i ++) {
            robot.resetReads();
            MyPosition.initialize(robot.getLeftEncoderWheelPosition(),
                    robot.getRightEncoderWheelPosition(),
                    robot.getStrafeEncoderWheelPosition());
        }
        // Why do I have to negative angle to get the right angle!?! Everything works that way
        // but weird.
//        startLocation = new WayPoint(149.7584, 22.86, Math.toRadians(90.0), 0.0);
        MyPosition.setPosition(0, 0, toRadians(90.0));
    }

    @Override
    public void loop() {
        loopTime.reset();
        // Allow the robot to read sensors again
        robot.resetReads();
        MyPosition.giveMePositions(robot.getLeftEncoderWheelPosition(),
                robot.getRightEncoderWheelPosition(),
                robot.getStrafeEncoderWheelPosition());

        //left joystick is for moving
        //right joystick is for rotation
        yPower = -gamepad1.left_stick_y;
        xPower = gamepad1.left_stick_x;
        leftTriggerPower = gamepad1.left_trigger;
        rightTriggerPower = gamepad1.right_trigger;
//        yPower = -HardwareOmnibot.cleanMotionValues(gamepad1.left_stick_y);
//        xPower = HardwareOmnibot.cleanMotionValues(gamepad1.left_stick_x);
        // GF used the angle system where rotating left was positive, so have to reverse the sign
        // of the joystick.
        spin = gamepad1.right_stick_x;
//        spin = -HardwareOmnibot.cleanMotionValues(gamepad1.right_stick_x);
        crossPressed = gamepad1.cross;
        circlePressed = gamepad1.circle;
        trianglePressed = gamepad1.triangle;
		rightPressed = gamepad1.dpad_right;
		leftPressed = gamepad1.dpad_left;
		leftBumperPressed = gamepad1.left_bumper;
        rightBumperPressed = gamepad1.right_bumper;
        upPressed = gamepad1.dpad_up;
        downPressed = gamepad1.dpad_down;
        cross2Pressed = gamepad2.cross;
        circle2Pressed = gamepad2.circle;
        triangle2Pressed = gamepad2.triangle;
        square2Pressed = gamepad2.square;
        up2Pressed = gamepad2.dpad_up;
        down2Pressed = gamepad2.dpad_down;
        right2Pressed = gamepad2.dpad_right;
        left2Pressed = gamepad2.dpad_left;
        leftBumper2Pressed = gamepad2.left_bumper;
        rightBumper2Pressed = gamepad2.right_bumper;

        if (gamepad1.square) {
            // The driver presses Square, then uses the left joystick to say what angle the robot
            // is aiming.  This will calculate the values as long as square is pressed, and will
            // not drive the robot using the left stick.  Once square is released, it will use the
            // final calculated angle and drive with the left stick.  Button should be released
            // before stick.  The default behavior of atan2 is 0 to -180 on Y Axis CCW, and 0 to
            // 180 CW.  This code normalizes that to 0 to 360 CCW from the Y Axis
            //robot.resetGyro();
            // This was -90, but needed to be oriented 90 degrees from actual angle.
            driverAngle = toDegrees(atan2(yPower, xPower))- 90.0 - robot.readIMU();
            xPower = 0.0;
            yPower = 0.0;
            spin = 0.0;
        }

		// ********************************************************************
		// DRIVER JOYSTICK
		// ********************************************************************
        if(!circleHeld && circlePressed) {
            circleHeld = true;
        } else if(!circlePressed) {
            circleHeld = false;
        }

        if(!triangleHeld && trianglePressed) {
            triangleHeld = true;
        } else if(!trianglePressed) {
            triangleHeld = false;
        }

        if(!crossHeld && crossPressed) {
            if(robot.shooterMotorTargetVelocity == 0) {
                robot.shooterOnHighGoal();
            } else {
                robot.shooterOff();
            }
            crossHeld = true;
        } else if(!crossPressed) {
            crossHeld = false;
        }

        // This can be used for shoot alignment.
        if(!rightHeld && rightPressed) {
            if(!aligning) {
                calibrationTarget.x = 50.0;
                calibrationTarget.y = 50.0;
                calibrationTarget.angle = Math.toRadians(180.0);
                calibrationTarget.speed = 1.0;
                robot.startShotAligning(calibrationTarget, false);
                aligning = true;
            } else {
                aligning = false;
                robot.stopShotAligning();
            }
            rightHeld = true;
        } else if(!rightPressed) {
            rightHeld = false;
        }

        if(!leftHeld && leftPressed) {
            if(!aligning) {
                calibrationTarget.x = 0.0;
                calibrationTarget.y = 0.0;
                calibrationTarget.angle = Math.toRadians(90.0);
                calibrationTarget.speed = 1.0;
                robot.startShotAligning(calibrationTarget, false);
                aligning = true;
            } else {
                aligning = false;
                robot.stopShotAligning();
            }
            leftHeld = true;
        } else if(!leftPressed) {
            leftHeld = false;
        }

        if(!downHeld && downPressed) {
            if(!aligning) {
                calibrationTarget.x = 0.0;
                calibrationTarget.y = 0.0;
                calibrationTarget.angle = Math.toRadians(90.0);
                calibrationTarget.speed = 1.0;
                robot.startShotAligning(calibrationTarget, false);
                aligning = true;
            } else {
                aligning = false;
                robot.stopShotAligning();
            }
            downHeld = true;
        } else if (!downPressed) {
            downHeld = false;
        }

        if(!upHeld && upPressed) {
            if(!aligning) {
                calibrationTarget.x = 0.0;
                calibrationTarget.y = 150.0;
                calibrationTarget.angle = Math.toRadians(90.0);
                calibrationTarget.speed = 1.0;
                robot.startShotAligning(calibrationTarget, false);
                aligning = true;
            } else {
                aligning = false;
                robot.stopShotAligning();
            }
            upHeld = true;
        } else if (!upPressed) {
            upHeld = false;
        }

        if(!rightBumperHeld && rightBumperPressed) {
            rightBumperHeld = true;
        } else if(!rightBumperPressed) {
            rightBumperHeld = false;
        }

        if(!leftBumperHeld && leftBumperPressed) {
            leftBumperHeld = true;
        } else if(!leftBumperPressed) {
            leftBumperHeld = false;
        }

        // This allows to reset claw, remove!!!
//        if(robot.disableDriverCentric) {
            if (rightTriggerPower >= 0.05) {
                robot.setWobbleMotorPower(-rightTriggerPower);
            } else if (leftTriggerPower >= 0.05) {
                robot.setWobbleMotorPower(leftTriggerPower);
            } else {
                robot.setWobbleMotorPower(0.0);
            }
  //      }

		// ********************************************************************
		// OPERATOR JOYSTICK
		// ********************************************************************
		// This was unassigned (fingers up/down)
        if(!square2Held && square2Pressed) {
            square2Held = true;
        } else if(!square2Pressed) {
            square2Held = false;
        }

        if(!cross2Held && cross2Pressed) {
            robot.injector.setPosition(UltimateGoalRobot.INJECTOR_FIRE);
            cross2Held = true;
        } else if(!cross2Pressed) {
            cross2Held = false;
        }

        if(!circle2Held && circle2Pressed) {
            robot.injector.setPosition(UltimateGoalRobot.INJECTOR_HOME);
            circle2Held = true;
        } else if(!circle2Pressed) {
            circle2Held = false;
        }

        if(!triangle2Held && triangle2Pressed) {
            robot.injector.setPosition(UltimateGoalRobot.INJECTOR_RESET);
            triangle2Held = true;
        } else if(!triangle2Pressed) {
            triangle2Held = false;
        }

        if(!left2Held && left2Pressed) {
            left2Held = true;
        } else if (!left2Pressed) {
            left2Held = false;
        }

        if(!right2Held && right2Pressed) {
            robot.injector.setPosition(UltimateGoalRobot.INJECTOR_HOME);
            right2Held = true;
        } else if (!right2Pressed) {
            right2Held = false;
        }

        if(!up2Held && up2Pressed) {
            robot.injector.setPosition(UltimateGoalRobot.INJECTOR_RESET);
            up2Held = true;
        } else if (!up2Pressed) {
			up2Held = false;
		}

        if(!down2Held && down2Pressed) {
            robot.injector.setPosition(UltimateGoalRobot.INJECTOR_FIRE);
            down2Held = true;
        } else if (!down2Pressed) {
			down2Held = false;
		}

        if(!rightBumper2Held && rightBumper2Pressed) {
            rightBumper2Held = true;
        } else if(!rightBumper2Pressed) {
            rightBumper2Held = false;
        }

        if(!leftBumper2Held && leftBumper2Pressed) {
            leftBumper2Held = true;
        } else if(!leftBumper2Pressed) {
            leftBumper2Held = false;
        }

        // If the activity is not performing, it will be idle and return.
        performActivities();
        if(robot.shotAlignmentState == UltimateGoalRobot.SHOT_ALIGNMENT_STATE.IDLE) {
            aligning = false;
            robot.drive(speedMultiplier * xPower, speedMultiplier * yPower,
                    spinMultiplier * spin, driverAngle - 90.0, robot.defaultInputShaping);
        }

        telemetry.addData( "Offset Angle: ", driverAngle);
        telemetry.addData("Flap Position: ", robot.flapAngle);
        telemetry.addData("Shooter Velocity: ", robot.shooter.getVelocity());
        telemetry.addData("Shooter Stability: ", robot.sequentialStableVelocityChecks);
        telemetry.addData("FL Motor Velocity: ", robot.frontLeft.getVelocity());
        telemetry.addData("FR Motor Velocity: ", robot.frontRight.getVelocity());
        telemetry.addData("RL Motor Velocity: ", robot.rearLeft.getVelocity());
        telemetry.addData("RR Motor Velocity: ", robot.rearRight.getVelocity());
        telemetry.addData("Left Encoder: ", robot.getLeftEncoderWheelPosition());
        telemetry.addData("Strafe Encoder: ", robot.getStrafeEncoderWheelPosition());
        telemetry.addData("Right Encoder: ", robot.getRightEncoderWheelPosition());
        telemetry.addData("Forward Power: ", forwardPower);
        telemetry.addData("Strafe Power: ", strafePower);
        telemetry.addData("Spin: ", spin);
        // This prevents us from reading the IMU every loop if we aren't in driver centric.
        if(!robot.disableDriverCentric) {
            telemetry.addData("Gyro Angle: ", robot.readIMU());
        }
        telemetry.addData("World X Position: ", MyPosition.worldXPosition);
        telemetry.addData("World Y Position: ", MyPosition.worldYPosition);
        telemetry.addData("World Angle: ", Math.toDegrees(MyPosition.worldAngle_rad));
        telemetry.addData("Loop time: ", loopTime.milliseconds());
        updateTelemetry(telemetry);
    }

    @Override
    public void stop() {
//        robot.stopGroundEffects();
    }
    protected void performActivities() {
        robot.performClawToggle();
        robot.performInjecting();
        robot.performTripleInjecting();
        robot.performShotAligning();
        robot.performRotatingArm();
        robot.updateShooterStability();
    }
}
