/* FTC Team 7572 - Version 1.0 (09/05/2025)
*/
package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.HardwareSwyftBot.SpindexerState.SPIN_DECREMENT;
import static org.firstinspires.ftc.teamcode.HardwareSwyftBot.SpindexerState.SPIN_INCREMENT;
import static org.firstinspires.ftc.teamcode.HardwareSwyftBot.SpindexerState.SPIN_P1;
import static org.firstinspires.ftc.teamcode.HardwareSwyftBot.SpindexerState.SPIN_P3;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * TeleOp for the 2025-2026 FTC DECODE Season
 */
//@Disabled
public abstract class Teleop extends LinearOpMode {
    double  yTranslation, xTranslation, rotation;                  /* Driver control inputs */
    double  rearLeft, rearRight, frontLeft, frontRight, maxPower;  /* Motor power levels */
    boolean backwardDriveControl = false; // drive controls backward (other end of robot becomes "FRONT")
    boolean controlMultSegLinear = true;

    double  shooterPower = 0.55;  // far shooting default. scale for location.
    double  odoShootDistance = 0.0;
    double  odoShootAngleDeg = 0.0;
    boolean isAutoShooterAngleGood = false; // false if the robot facing too far away from the target
    boolean isAutoShooterSpeedGood = false; // is shooter motor up to target speed
    boolean autoAimEnabled         = false; // wait until we move to avoid early PLAY button movement penalty
    // FIXME: enter correct min distance value.
    double MIN_SHOOT_DISTANCE_INCHES = 30; // minimum distance required to shoot

    boolean leftTriggerPressNow   = false;
    boolean leftTriggerPressLast  = false;
    boolean rightTriggerPressNow  = false;
    boolean rightTriggerPressLast = false;

    boolean blueAlliance;   // set in the Blue/Red
    boolean farAlliance;    //
    int     aprilTagGoal;
    boolean showApriltagTargetData = true;

    final int DRIVER_MODE_SINGLE_WHEEL = 1;
    final int DRIVER_MODE_STANDARD     = 2;
    final int DRIVER_MODE_DRV_CENTRIC  = 3;
    int       driverMode               = DRIVER_MODE_STANDARD;
    double    driverAngle              = 0.0;  /* for DRIVER_MODE_DRV_CENTRIC */

    boolean enableOdometry   = true;
    boolean intakeMotorOnFwd = false;
    boolean intakeMotorOnRev = false;
    boolean shooterMotorsOn  = false;

    boolean newPinpointFieldPositionUpdate = false;  // ensure we only update once per cycle

    Gamepad.RumbleEffect tooCloseRumbleLR;    // Too close to shoot!
    Gamepad.RumbleEffect spindexerRumbleL;    // Can't spin further LEFT!
    Gamepad.RumbleEffect spindexerRumbleR;    // Can't spin further RIGHT!
    Gamepad.RumbleEffect odoUpdateRumble;

    long      nanoTimeCurr=0, nanoTimePrev=0;
    double    cycleTimeElapsed, cycleTimeHz;

    double    limelightTimestampCurr=0, limelightTimestampPrev=0;
    double    limelightUpdateElapsed=0, limelightUpdateHz=0;

    /* Declare OpMode members. */
    HardwareSwyftBot robot = new HardwareSwyftBot();
    // sets unique behavior based on alliance
    public abstract void setAllianceSpecificBehavior();

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("State", "Initializing (please wait)");
        telemetry.update();

        tooCloseRumbleLR = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 300)  //  Rumble BOTH motor 100% for 300 mSec
                .addStep(0.0, 0.0, 250)  //  Pause for 250 mSec
                .addStep(1.0, 1.0, 300)  //  Rumble BOTH motor 100% for 300 mSec
                .build();

        spindexerRumbleL = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 0.0, 250)  //  Rumble LEFT motor 100% for 250 mSec
                .build();

        spindexerRumbleR = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 250)  //  Rumble RIGHT motor 100% for 250 mSec
                .build();

        odoUpdateRumble = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 0.0, 250)  //  Rumble LEFT motor 100% for 250 mSec
                .addStep(0.0, 1.0, 250)  //  Rumble RIGHT motor 100% for 250 mSec
                .build();
                
        // Initialize robot hardware (not autonomous mode)
        robot.init(hardwareMap,false);
        robot.limelightStart();
//      llodo = new LimelightFusedPinpointOdometry(robot.limelight, robot.odom, telemetry, 0.0);
        // Establish whether this is the RED or BLUE alliance
        setAllianceSpecificBehavior();
        // limelight pipelines 6 & 7 filter for the BLUE and RED goal apriltags
        robot.limelightPipelineSwitch( (blueAlliance)? 6:7 );

        // Initialize driver centric angle based on the alliance color
        driverMode  = DRIVER_MODE_DRV_CENTRIC;
        driverAngle = (blueAlliance)? +90.0 : -90.0;  // assumes auto ended from BLUE-FAR or RED-FAR

        // Wait for the game to start (driver presses PLAY)
        while (!isStarted()) {
            // Send telemetry message to signify robot waiting;
            telemetry.addData("State", "Ready");
            telemetry.addLine("Press X (cross) to reset encoders");
            telemetry.addLine("(to run Teleop without Auto first)");
            // Bulk-refresh the hub data and updates our state machines (spindexer!)
            performEveryLoopTeleop();
            telemetry.addData("Limelight","x=%.2f y=%.2f  %.2f deg (Apriltag)",
                    robot.limelightFieldXpos, robot.limelightFieldYpos, robot.limelightFieldAngleDeg );
            telemetry.addData("  stdev","%.5f %.5f  %.5f",
                    robot.limelightFieldXstd, robot.limelightFieldYstd, robot.limelightFieldAnglestd );
            telemetry.addData("Pinpoint","x=%.2f y=%.2f  %.2f deg (odom)",
                    robot.robotGlobalXCoordinatePosition, robot.robotGlobalYCoordinatePosition, robot.robotOrientationDegrees );
            telemetry.addData(" "," %.2f in/sec %.2f in/sec %.2f deg/sec",
                    robot.robotGlobalXvelocity, robot.robotGlobalYvelocity, robot.robotAngleVelocity );
            telemetry.update();
            // Normally autonomous resets encoders/odometry.  Do we need to for teleop??
            if( gamepad1.crossWasPressed() ) {
                robot.resetEncoders();
                robot.resetGlobalCoordinatePosition( 0.0, 0.0, 0.0 );
                autoAimEnabled = true;   // implies teleop without autonomous first (
            }
            // Pause briefly before looping
            idle();
        } // !isStarted

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            // Bulk-refresh the hub data and updates our state machines
            performEveryLoopTeleop();

            // Check for an OFF-to-ON toggle of the gamepad1 TRIANGLE button (toggles SINGLE-MOTOR drive control)
//          if( gamepad1_triangle_now && !gamepad1_triangle_last)
//          {
//              driverMode = DRIVER_MODE_SINGLE_WHEEL; // allow control of individual drive motors
//          }

            // Check for an OFF-to-ON toggle of the gamepad1 SQUARE button (toggles DRIVER-CENTRIC drive control)
            if( gamepad1.squareWasPressed() )
            {
                driverMode = DRIVER_MODE_DRV_CENTRIC;
            }

            // Check for an OFF-to-ON toggle of the gamepad1 CIRCLE button (toggles STANDARD/BACKWARD drive control)
            if( gamepad1.circleWasPressed() )
            {
                // If currently in DRIVER-CENTRIC mode, switch to STANDARD (robot-centric) mode
                if( driverMode != DRIVER_MODE_STANDARD ) {
                    driverMode = DRIVER_MODE_STANDARD;
                    backwardDriveControl = false;  // start with phone-end as front of robot
                }
                // Already in STANDARD mode; Just toggle forward/backward mode
                else {
                    backwardDriveControl = !backwardDriveControl; // reverses which end of robot is "FRONT"
                }
            }

//          telemetry.addData("cross","Toggle Intake");
//          telemetry.addData("circle","Robot-centric (fwd/back modes)");
//          telemetry.addData("square","Driver-centric (set joystick!)");
//          telemetry.addData("d-pad","Fine control 15%)");

            if( processDpadDriveMode() == false ) {
                // Control based on joystick; report the sensed values
//              telemetry.addData("Joystick1", "x=%.3f, y=%.3f spin=%.3f",
//                      gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x );
//              telemetry.addData("Joystick2", "pan=%.3f, tilt=%.3f extend=%.3f",
//                      gamepad2.left_stick_x, -gamepad2.left_stick_y, gamepad2.right_stick_y );
                switch( driverMode ) {
                    case DRIVER_MODE_SINGLE_WHEEL :
                       telemetry.addData("Driver Mode", "SINGLE-WHEEL (tri)" );
                       processSingleWheelControl();
                       break;
                    case DRIVER_MODE_STANDARD :
                        telemetry.addData("Driver Mode", "STD-%s (cir)",
                                (backwardDriveControl)? "BACKWARD":"FORWARD" );
                        processStandardDriveMode();
                        break;
                    case DRIVER_MODE_DRV_CENTRIC :
                        telemetry.addData("Driver Mode", "DRIVER-CENTRIC (sq)" );
                        processDriverCentricDriveMode();
                        break;
                    default :
                        // should never happen; reset to standard drive mode
                        driverMode = DRIVER_MODE_STANDARD;
                        break;
                } // switch()
            } // processDpadDriveMode

            updateLimelightPinpointOffset();

            processCollector();
            processTurretAutoAim();
            processSpindexer();
            processShooter();
            processInjector();

            // Compute current cycle time
            nanoTimePrev = nanoTimeCurr;
            nanoTimeCurr = System.nanoTime();
            cycleTimeElapsed = (nanoTimeCurr - nanoTimePrev)/ 1000000.0;   // msec
            cycleTimeHz =  1000.0 / cycleTimeElapsed;

            // Compute limelight update rate
            limelightTimestampPrev = limelightTimestampCurr;
            limelightTimestampCurr = robot.limelightTimestamp;
            if( (limelightTimestampCurr > 0) && (limelightTimestampPrev > 0) &&
                (limelightTimestampCurr > limelightTimestampPrev)) {
                limelightUpdateElapsed = (limelightTimestampCurr - limelightTimestampPrev) * 1000.0;  // convert to msec
                limelightUpdateHz = 1000.0 / limelightUpdateElapsed;
            }

            // Update telemetry data
            telemetry.addData("Limelight","x=%.2f y=%.2f  %.2f deg (Apriltag)",
                    robot.limelightFieldXpos, robot.limelightFieldYpos, robot.limelightFieldAngleDeg );
            telemetry.addData("  stdev","%.5f %.5f  %.5f",
                    robot.limelightFieldXstd, robot.limelightFieldYstd, robot.limelightFieldAnglestd );
            telemetry.addData("Pinpoint","x=%.2f y=%.2f  %.2f deg (odom)",
                   robot.robotGlobalXCoordinatePosition, robot.robotGlobalYCoordinatePosition, robot.robotOrientationDegrees );
            telemetry.addData(" "," %.2f in/sec %.2f in/sec %.2f deg/sec",
                   robot.robotGlobalXvelocity, robot.robotGlobalYvelocity, robot.robotAngleVelocity );
            // Show limelight-to-pinpoint offset (press triangle to apply)
            // Display X and Y offsets with their individual confidence values
            if( robot.limelightPinpointOffsetXvalid || robot.limelightPinpointOffsetYvalid ) {
                String xInfo = robot.limelightPinpointOffsetXvalid ?
                    String.format("X=%.2f(%.5f)", robot.limelightPinpointOffsetX, robot.limelightPinpointOffsetXconfidence) : "X=(none)";
                String yInfo = robot.limelightPinpointOffsetYvalid ?
                    String.format("Y=%.2f(%.5f)", robot.limelightPinpointOffsetY, robot.limelightPinpointOffsetYconfidence) : "Y=(none)";
                telemetry.addData("LL Offset", "%s %s", xInfo, yInfo);
                telemetry.addData(" ","(press TRI to apply)");
            } else {
                telemetry.addData("LL Offset","(none - stop near apriltag)");
            }
            telemetry.addData("Goal", "%s dist: %.2f in, angle: %.2f deg", ((blueAlliance)? "BLUE":"RED"), odoShootDistance, odoShootAngleDeg);
//          telemetry.addData("Shooter POWER", "%.3f (P1 tri/cross to adjust)", shooterPower);
//          if(robot.shooterMotorsReady) {
//              telemetry.addData("Shooter ms to ready ", "%.1f", robot.shooterMotorsTime);
//          } else {
//              telemetry.addData("Shooter Target Velocity", "%.1f", robot.shooterTargetVel );
//          }
//          telemetry.addData("Shooter Velocity", "%.1f %.1f", robot.shooterMotor1Vel, robot.shooterMotor2Vel );
//          telemetry.addData("Shooter mA", "%.1f %.1f", robot.shooterMotor1Amps, robot.shooterMotor2Amps );
//          telemetry.addData("Turret", "set %.3f get %.3f analog %.3f", robot.turretServoSet, robot.turretServoGet, robot.turretServoPos );
//          telemetry.addData(" ", "in position: %s autoAim: %s",
//                  ((robot.turretServoIsBusy)? "no":"YES"), ((autoAimEnabled)?"ON":"off") );
//          telemetry.addData(" manual offset", "%.1f deg", robot.turretManualOffset);
//          telemetry.addData("Spindexer", "set=%.2f get=%.2f time=%.0f msec",
//                  robot.spinServoSetPos, robot.getSpindexerPos(), robot.spinServoTime );
//          telemetry.addData(" ", "delta=%.3f InPos=%s timeout=%.0f msec",
//                  robot.spinServoDelta, ((robot.spinServoInPos)? "YES":"no"), robot.spinServoTimeout );
            telemetry.addData("Spindexer ",robot.spinServoCurPos  );
//          telemetry.addData("Triple-shoot time","%.0f msec", robot.shoot3Time  );
//          telemetry.addData("Driver Angle", "%.3f deg", driverAngle );
//          telemetry.addData("IMU Angle", "%.3f deg", robot.headingIMU() );
//          telemetry.addData("Driver Centric", "%.3f deg", (driverAngle - robot.headingIMU()) );
            telemetry.addData( "Presence ", "Left: %d Right: %d",
                    (robot.leftBallIsPresent)? 1:0 ,(robot.rightBallIsPresent)? 1 : 0);
            telemetry.addData( "Color Hue", "Left: %.1f Right: %.1f",
                    robot.readColorSensor(robot.leftBallColorSensor) ,robot.readColorSensor(robot.rightBallColorSensor));
            telemetry.addData("Spinventory", "Spindex: %d SpindexL: %d SpindexR: %d SpindexC:%d",
                    robot.spindex, robot.spindexerLeft, robot.spindexerRight, robot.spindexerCenter );
            telemetry.addData("Spinventory", "Hue: %.1f LeftHue: %.1f RightHue: %.1f",
                    robot.ballHueDetected, robot.leftBallHueDetected, robot.rightBallHueDetected );
            telemetry.addData("Spinventory", "Left: %s Center: %s Right: %s",
                robot.getLeftBall(), robot.getCenterBall(), robot.getRightBall() );
            telemetry.addLine( (robot.isRobot2)? "Robot2" : "Robot1");
            telemetry.addData("CycleTime", "%.1f msec (%.1f Hz)", cycleTimeElapsed, cycleTimeHz);
            telemetry.addData("Limelight Rate", "%.1f msec (%.1f Hz)", limelightUpdateElapsed, limelightUpdateHz);
            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
//          robot.waitForTick(40);
        } // opModeIsActive

//  robot.spinServoCR.setPower(0.0);  // only for spinServoCR (not currently used)
    } // runOpMode

    /*---------------------------------------------------------------------------------*/
    void performEveryLoopTeleop() {
        robot.readBulkData();
        isAutoShooterSpeedGood = robot.shooterMotorsReady;
        robot.processSpindexerMovement();
        robot.processInjectionStateMachine();
        robot.processTripleShotStateMachine();
        robot.processColorDetection();
//      robot.autoSpindexIfAppropriate(); // TEMPORARY DEBUG
        if( enableOdometry ) {
            robot.updatePinpointFieldPosition();
            robot.updateLimelightFieldPosition();
        } // enableOdometry
        // Did we start with a non-zero angle and need to reset?
        if( gamepad1.touchpadWasPressed() ){
            // Ensure robot is aligned to 0deg before pressing!
            // (x,y location less critical; can be updated via AprilTag)
            robot.resetGlobalCoordinatePosition( 0.0, 0.0, 0.0 );
        }
        // Apply the limelight-to-pinpoint offset correction
        if( gamepad1.triangleWasPressed() ) {
            if( robot.applyLimelightPinpointOffset() ) {
                gamepad1.runRumbleEffect(odoUpdateRumble);  // notify driver offset was applied
            }
        }
    } // performEveryLoopTeleop

    /*---------------------------------------------------------------------------------*/
    // used to dynamically update the pinpoint odometry using AprilTag on the goal
    void updatePinpointFieldPosition0() {
        // Ensure we don't get a spurious zero/clear reading
        boolean canSeeAprilTag = (robot.limelightFieldXpos != 0.0) && (robot.limelightFieldYpos !=0.0);
        boolean qualityReading = (robot.limelightFieldXstd <= 0.0022) && (robot.limelightFieldYstd <= 0.0027);
        boolean robotXslow = (Math.abs(robot.robotGlobalXvelocity) < 0.1)? true:false;
        boolean robotYslow = (Math.abs(robot.robotGlobalYvelocity) < 0.1)? true:false;
        boolean robotAslow = (Math.abs(robot.robotAngleVelocity)   < 0.1)? true:false;
        boolean notDriving = (robotXslow && robotYslow && robotAslow)?     true:false;
        if( canSeeAprilTag && qualityReading && notDriving ) {
            // We meet the conditions, but only want to update this once (not over and over and over)
            if( !newPinpointFieldPositionUpdate ) {
                robot.setPinpointFieldPosition(robot.limelightFieldXpos, robot.limelightFieldYpos);
                robot.turretManualOffset = 0.0; // we've updated; reset manual offset to zero
                gamepad1.runRumbleEffect(spindexerRumbleL);  // notify driver it's happening
                newPinpointFieldPositionUpdate = true;
            }
        } else {  // we don't meet the conditions anymore; reset for next time we do
            newPinpointFieldPositionUpdate = false;
        }
    }  // updatePinpointFieldPosition0

    /*---------------------------------------------------------------------------------*/
    // Calculate the offset between limelight and pinpoint (doesn't apply it)
    // The offset will be applied when the user presses a button
    void updateLimelightPinpointOffset() {
        // Ensure we don't get a spurious zero/clear reading
        boolean canSeeAprilTag = (robot.limelightFieldXpos != 0.0) && (robot.limelightFieldYpos !=0.0);
        boolean robotXslow = (Math.abs(robot.robotGlobalXvelocity) < 0.1)? true:false;
        boolean robotYslow = (Math.abs(robot.robotGlobalYvelocity) < 0.1)? true:false;
        boolean robotAslow = (Math.abs(robot.robotAngleVelocity)   < 0.1)? true:false;
        boolean notDriving = (robotXslow && robotYslow && robotAslow)?     true:false;

        if( canSeeAprilTag && notDriving ) {
            // Calculate the offset between limelight and pinpoint positions
            // This doesn't apply it yet - just tracks the difference
            if( robot.updateLimelightPinpointOffsets() && !newPinpointFieldPositionUpdate ) {
                gamepad1.runRumbleEffect(odoUpdateRumble);  // notify driver offset is ready
                newPinpointFieldPositionUpdate = true;
            }
        } else {  // we don't meet the conditions anymore; reset flag for next reading
            newPinpointFieldPositionUpdate = false;
        }
    }  // updateLimelightPinpointOffset

    /*---------------------------------------------------------------------------------*/
    /*  TELE-OP: Mecanum-wheel drive control using Dpad (slow/fine-adjustment mode)    */
    /*---------------------------------------------------------------------------------*/
    boolean processDpadDriveMode() {
        double fineControlSpeed = 0.15;
        boolean dPadMode = true;
        // Only process 1 Dpad button at a time
        if( gamepad1.dpad_up ) {
            telemetry.addData("Dpad","FORWARD");
            frontLeft  = fineControlSpeed;
            frontRight = fineControlSpeed;
            rearLeft   = fineControlSpeed;
            rearRight  = fineControlSpeed;
        }
        else if( gamepad1.dpad_down ) {
            telemetry.addData("Dpad","BACKWARD");
            frontLeft  = -fineControlSpeed;
            frontRight = -fineControlSpeed;
            rearLeft   = -fineControlSpeed;
            rearRight  = -fineControlSpeed;
        }
        else if( gamepad1.dpad_left ) {
            telemetry.addData("Dpad","LEFT");
            frontLeft  = -fineControlSpeed;
            frontRight =  fineControlSpeed;
            rearLeft   =  fineControlSpeed;
            rearRight  = -fineControlSpeed;
        }
        else if( gamepad1.dpad_right ) {
            telemetry.addData("Dpad","RIGHT");
            frontLeft  =  fineControlSpeed;
            frontRight = -fineControlSpeed;
            rearLeft   = -fineControlSpeed;
            rearRight  =  fineControlSpeed;
        }
        else {
            dPadMode = false;
        }
        if( dPadMode ) {
            robot.driveTrainMotors( frontLeft, frontRight, rearLeft, rearRight);
        }
        return dPadMode;
    } // processDpadDriveMode

    private double minThreshold( double valueIn ) {
        double valueOut;

        //========= NO/MINIMAL JOYSTICK INPUT =========
        if( Math.abs( valueIn) < 0.02 ) {
            valueOut = 0.0;
        }
        else {
            valueOut = valueIn;
        }
        return valueOut;
    } // minThreshold

    private double multSegLinearRot( double valueIn ) {
        double valueOut;

        //========= NO JOYSTICK INPUT =========
        if( Math.abs( valueIn) < 0.05 ) {
            valueOut = 0.0;
        }
        //========= POSITIVE JOYSTICK INPUTS =========
        else if( valueIn > 0.0 ) {
            if( valueIn < 0.33 ) {                      // NOTE: approx 0.06 required to **initiate** rotation
                valueOut = (0.25 * valueIn) + 0.0650;   // 0.02=0.070  0.33=0.1475
            }
            else if( valueIn < 0.60 ) {
                valueOut = (0.50 * valueIn) - 0.0175;   // 0.33=0.1475  0.60=0.2825
            }
            else if( valueIn < 0.90 ) {
                valueOut = (0.75 * valueIn) - 0.1675;   // 0.60=0.2825  0.90=0.5075
            }
            else
                valueOut = (6.00 * valueIn) - 4.8925;   // 0.90=0.5075  1.00=1.1075 (clipped!)
        }
        //========= NEGATIVE JOYSTICK INPUTS =========
        else { // valueIn < 0.0
            if( valueIn > -0.33 ) {
                valueOut = (0.25 * valueIn) - 0.0650;
            }
            else if( valueIn > -0.60 ) {
                valueOut = (0.50 * valueIn) + 0.0175;
            }
            else if( valueIn > -0.90 ) {
                valueOut = (0.75 * valueIn) + 0.1675;
            }
            else
                valueOut = (6.00 * valueIn) + 4.8925;
        }

        return valueOut/2.0;
    } // multSegLinearRot

    private double multSegLinearXY( double valueIn ) {
        double valueOut;

        //========= NO JOYSTICK INPUT =========
        if( Math.abs( valueIn) < 0.05 ) {
            valueOut = 0.0;
        }
        //========= POSITIVE JOYSTICK INPUTS =========
        else if( valueIn > 0.0 ) {
            if( valueIn < 0.50 ) {                       // NOTE: approx 0.06 required to **initiate** rotation
                valueOut = (0.25 * valueIn) + 0.040;     // 0.01=0.0425   0.50=0.1650
            }
            else if( valueIn < 0.90 ) {
                valueOut = (0.75 * valueIn) - 0.210;     // 0.50=0.1650   0.90=0.4650
            }
            else
                valueOut = (8.0 * valueIn) - 6.735;      // 0.90=0.4650   1.00=1.265 (clipped)
        }
        //========= NEGATIVE JOYSTICK INPUTS =========
        else { // valueIn < 0.0
            if( valueIn > -0.50 ) {
                valueOut = (0.25 * valueIn) - 0.040;
            }
            else if( valueIn > -0.90 ) {
                valueOut = (0.75 * valueIn) + 0.210;
            }
            else
                valueOut = (8.0 * valueIn) + 6.735;
        }

        return valueOut;
    } // multSegLinearXY

    /*---------------------------------------------------------------------------------*/
    /*  TELE-OP: Drive-motor diagnostic tool (command one wheel/motor at a time)       */
    /*---------------------------------------------------------------------------------*/
    void processSingleWheelControl() {
        // Use the motor-power variables so our telemetry updates correctly
        frontLeft  = minThreshold( gamepad1.left_stick_y  );
        frontRight = minThreshold( gamepad1.right_stick_y );
        rearLeft   = minThreshold( gamepad1.left_stick_x  );
        rearRight  = minThreshold( gamepad1.right_stick_x );

        // Update motor power settings:
        robot.driveTrainMotors( frontLeft, frontRight, rearLeft, rearRight );
    } // processSingleWheelControl

    /*---------------------------------------------------------------------------------*/
    /*  TELE-OP: Standard Mecanum-wheel drive control (no dependence on gyro!)         */
    /*---------------------------------------------------------------------------------*/
    void processStandardDriveMode() {
        // Retrieve X/Y and ROTATION joystick input
        if( controlMultSegLinear ) {  // robot centric results in 1.0 max power
            yTranslation = multSegLinearXY( -gamepad1.left_stick_y );
            xTranslation = multSegLinearXY(  gamepad1.left_stick_x );
            rotation     = multSegLinearRot( -gamepad1.right_stick_x );
        }
        else {
            yTranslation = -gamepad1.left_stick_y * 1.00;
            xTranslation = -gamepad1.left_stick_x * 1.25;
            rotation     = -gamepad1.right_stick_x * 0.50;
        }
        // If BACKWARD drive control, reverse the operator inputs
        if( backwardDriveControl ) {
            yTranslation = -yTranslation;
            xTranslation = -xTranslation;
          //rotation     = -rotation;  // clockwise/counterclockwise doesn't change
        } // backwardDriveControl
        // Normal teleop drive control:
        // - left joystick is TRANSLATE fwd/back/left/right
        // - right joystick is ROTATE clockwise/counterclockwise
        // NOTE: assumes the right motors are defined FORWARD and the
        // left motors are defined REVERSE so positive power is FORWARD.
        frontRight = yTranslation - xTranslation + rotation;
        frontLeft  = yTranslation + xTranslation - rotation;
        rearRight  = yTranslation + xTranslation + rotation;
        rearLeft   = yTranslation - xTranslation - rotation;

        // Normalize the values so none exceed +/- 1.0
        maxPower = Math.max( Math.max( Math.abs(rearLeft),  Math.abs(rearRight)  ),
                             Math.max( Math.abs(frontLeft), Math.abs(frontRight) ) );
        if (maxPower > 1.0)
        {
            rearLeft   /= maxPower;
            rearRight  /= maxPower;
            frontLeft  /= maxPower;
            frontRight /= maxPower;
        }
        // Update motor power settings:
        robot.driveTrainMotors( frontLeft, frontRight, rearLeft, rearRight );
    } // processStandardDriveMode

    /*---------------------------------------------------------------------------------*/
    /*  TELE-OP: Driver-centric Mecanum-wheel drive control (depends on gyro!)         */
    /*---------------------------------------------------------------------------------*/
    void processDriverCentricDriveMode() {
        double y, x, rx;
        double botHeading;
        double effectiveHeading;

        // Retrieve X/Y and ROTATION joystick input
        y = gamepad1.left_stick_y;
        x = -gamepad1.left_stick_x;
        rx = gamepad1.right_stick_x;
        botHeading = -robot.headingIMU();  // Assume this returns degrees; negative sign may need adjustment based on IMU convention

        if (gamepad1.square) {
            // The driver presses SQUARE, then uses the left joystick to say what angle the robot
            // is aiming.  This will calculate the values as long as SQUARE is pressed, and will
            // not drive the robot using the left stick.  Once SQUARE is released, it will use the
            // final calculated angle and drive with the left stick.  Button should be released
            // before stick.  The default behavior of atan2 is 0 to -180 on Y Axis CCW, and 0 to
            // 180 CW.  This code normalizes that to 0 to 360 CCW from the Y Axis
            driverAngle = Math.toDegrees(Math.atan2(gamepad1.left_stick_x, gamepad1.left_stick_y));
            if (driverAngle < 0) {
                driverAngle += 360.0;
            }
            driverAngle -= botHeading;
            x = 0.0;
            y = 0.0;
            rx = 0.0;
        }

        // Adjust new gyro angle for the driver reference angle
        effectiveHeading = Math.toRadians(botHeading + driverAngle);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-effectiveHeading) - y * Math.sin(-effectiveHeading);
        double rotY = x * Math.sin(-effectiveHeading) + y * Math.cos(-effectiveHeading);

        // Apply strafing compensation if needed (adjust 1.1 based on empirical testing)
        rotX = rotX * 1.1;

        // Normalize the values so none exceed +/- 1.0
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
        double frontLeft = (rotY + rotX + rx) / denominator;
        double frontRight = (rotY - rotX - rx) / denominator;
        double rearLeft = (rotY - rotX + rx) / denominator;
        double rearRight = (rotY + rotX - rx) / denominator;

        // Update motor power settings (assuming left motors are defined as REVERSE mode in hardware,
        // or adjust signs here if necessary. If positive power to all moves forward without negation,
        // remove the negatives below.)
        robot.driveTrainMotors(frontLeft, frontRight, rearLeft, rearRight);
    } // processDriverCentricDriveMode

    /*---------------------------------------------------------------------------------*/
    void processCollector() {
        // Check for an OFF-to-ON toggle of the gamepad2 CROSS button (toggles INTAKE on/off)
        if( gamepad2.crossWasPressed() )
        {
            if (intakeMotorOnFwd == false){
                // Turn on collector in FORWARD
                robot.intakeMotor.setPower( robot.INTAKE_FWD_COLLECT );
                intakeMotorOnFwd = true;
                intakeMotorOnRev = false;
            } else{
                // Shut OFF collector
                robot.intakeMotor.setPower(0.00);
                intakeMotorOnFwd = false;
                intakeMotorOnRev = false;
            }
        } // cross
        // Do we have too many balls and need to ANTI-collect?
        if( gamepad2.squareWasPressed() )
        {
            if (intakeMotorOnRev == false){
                // Turn on collector in REVERSE
                robot.intakeMotor.setPower( robot.INTAKE_REV_REJECT );
                intakeMotorOnFwd = false;
                intakeMotorOnRev = true;
            } else{
                // Shut OFF collector
                robot.intakeMotor.setPower(0.00);
                intakeMotorOnFwd = false;
                intakeMotorOnRev = false;
            }
        } // square
    } // processCollector

    /*---------------------------------------------------------------------------------*/
    void processSpindexer() {
        boolean safeToSpindex    = (robot.getInjectorAngle() <= robot.LIFT_SERVO_RESET_ANG);
        boolean leftTriggerHeld  = (gamepad2.left_trigger  > 0.25)? true:false;
        boolean rightTriggerHeld = (gamepad2.right_trigger > 0.25)? true:false;
        if( !safeToSpindex ) return;
        // Rotate spindexer RIGHT one FULL position?
        if( gamepad2.rightBumperWasPressed() ) {
            if (robot.spinServoCurPos != SPIN_P1)
                robot.spinServoSetPosition( SPIN_DECREMENT );
            else
                gamepad2.runRumbleEffect(spindexerRumbleL);
        }
        // Rotate spindexer LEFT one FULL position?
        else if( gamepad2.leftBumperWasPressed() ) {
            if( robot.spinServoCurPos != SPIN_P3 )
                robot.spinServoSetPosition( SPIN_INCREMENT );
            else
                gamepad2.runRumbleEffect(spindexerRumbleR);
        }
        // Temporarily rotate spindexer RIGHT one HALF position?
        else if( (rightTriggerHeld == true) && (robot.spinServoMidPos == false) ) {
           robot.spinServoSavPos = robot.spinServoCurPos;  // save current position
           HardwareSwyftBot.SpindexerState leftHalf = robot.whichSpindexerHalfPosition( SPIN_DECREMENT );
           robot.spinServoSetPosition( leftHalf );
           robot.spinServoMidPos = true;  // remember to undo!
        }
        // Temporarily rotate spindexer LEFTT one HALF position?
        else if( (leftTriggerHeld == true) && (robot.spinServoMidPos == false) ) {
           robot.spinServoSavPos = robot.spinServoCurPos;  // save current position
            HardwareSwyftBot.SpindexerState rightHalf = robot.whichSpindexerHalfPosition( SPIN_INCREMENT );
           robot.spinServoSetPosition( rightHalf );
           robot.spinServoMidPos = true;  // remember to undo!
        }
        // Do we need to RESTORE from a temporary half position?
        else if( (robot.spinServoMidPos == true) && (leftTriggerHeld == false) && (rightTriggerHeld == false)) {
            robot.spinServoSetPosition( robot.spinServoSavPos );
            robot.spinServoMidPos = false;
        }
    } // processSpindexer

    /*---------------------------------------------------------------------------------*/
    void processShooter() {
        // Check for an OFF-to-ON toggle of the gamepad2 CIRCLE button (toggles SHOOTER on/off)
        if( gamepad2.circleWasPressed() )
        {
            if (shooterMotorsOn == false){
                robot.shooterMotorsSetPower( shooterPower );
                shooterMotorsOn = true;
            } else {
                robot.shooterMotorsSetPower( 0.0 );
                shooterMotorsOn = false;
            }
        }
    } // processShooter

    private void processTurretAutoAim() {
        // Do we want to use them? (so long as the button is held...)
        if( autoAimEnabled ) {
            // update pinpoint coordinates if conditions are good to do so
//          updatePinpointFieldPosition0();
            // now that we have the latest coordinate update, compute the auto-aim parameters
            odoShootDistance = robot.getShootDistance( (blueAlliance)? Alliance.BLUE : Alliance.RED );
            odoShootAngleDeg = robot.getShootAngleDeg( (blueAlliance)? Alliance.BLUE : Alliance.RED );
            // set the turret angle and shooter power
            isAutoShooterAngleGood = robot.setTurretAngle(odoShootAngleDeg);
            shooterPower = robot.computeShooterPower(odoShootDistance);
            if(shooterMotorsOn) {
                robot.shooterMotorsSetPower(shooterPower);
            }
        } // autoAimEnabled
        else {
            // We're not going to use it to auto-aim, but still compute it for telemetry
            odoShootDistance = robot.getShootDistance( (blueAlliance)? Alliance.BLUE : Alliance.RED );
            odoShootAngleDeg = robot.getShootAngleDeg( (blueAlliance)? Alliance.BLUE : Alliance.RED );
        }
        // Has something gone wrong and we want to reset to manual straight-on mode?
        if (gamepad1.leftBumperWasPressed()) {
            autoAimEnabled = true;
        }
        else if (gamepad1.rightBumperWasPressed()) {
            // reset turret to the center and reset shooter power to FAR zone
            autoAimEnabled = false;
            robot.turretServoSetPosition(robot.TURRET_SERVO_INIT);
            shooterPower = 0.55;
            if(shooterMotorsOn) {
                robot.shooterMotorsSetPower(shooterPower);
            }
        }
        // Does the driver want a manual adjustment of the auto-aim angle?
        leftTriggerPressLast  = leftTriggerPressNow;
        leftTriggerPressNow   = (gamepad1.left_trigger > 0.25);
        rightTriggerPressLast = rightTriggerPressNow;
        rightTriggerPressNow  = (gamepad1.right_trigger > 0.25);

        if( leftTriggerPressNow && !leftTriggerPressLast  ) {
            robot.turretManualOffset += 3.0; // degrees
            if( robot.turretManualOffset > 15.0 ) robot.turretManualOffset = 15.0;
        } else if( rightTriggerPressNow && !rightTriggerPressLast ) {
            robot.turretManualOffset -= 3.0; // degrees
            if( robot.turretManualOffset < -15.0 ) robot.turretManualOffset = -15.0;
        }
    } // processTurretAutoAim

    /*---------------------------------------------------------------------------------*/
    void processInjector() {
        // Has the spindexer achieved one of the 3 valid shooting positions?
        boolean safeToInject = (robot.spinServoInPos && !robot.spinServoMidPos)? true:false;
        // Is the operator attempting to shoot? (either single or triple)
        boolean shootSingle = gamepad2.triangleWasPressed();//only query this once per cycle!
        boolean shootTriple = gamepad2.dpadUpWasPressed();
        // Is the robot too close to the goal to shoot?
        boolean tooCloseToShoot = (odoShootDistance < MIN_SHOOT_DISTANCE_INCHES);
        if( (shootSingle || shootTriple) && tooCloseToShoot ) {
            // notify both players robot is too close to shoot
            gamepad1.runRumbleEffect( tooCloseRumbleLR );
            gamepad2.runRumbleEffect( tooCloseRumbleLR );
        }
        // TRIANGLE button is a single-shot command
        if( shootSingle && safeToInject && !tooCloseToShoot ) {
            // Ensure an earlier injection request isn't already underway
            if ((robot.liftServoBusyU == false) && (robot.liftServoBusyD == false)) {
                robot.startInjectionStateMachine();
            }
        }
        // DPAD UP is the triple-shot command
        if( shootTriple && safeToInject && !tooCloseToShoot ) {
            // Ensure shooter is ON
            if (shooterMotorsOn == false){
                robot.shooterMotorsSetPower( shooterPower );
                shooterMotorsOn = true;
            }
            // Ensure collector is ON
            if (intakeMotorOnFwd == false){
                // Turn on collector in FORWARD
                robot.intakeMotor.setPower( robot.INTAKE_FWD_COLLECT );
                intakeMotorOnFwd = true;
                intakeMotorOnRev = false;
            }
            // start pew-pew-pew
            robot.startTripleShotStateMachine();
        }
        // DPAD DOWN cancels triple-shot
        else if( gamepad2.dpadDownWasPressed() ) {
            robot.abortTripleShotStateMachine();
        }
    } // processInjector

/*---------------------------------------------------------------------------------*/
    void processInjector0() {
        // Has the spindexer achieved one of the 3 valid shooting positions?
        boolean safeToInject = (robot.spinServoInPos && !robot.spinServoMidPos)? true:false;
        // TRIANGLE button is a single-shot command
        if( safeToInject && gamepad2.triangleWasPressed() ) {
            // Ensure an earlier injection request isn't already underway
            if ((robot.liftServoBusyU == false) && (robot.liftServoBusyD == false)) {
                robot.startInjectionStateMachine();
            }
        }
        // DPAD UP is the triple-shot command
        if( safeToInject && gamepad2.dpadUpWasPressed() ) {
            // Ensure shooter is ON
            if (shooterMotorsOn == false){
                robot.shooterMotorsSetPower( shooterPower );
                shooterMotorsOn = true;
            }
            // Ensure collector is ON
            if (intakeMotorOnFwd == false){
                // Turn on collector in FORWARD
                robot.intakeMotor.setPower( robot.INTAKE_FWD_COLLECT );
                intakeMotorOnFwd = true;
                intakeMotorOnRev = false;
            }
            // start pew-pew-pew
            robot.startTripleShotStateMachine();
        }
        // DPAD DOWN cancels triple-shot
        else if( gamepad2.dpadDownWasPressed() ) {
            robot.abortTripleShotStateMachine();
        }
    } // processInjector0

} // Teleop
