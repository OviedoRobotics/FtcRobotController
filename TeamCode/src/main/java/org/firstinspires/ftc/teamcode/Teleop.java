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
    boolean autoAimEnabled   = false; // turret power/angle only adjusted when this flag is enabled

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

    Gamepad.RumbleEffect spindexerRumbleL;    // Can't spin further LEFT!
    Gamepad.RumbleEffect spindexerRumbleR;    // Can't spin further RIGHT!

    long      nanoTimeCurr=0, nanoTimePrev=0;
    double    cycleTimeElapsed, cycleTimeHz;

    /* Declare OpMode members. */
    HardwareSwyftBot robot = new HardwareSwyftBot();
    // sets unique behavior based on alliance
    public abstract void setAllianceSpecificBehavior();

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("State", "Initializing (please wait)");
        telemetry.update();

        spindexerRumbleL = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 0.0, 250)  //  Rumble LEFT motor 100% for 250 mSec
                .build();

        spindexerRumbleR = new Gamepad.RumbleEffect.Builder()
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
                    backwardDriveControl = true;  // start with phone-end as front of robot
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

            processCollector();
            processTurretAutoAim();
            processSpindexer();
            processShooterFlap();
            processShooter();
            processInjector();

            // Compute current cycle time
            nanoTimePrev = nanoTimeCurr;
            nanoTimeCurr = System.nanoTime();
            cycleTimeElapsed = (nanoTimeCurr - nanoTimePrev)/ 1000000.0;   // msec
            cycleTimeHz =  1000.0 / cycleTimeElapsed;

            // Update telemetry data
            telemetry.addData("Limelight","x=%.2f y=%.2f  %.2f deg (Apriltag)",
                    robot.limelightFieldXpos, robot.limelightFieldYpos, robot.limelightFieldAngleDeg );
            telemetry.addData("  stdev","%.5f %.5f  %.5f",
                    robot.limelightFieldXstd, robot.limelightFieldYstd, robot.limelightFieldAnglestd );
            telemetry.addData("Pinpoint","x=%.2f y=%.2f  %.2f deg (odom)",
                   robot.robotGlobalXCoordinatePosition, robot.robotGlobalYCoordinatePosition, robot.robotOrientationDegrees );
            telemetry.addData(" "," %.2f in/sec %.2f in/sec %.2f deg/sec", 
                   robot.robotGlobalXvelocity, robot.robotGlobalYvelocity, robot.robotAngleVelocity );
            telemetry.addData("Goal", "%s dist: %.2f in, angle: %.2f deg", ((blueAlliance)? "BLUE":"RED"), odoShootDistance, odoShootAngleDeg);
//          telemetry.addData("Shooter POWER", "%.3f (P1 tri/cross to adjust)", shooterPower);
//          telemetry.addData("Shooter RPM", "%.1f %.1f", robot.shooterMotor1Vel, robot.shooterMotor2Vel );
            telemetry.addData("Turret", "set %.3f get %.3f analog %.3f", robot.turretServoSet, robot.turretServoGet, robot.turretServoPos );
//          telemetry.addData("Shooter mA", "%.1f %.1f", robot.shooterMotor1Amps, robot.shooterMotor2Amps );
//          telemetry.addData("Spindexer", "set %.2f get %.2f time %.3f ms",
//                  robot.spinServoSetPos, robot.getSpindexerPos(), robot.spinServoTime );
            telemetry.addLine( (robot.isRobot2)? "Robot2" : "Robot1");
//          telemetry.addData("Driver Angle", "%.3f deg", driverAngle );
//          telemetry.addData("IMU Angle", "%.3f deg", robot.headingIMU() );
//          telemetry.addData("Driver Centric", "%.3f deg", (driverAngle - robot.headingIMU()) );
            telemetry.addData("Robot velocity", "x=%.2f y=%.2f ang=%.2f",
                robot.robotGlobalXvelocity, robot.robotGlobalYvelocity, robot.robotAngleVelocity );
            telemetry.addData("CycleTime", "%.1f msec (%.1f Hz)", cycleTimeElapsed, cycleTimeHz);
            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
//          robot.waitForTick(40);
        } // opModeIsActive

//  robot.spinServoCR.setPower(0.0);  // only for spinServoCR (not currently used)
    } // runOpMode

    /*---------------------------------------------------------------------------------*/
    void performEveryLoopTeleop() {
        robot.readBulkData();
        robot.processInjectionStateMachine();
//      robot.processSpindexerControl();  // only for spinServoCR (not currently used)
        if( enableOdometry ) {
            robot.updatePinpointFieldPosition();
            robot.updateLimelightFieldPosition();
        } // enableOdometry
        // Do we manually update the field position based on apriltag?
        if( gamepad1.touchpadWasPressed() ){
            updatePinpointFieldPosition();
        }
    } // performEveryLoopTeleop

    void updatePinpointFieldPosition() {
        // Ensure we don't get a spurious zero/clear reading
        boolean canSeeAprilTag = (robot.limelightFieldXpos != 0.0) && (robot.limelightFieldYpos !=0.0) && (robot.limelightFieldAngleDeg != 0.0);
        boolean qualityReading = (robot.limelightFieldXstd < 0.002) && (robot.limelightFieldYstd < 0.002);
        boolean robotXslow = (Math.abs(robot.robotGlobalXvelocity) < 0.1)? true:false;
        boolean robotYslow = (Math.abs(robot.robotGlobalYvelocity) < 0.1)? true:false;
        boolean robotAslow = (Math.abs(robot.robotAngleVelocity)   < 0.1)? true:false;
        boolean notDriving = (robotXslow && robotYslow && robotAslow)?  true:false;
        if( canSeeAprilTag && qualityReading && notDriving ) {
            robot.setPinpointFieldPosition(robot.limelightFieldXpos, robot.limelightFieldYpos, robot.limelightFieldAngleDeg);
        }
    }  // updatePinpointFieldPosition

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
                robot.intakeMotor.setPower(0.90);
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
                robot.intakeMotor.setPower(-0.90);
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
        boolean safeToSpindex = (robot.getInjectorAngle() <= robot.LIFT_SERVO_RESET_ANG);
        if( !safeToSpindex ) return;
        // Rotate spindexer left one position?
        if( gamepad2.leftBumperWasPressed() ) {
            if (robot.spinServoCurPos != SPIN_P1)
                robot.spinServoSetPosition(SPIN_DECREMENT);
            else
                gamepad2.runRumbleEffect(spindexerRumbleL);
//          robot.spinServoSetPositionCR(SPIN_DECREMENT);  // only for spinServoCR
        }
        // Rotate spindexer right one position?
        else if( gamepad2.rightBumperWasPressed() ) {
            if( robot.spinServoCurPos != SPIN_P3 )
                robot.spinServoSetPosition( SPIN_INCREMENT );
            else
                gamepad2.runRumbleEffect(spindexerRumbleR);
//          robot.spinServoSetPositionCR(SPIN_INCREMENT);   // only for spinServoCR
        } // bumper
    } // processSpindexer

    /*---------------------------------------------------------------------------------*/
    void processShooterFlap() {
    // Check for an OFF-to-ON toggle of gamepad2 DPAD buttons (controls shooter flapper up/down)
        if( gamepad2.dpadDownWasPressed() ) {
            // aim LOWER
            robot.shooterServoCurPos += 0.01;
            // Don't exceed our mechanical limits
            if( robot.shooterServoCurPos > robot.SHOOTER_SERVO_MAX )
                robot.shooterServoCurPos = robot.SHOOTER_SERVO_MAX;
            robot.shooterServo.setPosition( robot.shooterServoCurPos );
        }
        else if( gamepad2.dpadUpWasPressed() ) {
            // aim HIGHER
            robot.shooterServoCurPos -= 0.01;
            // Don't exceed our mechanical limits
            if( robot.shooterServoCurPos < robot.SHOOTER_SERVO_MIN )
                robot.shooterServoCurPos = robot.SHOOTER_SERVO_MIN;
            robot.shooterServo.setPosition( robot.shooterServoCurPos );
        }
    }   // processShooterFlap

    /*---------------------------------------------------------------------------------*/
    void processShooter() {
        if( gamepad1.triangleWasPressed() ) {
            shooterPower += 0.01;
            if(shooterMotorsOn) {
                robot.shooterMotorsSetPower( shooterPower );
            }
        } else if( gamepad1.crossWasPressed() ) {
            shooterPower -= 0.01;
            if(shooterMotorsOn) {
                robot.shooterMotorsSetPower( shooterPower );
            }
        }

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
        autoAimEnabled = gamepad1.left_bumper;
        if( autoAimEnabled ) {
            // update pinpoint coordinates if conditions are good to do so
            updatePinpointFieldPosition();
            // now that we have the latest coordinate update, compute the auto-aim parameters
            odoShootDistance = robot.getShootDistance( (blueAlliance)? Alliance.BLUE : Alliance.RED );
            odoShootAngleDeg = robot.getShootAngleDeg( (blueAlliance)? Alliance.BLUE : Alliance.RED );
            // set the turret angle and shooter power
            robot.setTurretAngle(odoShootAngleDeg);
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
        if (gamepad1.rightBumperWasPressed()) {
            // reset turret to the center and reset shooter power to FAR zone
            robot.turretServoSetPosition(robot.TURRET_SERVO_INIT);
            shooterPower = 0.55;
            if(shooterMotorsOn) {
                robot.shooterMotorsSetPower(shooterPower);
            }
        }
    } // processTurretAutoAim

    /*---------------------------------------------------------------------------------*/
    void processInjector() {
        // Check for an OFF-to-ON toggle of the gamepad2 TRIANGLE button (command ball injection!)
        if( gamepad2.triangleWasPressed() ) {
            // Ensure an earlier injection request isn't already underway
            if ((robot.liftServoBusyU == false) && (robot.liftServoBusyD == false)) {
                robot.startInjectionStateMachine();
            }
        }
    } // processInjector

} // Teleop
