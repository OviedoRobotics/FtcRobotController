/* FTC Team 7572 - Version 1.0 (09/05/2025)
*/
package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.HardwareSwyftBot.SpindexerState.SPIN_DECREMENT;
import static org.firstinspires.ftc.teamcode.HardwareSwyftBot.SpindexerState.SPIN_INCREMENT;
import static org.firstinspires.ftc.teamcode.HardwareSwyftBot.SpindexerState.SPIN_P1;
import static org.firstinspires.ftc.teamcode.HardwareSwyftBot.SpindexerState.SPIN_P3;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

/**
 * TeleOp for the 2025-2026 FTC DECODE Season
 */
//@Disabled
public abstract class Teleop extends LinearOpMode {
    boolean gamepad1_triangle_last,   gamepad1_triangle_now   = false;  // Single Wheel Control
    boolean gamepad1_circle_last,     gamepad1_circle_now     = false;  // Backwards Drive mode (also turns off driver-centric mode)
    boolean gamepad1_cross_last,      gamepad1_cross_now      = false;  // UNUSED
    boolean gamepad1_square_last,     gamepad1_square_now     = false;  // Enables/calibrates driver-centric mode
    boolean gamepad1_dpad_up_last,    gamepad1_dpad_up_now    = false;
    boolean gamepad1_dpad_down_last,  gamepad1_dpad_down_now  = false;
    boolean gamepad1_dpad_left_last,  gamepad1_dpad_left_now  = false;
    boolean gamepad1_dpad_right_last, gamepad1_dpad_right_now = false;
    boolean gamepad1_l_bumper_last,   gamepad1_l_bumper_now   = false;  // UNUSED
    boolean gamepad1_r_bumper_last,   gamepad1_r_bumper_now   = false;  // UNUSED
    boolean gamepad1_touchpad_last,   gamepad1_touchpad_now   = false;  
    boolean gamepad1_l_trigger_last,  gamepad1_l_trigger_now  = false;
    boolean gamepad1_r_trigger_last,  gamepad1_r_trigger_now  = false;

    boolean gamepad2_triangle_last,   gamepad2_triangle_now   = false;  //
    boolean gamepad2_circle_last,     gamepad2_circle_now     = false;  //
    boolean gamepad2_cross_last,      gamepad2_cross_now      = false;  //
    boolean gamepad2_square_last,     gamepad2_square_now     = false;  //
    boolean gamepad2_dpad_up_last,    gamepad2_dpad_up_now    = false;  //
    boolean gamepad2_dpad_down_last,  gamepad2_dpad_down_now  = false;  //
    boolean gamepad2_dpad_left_last,  gamepad2_dpad_left_now  = false;  //
    boolean gamepad2_dpad_right_last, gamepad2_dpad_right_now = false;  //
    boolean gamepad2_l_bumper_last,   gamepad2_l_bumper_now   = false;  //
    boolean gamepad2_r_bumper_last,   gamepad2_r_bumper_now   = false;  //
    boolean gamepad2_touchpad_last,   gamepad2_touchpad_now   = false;  //
    boolean gamepad2_share_last,      gamepad2_share_now      = false;  //

    double  yTranslation, xTranslation, rotation;                  /* Driver control inputs */
    double  rearLeft, rearRight, frontLeft, frontRight, maxPower;  /* Motor power levels */
    boolean backwardDriveControl = false; // drive controls backward (other end of robot becomes "FRONT")
    boolean controlMultSegLinear = true;
    double   curX, curY, curAngle;

    boolean blueAlliance;   // set in the Blue/Red
    boolean farAlliance;    //
    int     aprilTagGoal;

    final int DRIVER_MODE_SINGLE_WHEEL = 1;
    final int DRIVER_MODE_STANDARD     = 2;
    final int DRIVER_MODE_DRV_CENTRIC  = 3;
    int       driverMode               = DRIVER_MODE_STANDARD;
    double    driverAngle              = 0.0;  /* for DRIVER_MODE_DRV_CENTRIC */

    boolean enableOdometry = true;
    boolean intakeMotorOnFwd = false;
    boolean intakeMotorOnRev = false;
    boolean shooterMotorsOn = false;

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
        setAllianceSpecificBehavior();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("State", "Ready");
        telemetry.addLine("Press X (cross) to reset encoders");
        telemetry.addLine("(to run Teleop without Auto first)");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        while (!isStarted()) {
            // Check for operator input that changes Autonomous options
            captureGamepad1Buttons();
            // Normally autonomous resets encoders.  Do we need to for teleop??
            if( gamepad1_cross_now && !gamepad1_cross_last) {
                robot.resetEncoders();
            }
            // Pause briefly before looping
            idle();
        } // !isStarted

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            // Refresh gamepad button status
            captureGamepad1Buttons();
            captureGamepad2Buttons();

            // Bulk-refresh the Control/Expansion Hub device status (motor status, digital I/O) -- FASTER!
            robot.readBulkData();

            // Request an update from the Pinpoint odometry computer (single I2C read)
            if( enableOdometry ) {
                robot.odom.update();
                Pose2D pos = robot.odom.getPosition();  // x,y pos in inch; heading in degrees
                curX     = pos.getX(DistanceUnit.INCH);
                curY     = pos.getY(DistanceUnit.INCH);
                curAngle = pos.getHeading(AngleUnit.DEGREES);
                String posStr = String.format(Locale.US, "{X,Y: %.1f, %.1f in  H: %.1f deg}", curX, curY, curAngle);
                telemetry.addData("Position", posStr);
                //==== TEMPORARY ODOMETRY CALIBRATION CODE ============================================================
//              if(curX<minX){minX=curX;} if(curX>maxX){maxX=curX;}
//              if(curY<minY){minY=curY;} if(curY>maxY){maxY=curY;}
//              double x_radius_mm = 25.4 * (maxX-minX)/2.0;  // rotate 180deg; max-min is the diameter of the circle
//              double y_radius_mm = 25.4 * (maxY-minY)/2.0;  // of error relative to the true center of the robot
//              telemetry.addData("Odo Circle", "x=%.2f, y=%.2f mm", x_radius_mm, y_radius_mm );
                //=====================================================================================================
                Pose2D vel = robot.odom.getVelocity(); // x,y velocities in inch/sec; heading in deg/sec
                String velStr = String.format(Locale.US,"{X,Y: %.1f, %.1f in/sec, HVel: %.2f deg/sec}",
                     vel.getX(DistanceUnit.INCH), vel.getY(DistanceUnit.INCH), vel.getHeading(AngleUnit.DEGREES));
                telemetry.addData("Velocity", velStr);
                telemetry.addData("Status", robot.odom.getDeviceStatus());
            }

            // Check for an OFF-to-ON toggle of the gamepad1 TRIANGLE button (toggles SINGLE-MOTOR drive control)
            if( gamepad1_triangle_now && !gamepad1_triangle_last)
            {
                driverMode = DRIVER_MODE_SINGLE_WHEEL; // allow control of individual drive motors
            }

            // Check for an OFF-to-ON toggle of the gamepad1 SQUARE button (toggles DRIVER-CENTRIC drive control)
            if( gamepad1_square_now && !gamepad1_square_last)
            {
                driverMode = DRIVER_MODE_DRV_CENTRIC;
            }

            // Check for an OFF-to-ON toggle of the gamepad1 CIRCLE button (toggles STANDARD/BACKWARD drive control)
            if( gamepad1_circle_now && !gamepad1_circle_last)
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

            telemetry.addData("cross","Toggle Intake");
            telemetry.addData("triangle","Single Wheel Control");
            telemetry.addData("circle","Robot-centric (fwd/back modes)");
            telemetry.addData("square","Driver-centric (set joystick!)");
            telemetry.addData("d-pad","Fine control (30%)");
            telemetry.addData(" "," ");

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
            processSpindexer();
            processShooterFlap();
            processShooter();
            processInjector();
            performEveryLoopTeleop();

            // Compute current cycle time
            nanoTimePrev = nanoTimeCurr;
            nanoTimeCurr = System.nanoTime();
            cycleTimeElapsed = (nanoTimeCurr - nanoTimePrev)/ 1000000.0;   // msec
            cycleTimeHz =  1000.0 / cycleTimeElapsed;

            // Update telemetry data
            telemetry.addData("Shooter Servo", "%.3f", robot.shooterServoCurPos );
            telemetry.addData("Shooter RPM", "%.1f", robot.shooterMotorVel );
//          telemetry.addData("Angles", "IMU %.2f, Pinpoint %.2f deg)", robot.headingIMU(), curAngle );
            telemetry.addData("CycleTime", "%.1f msec (%.1f Hz)", cycleTimeElapsed, cycleTimeHz);
            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
//          robot.waitForTick(40);
        } // opModeIsActive

    } // runOpMode

    /*---------------------------------------------------------------------------------*/
    void performEveryLoopTeleop() {
        robot.processInjectionStateMachine();
//      robot.processViperSlideExtension();
//      robot.processWormTilt();
//      processHoverArm();
//      processSecureArm();
//      processScoreArm();
//      processScoreArmSpec();
//      processSweeper();
    } // performEveryLoopTeleop

    /*---------------------------------------------------------------------------------*/
    void captureGamepad1Buttons() {
        gamepad1_triangle_last   = gamepad1_triangle_now;    gamepad1_triangle_now   = gamepad1.triangle;
        gamepad1_circle_last     = gamepad1_circle_now;      gamepad1_circle_now     = gamepad1.circle;
        gamepad1_cross_last      = gamepad1_cross_now;       gamepad1_cross_now      = gamepad1.cross;
        gamepad1_square_last     = gamepad1_square_now;      gamepad1_square_now     = gamepad1.square;
        gamepad1_dpad_up_last    = gamepad1_dpad_up_now;     gamepad1_dpad_up_now    = gamepad1.dpad_up;
        gamepad1_dpad_down_last  = gamepad1_dpad_down_now;   gamepad1_dpad_down_now  = gamepad1.dpad_down;
        gamepad1_dpad_left_last  = gamepad1_dpad_left_now;   gamepad1_dpad_left_now  = gamepad1.dpad_left;
        gamepad1_dpad_right_last = gamepad1_dpad_right_now;  gamepad1_dpad_right_now = gamepad1.dpad_right;
        gamepad1_l_bumper_last   = gamepad1_l_bumper_now;    gamepad1_l_bumper_now   = gamepad1.left_bumper;
        gamepad1_r_bumper_last   = gamepad1_r_bumper_now;    gamepad1_r_bumper_now   = gamepad1.right_bumper;
        gamepad1_touchpad_last   = gamepad1_touchpad_now;    gamepad1_touchpad_now   = gamepad1.touchpad;
    } // captureGamepad1Buttons

    /*---------------------------------------------------------------------------------*/
    void captureGamepad2Buttons() {
        gamepad2_triangle_last   = gamepad2_triangle_now;    gamepad2_triangle_now   = gamepad2.triangle;
        gamepad2_circle_last     = gamepad2_circle_now;      gamepad2_circle_now     = gamepad2.circle;
        gamepad2_cross_last      = gamepad2_cross_now;       gamepad2_cross_now      = gamepad2.cross;
        gamepad2_square_last     = gamepad2_square_now;      gamepad2_square_now     = gamepad2.square;
        gamepad2_dpad_up_last    = gamepad2_dpad_up_now;     gamepad2_dpad_up_now    = gamepad2.dpad_up;
        gamepad2_dpad_down_last  = gamepad2_dpad_down_now;   gamepad2_dpad_down_now  = gamepad2.dpad_down;
        gamepad2_dpad_left_last  = gamepad2_dpad_left_now;   gamepad2_dpad_left_now  = gamepad2.dpad_left;
        gamepad2_dpad_right_last = gamepad2_dpad_right_now;  gamepad2_dpad_right_now = gamepad2.dpad_right;
        gamepad2_l_bumper_last   = gamepad2_l_bumper_now;    gamepad2_l_bumper_now   = gamepad2.left_bumper;
        gamepad2_r_bumper_last   = gamepad2_r_bumper_now;    gamepad2_r_bumper_now   = gamepad2.right_bumper;
        gamepad2_touchpad_last   = gamepad2_touchpad_now;    gamepad2_touchpad_now   = gamepad2.touchpad;
//      gamepad2_share_last      = gamepad2_share_now;       gamepad2_share_now      = gamepad2.share;
    } // captureGamepad2Buttons

    /*---------------------------------------------------------------------------------*/
    /*  TELE-OP: Mecanum-wheel drive control using Dpad (slow/fine-adjustment mode)    */
    /*---------------------------------------------------------------------------------*/
    boolean processDpadDriveMode() {
        double fineControlSpeed = 0.40;
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
        if( gamepad2_cross_now && !gamepad2_cross_last)
        {
            if (intakeMotorOnFwd == false){
                robot.intakeMotor.setPower(0.90);
                intakeMotorOnFwd = true;
                intakeMotorOnRev = false;
            } else{
                robot.intakeMotor.setPower(0.00);
                intakeMotorOnFwd = false;
                intakeMotorOnRev = false;
            }
        } // cross
        // Do we have too many balls and need to ANTI-collect?
        if( gamepad2_square_now && !gamepad2_square_last)
        {
            if (intakeMotorOnRev == false){
                robot.intakeMotor.setPower(-0.90);
                intakeMotorOnFwd = false;
                intakeMotorOnRev = true;
            } else{
                robot.intakeMotor.setPower(0.00);
                intakeMotorOnFwd = false;
                intakeMotorOnRev = false;
            }
        } // square
    } // processCollector

    /*---------------------------------------------------------------------------------*/
    void processSpindexer() {
        // Rotate spindexer left or right one position
        if( gamepad2_l_bumper_now && !gamepad2_l_bumper_last) {
            robot.waitForInjector();
            if( robot.spinServoCurPos != SPIN_P1 )
                robot.spinServoSetPosition( SPIN_DECREMENT );
            else
                gamepad2.runRumbleEffect(spindexerRumbleL);            
        } else if( gamepad2_r_bumper_now && !gamepad2_r_bumper_last) {
            robot.waitForInjector();
            if( robot.spinServoCurPos != SPIN_P3 )
                robot.spinServoSetPosition( SPIN_INCREMENT );
            else
                gamepad2.runRumbleEffect(spindexerRumbleR);
        } 
    } // processSpindexer

    /*---------------------------------------------------------------------------------*/
    void processShooterFlap() {
    // Check for an OFF-to-ON toggle of gamepad2 DPAD buttons (controls shooter flapper up/down)
        if( gamepad2_dpad_down_now && !gamepad2_dpad_down_last ) {
            // aim LOWER
            robot.shooterServoCurPos += 0.01;
            // Don't exceed our mechanical limits
            if( robot.shooterServoCurPos > robot.SHOOTER_SERVO_MAX )
                robot.shooterServoCurPos = robot.SHOOTER_SERVO_MAX;
            robot.shooterServo.setPosition( robot.shooterServoCurPos );
        }
        else if( gamepad2_dpad_up_now && !gamepad2_dpad_up_last ) {
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
        double shooterPower = 0.55;  // function of location!!
        // Check for an OFF-to-ON toggle of the gamepad2 CIRCLE button (toggles SHOOTER on/off)
        if( gamepad2_circle_now && !gamepad2_circle_last)
        {
            if (shooterMotorsOn == false){
                robot.shooterMotor1.setPower( shooterPower );
                robot.shooterMotor2.setPower( shooterPower );
                shooterMotorsOn = true;
            } else {
                robot.shooterMotor1.setPower( 0.0 );
                robot.shooterMotor2.setPower( 0.0 );
                shooterMotorsOn = false;
            }
        }
    } // processShooter

    /*---------------------------------------------------------------------------------*/
    void processInjector() {
        // Check for an OFF-to-ON toggle of the gamepad2 TRIANGLE button (command ball injection!)
        if( gamepad2_triangle_now && !gamepad2_triangle_last) {
            // Ensure an earlier injection request isn't already underway
            if ((robot.liftServoBusyU == false) && (robot.liftServoBusyD == false)) {
                robot.startInjectionStateMachine();
            }
        }
    } // processInjector

} // Teleop
