package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.HardwareSwyftBot.SpindexerState.SPIN_DECREMENT;
import static org.firstinspires.ftc.teamcode.HardwareSwyftBot.SpindexerState.SPIN_P1;
import static org.firstinspires.ftc.teamcode.HardwareSwyftBot.SpindexerState.SPIN_P2;
import static org.firstinspires.ftc.teamcode.HardwareSwyftBot.SpindexerState.SPIN_P3;
import static java.lang.Math.abs;
import static java.lang.Math.toRadians;

import android.os.Environment;
import android.os.SystemClock;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.io.File;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;
import java.util.Locale;

public abstract class AutonomousBase extends LinearOpMode {
    /* Declare OpMode members. */
    HardwareSwyftBot robot = new HardwareSwyftBot();

    static final int     DRIVE_TO             = 1;       // ACCURACY: tighter tolerances, and slows then stops at final position
    static final int     DRIVE_THRU           = 2;       // SPEED: looser tolerances, and leave motors running (ready for next command)
    static final double  P_DRIVE_COEFF        = 0.005;   // Larger is more responsive, but also less stable
    static final double  HEADING_THRESHOLD    = 2.0;     // Minimum of 1 degree for an integer gyro
    static final double  P_TURN_COEFF         = 0.050;   // Larger is more responsive, but also less stable
    static final double  DRIVE_SPEED_10       = 0.10;    // 
    static final double  DRIVE_SPEED_20       = 0.20;    // Lower speed for moving from a standstill
    static final double  DRIVE_SPEED_30       = 0.30;    // Lower speed for fine control going sideways
    static final double  DRIVE_SPEED_40       = 0.40;    // Normally go slower to achieve better accuracy
    static final double  DRIVE_SPEED_45       = 0.45;    // 
    static final double  DRIVE_SPEED_50       = 0.50;    //
    static final double  DRIVE_SPEED_55       = 0.55;    //
    static final double  DRIVE_SPEED_60       = 0.60;    //
    static final double  DRIVE_SPEED_70       = 0.70;    //
    static final double  DRIVE_SPEED_75       = 0.75;    //
    static final double  DRIVE_SPEED_80       = 0.80;    //
    static final double  DRIVE_SPEED_90       = 0.90;    //
    static final double  DRIVE_SPEED_100      = 1.00;    //
    static final double  TURN_SPEED_15        = 0.15;    //
    static final double  TURN_SPEED_20        = 0.15;    //
    static final double  TURN_SPEED_30        = 0.30;    //
    static final double  TURN_SPEED_40        = 0.40;    //
    static final double  TURN_SPEED_50        = 0.50;    //
    static final double  TURN_SPEED_55        = 0.55;    //
    static final double  TURN_SPEED_60        = 0.60;    //
    static final double  TURN_SPEED_70        = 0.70;    //
    static final double  TURN_SPEED_80        = 0.80;    //
    static final double  TURN_SPEED_90        = 0.90;    //
    static final double  TURN_SPEED_100       = 1.00;    //
    static final double STRAFE_MULTIPLIER = 1.5;
    static final double MIN_SPIN_RATE      = 0.06;    // Minimum power to turn the robot
    static final double MIN_DRIVE_POW      = 0.06;    // Minimum speed to move the robot
    static final double MIN_DRIVE_MAGNITUDE = Math.sqrt(MIN_DRIVE_POW*MIN_DRIVE_POW+MIN_DRIVE_POW*MIN_DRIVE_POW);
    static final double FIRST_SPIKE_MARK_RED_POS_X = 0.0; // TODO: Find real position
    static final double FIRST_SPIKE_MARK_RED_POS_Y = 0.0; // TODO: Find real position

    public Limelight3A limelight;
    public int         obeliskID=23; // if we can't see it, default to PPG (purple purple green)

    // NOTE: Initializing the odometry global X-Y and ANGLE to 0-0 and 0deg means the frame of reference for all movements is
    // the starting positiong/orientation of the robot.  An alternative is to make the bottom-left corner of the field the 0-0
    // point, with 0deg pointing forward.  That allows all absolute driveToPosition() commands to be an absolute x-y location
    // on the field.
    double robotGlobalXCoordinatePosition       = 0.0;   // inches
    double robotGlobalYCoordinatePosition       = 0.0;   // inches
    double robotOrientationRadians              = 0.0;   // radians 0deg (straight forward)

    // Odometry values corrected by external source, IE AprilTags
//  FieldCoordinate robotGlobalCoordinateCorrectedPosition = new FieldCoordinate(0.0, 0.0, 0.0);

    double autoXpos                             = 0.0;   // Keeps track of our Autonomous X-Y position and Angle commands.
    double autoYpos                             = 0.0;   // (useful when a given value remains UNCHANGED from one
    double autoAngle                            = 0.0;   // movement to the next, or INCREMENTAL change from current location).

    String      storageDir;
    boolean     redAlliance      = true;  // Is alliance BLUE (true) or RED (false)?
    boolean     forceAlliance    = false; // Override vision pipeline? (toggled during init phase of autonomous)
    int         initMenuSelected = 1;    // start on the first entry
    int         initMenuMax      = 2;    // we have 6 total entries
    int         startDelaySec    = 0;     // 1: wait [seconds] at startup -- applies to both left/rigth starting positions
    int         scoringZones     = 0;

    //---------------------------------------------------------------------------------------
    // When to open the gate and release classified balls during autonomous is a function
    // of many factors.  This setting controls when our robot will open the gate.  It could
    // be NEVER, if we either only score 9 balls total and want them to get scored at the
    // end of autonomous, or if we plan to score more than 9 total but our alliance partner
    // tasked with opening the gate.  Similiarly, after the first pre-loaded 3 balls, or 6
    // balls or all 9 balls, depending upon other factors.  This allows dynamic selection
    // during init.
    public enum GateOptions {
       GATE_NEVER("NEVER"),    // NEVER open the gate during Autonomous
       GATE_AFTER3("After 3"), // Open the gate after scoring 3 balls
       GATE_AFTER6("After 6"), // Open the gate after scoring 6 balls
       GATE_AFTER9("After 9"); // Open the gate after scoring 9 balls

       private final String description;

       // Private constructor - called for each enum constant above
       private GateOptions(String description) {
           this.description = description;
       }

       // Description Getter
       public String getDescription() {
           return description;
       }

       // Nice string representation
       @Override
       public String toString() {
           return name() + " (" + description + ")";
       }

       // Increment (next) with wrap-around back to start
       public GateOptions next() {
           GateOptions[] values = values();
           return values[(this.ordinal() + 1) % values.length];
       }

       // Decrement (previous) with wrap-around back to end
       public GateOptions prev() {
           GateOptions[] values = values();
           int prevIndex = (this.ordinal() - 1 + values.length) % values.length;
           // the +values.length guarantees non-negative result when ordinal() == 0
           return values[prevIndex];
       }

    } // enum

    GateOptions gateOption = GateOptions.GATE_NEVER;

    //---------------------------------------------------------------------------------------

    ElapsedTime autonomousTimer     = new ElapsedTime();  // overall
    ElapsedTime motionTimer         = new ElapsedTime();  // for driving

    // gamepad controls for changing autonomous options
    boolean gamepad1_circle_last,   gamepad1_circle_now  =false;
    boolean gamepad1_cross_last,    gamepad1_cross_now   =false;
    boolean gamepad1_l_bumper_last, gamepad1_l_bumper_now=false;
    boolean gamepad1_r_bumper_last, gamepad1_r_bumper_now=false;
    boolean gamepad1_dpad_up_last, gamepad1_dpad_up_now = false;
    boolean gamepad1_dpad_down_last, gamepad1_dpad_down_now = false;
    boolean gamepad1_dpad_left_last, gamepad1_dpad_left_now = false;
    boolean gamepad1_dpad_right_last, gamepad1_dpad_right_now = false;

    /*---------------------------------------------------------------------------------*/
    protected void captureGamepad1Buttons() {
        gamepad1_circle_last     = gamepad1_circle_now;      gamepad1_circle_now     = gamepad1.circle;
        gamepad1_cross_last      = gamepad1_cross_now;       gamepad1_cross_now      = gamepad1.cross;
        gamepad1_l_bumper_last   = gamepad1_l_bumper_now;    gamepad1_l_bumper_now   = gamepad1.left_bumper;
        gamepad1_r_bumper_last   = gamepad1_r_bumper_now;    gamepad1_r_bumper_now   = gamepad1.right_bumper;
        gamepad1_dpad_up_last    = gamepad1_dpad_up_now;     gamepad1_dpad_up_now    = gamepad1.dpad_up;
        gamepad1_dpad_down_last  = gamepad1_dpad_down_now;   gamepad1_dpad_down_now  = gamepad1.dpad_down;
        gamepad1_dpad_left_last  = gamepad1_dpad_left_now;   gamepad1_dpad_left_now  = gamepad1.dpad_left;
        gamepad1_dpad_right_last = gamepad1_dpad_right_now;  gamepad1_dpad_right_now = gamepad1.dpad_right;
    } // captureGamepad1Buttons

    protected void processAutonomousInitMenu(boolean auto5) {
        // Refresh the gamepad controls
        captureGamepad1Buttons();
        // Update  mechanisms (including bulkread, spindexer & odometry)
        performEveryLoop();

        boolean nextEntry = (gamepad1_dpad_down_now  && !gamepad1_dpad_down_last);
        boolean prevEntry = (gamepad1_dpad_up_now    && !gamepad1_dpad_up_last);
        boolean nextValue = (gamepad1_dpad_right_now && !gamepad1_dpad_right_last);
        boolean prevValue = (gamepad1_dpad_left_now  && !gamepad1_dpad_left_last);

        // Shift DOWN one menu entry?
        if( nextEntry ) {
            initMenuSelected++;
            if (initMenuSelected > initMenuMax) {
                initMenuSelected = 1;  // wrap back to the start
            }
        } // next

        // Shift UP one menu entry?
        if( prevEntry ) {
            initMenuSelected--;
            if (initMenuSelected < 1) {
                initMenuSelected = initMenuMax;  // wrap down to the end
            }
        } // prev

        switch( initMenuSelected ) {
            //-------------- START DELAY [sec] --------------
            case 1 :
                if( nextValue ) {
                    if (startDelaySec < 9) {
                        startDelaySec++;
                    }
                } // next
                if( prevValue ) {
                    if (startDelaySec > 0) {
                        startDelaySec--;
                    }
                } // prev
                break;
            //-------------- GATE OPTIONS [never, etc]  --------------
            case 2 :
                if( nextValue ) {
                  gateOption = gateOption.next();
                } // next

                if( prevValue ) {
                  gateOption = gateOption.prev();
                } // prev
                break;
            default : // recover from bad state
                initMenuSelected = 1;
                break;
        } // switch()

        // Update our telemetry
        telemetry.addData("ALLIANCE", "%s", ((redAlliance)? "RED":"BLUE"));
        telemetry.addData("Start Delay",  "%d sec %s", startDelaySec, ((initMenuSelected==1)? "<-":"  ") );
        telemetry.addData("Open Gate", "%s %s", gateOption.getDescription(), ((initMenuSelected==2)? "<-":"  ") );
        telemetry.addData("Odometry","x=%.2f y=%.2f  %.2f deg",
                robotGlobalXCoordinatePosition, robotGlobalYCoordinatePosition, Math.toDegrees(robotOrientationRadians) );
        telemetry.addLine("Preload=PPG (Green in front right!)");
        telemetry.addData(">","version 100" );
        telemetry.update();
    } // processAutonomousInitMenu

    /*--------------------------------------------------------------------------------------------*/
    // Resets odometry starting position and angle to zero accumulated encoder counts
    public void resetGlobalCoordinatePosition(){
//      robot.odom.resetPosAndIMU();   // don't need a full recalibration, just reset for any movement
        robot.odom.setOffsets(0.0, 0.0, DistanceUnit.MM);
//      robot.odom.setHeading( 180.0, AngleUnit.DEGREES ); // start pointing backward!
        robotGlobalXCoordinatePosition = 0.0;  // This will get overwritten the first time
        robotGlobalYCoordinatePosition = 0.0;  // we call robot.odom.update()!
        robotOrientationRadians        = 0.0;
    } // resetGlobalCoordinatePosition

    /*---------------------------------------------------------------------------------*/
    public void performEveryLoop() {
        robot.readBulkData();  // 7.3 msec for readBulkData
        robot.odom.update();   // 6.9 msec for odom.update + odom.getPosition (14 msec total)
        Pose2D pos = robot.odom.getPosition();  // x,y pos in inch; heading in degrees
        robotGlobalXCoordinatePosition = pos.getX(DistanceUnit.INCH);
        robotGlobalYCoordinatePosition = pos.getY(DistanceUnit.INCH);
        robotOrientationRadians        = pos.getHeading(AngleUnit.RADIANS);
        robot.processInjectionStateMachine();
        robot.processSpindexerControl();
    } // performEveryLoop

    public void processLimelightObelisk() {
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            // Access fiducial results
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                int limelightID = fr.getFiducialId();
                // Note: common OBELISK april tags for both RED & BLUE alliance
                //  21 = GPP (green purple purple)
                //  22 = PGP (purple green purple)
                //  23 = PPG (purple purple green)
                if( (limelightID >= 21) && (limelightID <= 23) ) {
                    telemetry.addData("Obelisk", "ID: %d", limelightID);
                    obeliskID = limelightID;
                }
            } // fiducialResults
        } // isValid

    } // processLimelightObelisk

    /*---------------------------------------------------------------------------------*/
    // Create a time stamped folder in the Robot Control flash file storage
    public void createAutoStorageFolder( boolean isRed, boolean isFar ) {
        // Create a subdirectory based on DATE
//      String timeString = new SimpleDateFormat("hh-mm-ss", Locale.getDefault()).format(new Date());
        String dateString = new SimpleDateFormat("yyyy-MM-dd", Locale.getDefault()).format(new Date());
        storageDir = Environment.getExternalStorageDirectory().getPath() + "//FIRST//Webcam//" + dateString;

        if (isRed) {
            if (isFar) {
                storageDir += "//red_far//";
            } else {
                storageDir += "//red_near//";
            }
        } else {
            if (isFar) {
                storageDir += "//blue_far//";
            } else {
                storageDir += "//blue_near//";
            }
        }
        // If we save more than one file per Autonomous run, use both DATE & TIME
//      storageDir += timeString;

        // Create the directory structure to store the autonomous image used to start auto.
        File baseDir = new File(storageDir);
        baseDir.mkdirs();
    }

    /*---------------------------------------------------------------------------------*/
    void driveAndRotate(double drivePower, double turnPower) {
        double frontRight, frontLeft, rearRight, rearLeft, maxPower;

        frontLeft  = drivePower - turnPower;
        frontRight = drivePower + turnPower;
        rearLeft   = drivePower - turnPower;
        rearRight  = drivePower + turnPower;

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
    } // driveAndRotate

    /*---------------------------------------------------------------------------------------------
     * getAngle queries the current gyro angle
     * @return  current gyro angle (-179.9 to +180.0)
     */
    protected double getAngle() {
        // calculate error in -179.99 to +180.00 range  (
        double gryoAngle = robot.headingIMU();
        while (gryoAngle >   180.0) gryoAngle -= 360.0;
        while (gryoAngle <= -180.0) gryoAngle += 360.0;
        return gryoAngle;
    } // getAngle()

    /*---------------------------------------------------------------------------------------------
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          Positive error means the robot should turn LEFT (CCW) to reduce error.
     */
    protected double getError(double targetAngle) {
        // calculate error in -179 to +180 range  (
        double robotError = targetAngle - robot.headingIMU();
        while (robotError >  180.0)  robotError -= 360.0;
        while (robotError <= -180.0) robotError += 360.0;
        return robotError;
    } // getError()

    /*---------------------------------------------------------------------------------------------
     * getAngleError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          Positive error means the robot should turn LEFT (CCW) to reduce error.
     */
    protected double getAngleError(double targetAngle) {
        // calculate error in -179 to +180 range  (
        double robotError = targetAngle - robot.headingIMU();
        while (robotError >  180.0)  robotError -= 360.0;
        while (robotError <= -180.0) robotError += 360.0;
        return robotError;
    } // getAngleError()

    /*---------------------------------------------------------------------------------------------
     * returns desired steering force.  +/- 1 range.  positive = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return clippedSteering
     */
    protected double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    } // getSteer()

    /*---------------------------------------------------------------------------------------------
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn( double speed, double angle ) {

        // keep looping while we are still active, and not on heading.
        while( opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF, 0.0,0.0) ) {
            // Bulk-refresh the Hub1/Hub2 device status (motor status, digital I/O) -- FASTER!
            performEveryLoop();

            // update range sensor data
//          robot.updateRangeL();
//          robot.updateRangeR();
        }
    } // gyroTurn()

    /*---------------------------------------------------------------------------------------------
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // range check the provided angle
        if( (angle > 360.0) || (angle < -360.0) ) {
            angle = getAngle();  // maintain current angle
        }

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Bulk-refresh the Hub1/Hub2 device status (motor status, digital I/O) -- FASTER!
            performEveryLoop();
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF, 0.0,0.0);
//          telemetry.update();
        }

        // Stop all motion;
        robot.driveTrainMotorsZero();
    } // gyroHold

    /*---------------------------------------------------------------------------------------------
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @param left_right Movement left/right  (positive=left,    negative=right)
     * @param fore_aft   Movement forward/aft (positive=forward, negative=aft)
     * @return onHeading
     */
    boolean onHeading(double speed, double angle, double PCoeff, double left_right, double fore_aft ) {
        double  error;
        double  steer;
        double  baseSpeed;
        double  faSpeed;
        double  lrSpeed;
        boolean onTarget = false;
        double  frontLeft;
        double  frontRight;
        double  backLeft;
        double  backRight;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            frontLeft  = 0.0;  // desired angle achieved; shut down all 4 motors
            frontRight = 0.0;
            backLeft   = 0.0;
            backRight  = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            baseSpeed  = speed * steer;
            lrSpeed    = speed * left_right;  // scale left/right speed (-1.0 .. +1.0) using turn speed
            faSpeed    = speed * fore_aft;    // scale fore/aft   speed (-1.0 .. +1.0) using turn speed
            frontLeft  =  baseSpeed + lrSpeed;  // increases LEFT speed
            frontRight = -baseSpeed + lrSpeed;  // decreased RIGHT speed (less negative)
            backLeft   = frontLeft  - faSpeed;  // decreases
            backRight  = frontRight + faSpeed;  // increases
        }

        // Send desired speeds to motors.
        robot.frontLeftMotor.setPower(frontLeft);
        robot.frontRightMotor.setPower(frontRight);
        robot.rearLeftMotor.setPower(backLeft);
        robot.rearRightMotor.setPower(backRight);

        // Display drive status for the driver.
        if( false ) {
            // Display it for the driver.
            telemetry.addData("Target", "%5.2f", angle);
            telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
            telemetry.addData("Speed Front", "%5.2f:%5.2f", frontLeft, frontRight);
            telemetry.addData("Speed Back ", "%5.2f:%5.2f", backLeft, backRight);
            telemetry.update();
        }
        return onTarget;
    } // onHeading()

    /**
     *
     * @param fl Front left motor power
     * @param fr Front right motor power
     * @param bl Back left motor power
     * @param br Back right motor power
     * @return Scaling factor to make sure power level is set to less than 1
     */
    public double scalePower(double fl, double fr, double bl, double br) {
        double scaleFactor = 1.0;
        double maxValue = Math.max(Math.abs(fl), Math.max(Math.abs(fr),
                Math.max(Math.abs(bl), Math.abs(br))));
        if(maxValue > 1.0) {
            scaleFactor = 1.0 / maxValue;
        }
        return scaleFactor;
    }

    //============================ TIME-BASED NAVIGATION FUNCTIONS ============================

    /*---------------------------------------------------------------------------------------------
     * Method will drive straight for a specified time.
     * @param speed  Speed to set all motors, positive forward, negative backward
     * @param time   How long to drive (milliseconds)
     */
    public void timeDriveStraight( double speed, int time ) {
        motionTimer.reset();
        robot.driveTrainMotors(speed, speed, speed, speed);
        while(opModeIsActive() && (motionTimer.milliseconds() <= time)) {
            performEveryLoop();
        }
        robot.stopMotion();
    } // timeDriveStraight

    /*---------------------------------------------------------------------------------------------
     * Method will strafe straight for a specified time.
     * @param speed  Speed to set all motors, postive strafe left, negative strafe right
     * @param time   How long to strafe (milliseconds)
     */
    public void timeDriveStrafe( double speed, int time ) {
        motionTimer.reset();
        robot.driveTrainMotors(-speed, speed, speed, -speed);
        while(opModeIsActive() && (motionTimer.milliseconds() <= time)) {
            performEveryLoop();
        }
        robot.stopMotion();
    } // timeDriveStrafe

    //============================ IMU/GYRO-BASED NAVIGATION FUNCTIONS ============================

    /*---------------------------------------------------------------------------------------------
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive( double maxSpeed, boolean driveY, double distance, double angle, int driveType ) {
        double  error, steer;
        double  curSpeed, leftSpeed, rightSpeed, max;
        boolean reachedTarget;
        int     loopCount = 0;
        int     posTol = (driveType == DRIVE_TO)? 15:40;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Bulk-refresh the Hub1/Hub2 device status (motor status, digital I/O) -- FASTER!
            performEveryLoop();

            // range check the provided angle
            if( (angle > 360.0) || (angle < -360.0) ) {
                angle = getAngle();  // maintain current angle
            }

            // configure RUN_TO_POSITION drivetrain motor setpoints
            robot.setRunToPosition( driveY, distance );

            // start motion.
            maxSpeed = Range.clip(Math.abs(maxSpeed), -1.0, 1.0);
            robot.frontLeftMotor.setPower(maxSpeed);
            robot.frontRightMotor.setPower(maxSpeed);
            robot.rearLeftMotor.setPower(maxSpeed);
            robot.rearRightMotor.setPower(maxSpeed);

            // Allow movement to start before checking isBusy()
            sleep( 150 ); // 150 msec

            // keep looping while we are still active, and ALL motors are running.
            while( opModeIsActive() ) {

                // Bulk-refresh the Hub1/Hub2 device status (motor status, digital I/O) -- FASTER!
                performEveryLoop();

                int frontLeftError  = Math.abs(robot.frontLeftMotorTgt - robot.frontLeftMotorPos);
                int frontRightError = Math.abs(robot.frontRightMotorTgt - robot.frontRightMotorPos);
                int rearLeftError   = Math.abs(robot.rearLeftMotorTgt - robot.rearLeftMotorPos);
                int rearRightError  = Math.abs(robot.rearRightMotorTgt - robot.rearRightMotorPos);
                int avgError = (frontLeftError + frontRightError + rearLeftError + rearRightError) / 4;

                // 19.2:1 is 537 counts/rotation (18.85" distance).  1/2" tolerance = 14.24 counts
                reachedTarget = ((frontLeftError < posTol) && (frontRightError < posTol) &&
                                 (rearLeftError  < posTol) && (rearRightError  < posTol) );
//              reachedTarget = !robot.frontLeftMotor.isBusy() && !robot.frontRightMotor.isBusy() &&
//                              !robot.rearLeftMotor.isBusy() && !robot.rearRightMotor.isBusy();
                if (reachedTarget) break;

                // update range sensor data
//              robot.updateRangeL();
//              robot.updateRangeR();

                // reduce motor speed as we get close so we don't overshoot
                if (avgError < 75) {
                    curSpeed = 0.15;
                } else if (avgError < 300) {
                    curSpeed = 0.75 * maxSpeed;
                }
                else {
                    curSpeed = maxSpeed;
                }

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed  = curSpeed - steer;
                rightSpeed = curSpeed + steer;

                // Normalize speeds if any one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.frontLeftMotor.setPower(leftSpeed);
                robot.frontRightMotor.setPower(rightSpeed);
                robot.rearLeftMotor.setPower(leftSpeed);
                robot.rearRightMotor.setPower(rightSpeed);

                loopCount++; // still working, or stale data?

                // Display drive status for the driver.
                if( false ) {
                    telemetry.addData("loopCount", "%d", loopCount );
                    telemetry.addData("Err/St", "%5.1f/%5.3f", error, steer);
                    telemetry.addData("Target F", "%7d:%7d", robot.frontLeftMotorTgt, robot.frontRightMotorTgt);
                    telemetry.addData("Target R", "%7d:%7d", robot.rearLeftMotorTgt, robot.rearRightMotorTgt);
                    telemetry.addData("Error F", "%7d:%7d", frontLeftError, frontRightError);
                    telemetry.addData("Error R", "%7d:%7d", rearLeftError, rearRightError);
                    telemetry.addData("Speed", "%5.3f:%5.3f", leftSpeed, rightSpeed);
                    telemetry.update();
                }
            } // opModeIsActive

            if( driveType == DRIVE_TO ) {
                // Stop all motion
                robot.driveTrainMotorsZero();
            }

            // Turn off RUN_TO_POSITION
//          sleep( 100 ); // 100 msec delay before changing modes
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } // opModeIsActive()
    } // gyroDrive()

    //============================ ODOMETRY-BASED NAVIGATION FUNCTIONS ============================
    public void driveToPosition(double xTarget, double yTarget, double angleTarget,
                                double speedMax, double turnMax, int driveType) {
        // Loop until we get to destination.
        performEveryLoop();
        while(!driveToXY(-yTarget, xTarget, angleTarget,
                speedMax, driveType)
                && opModeIsActive()) {
            performEveryLoop();
            if( false ) {
                telemetry.addData("Drive", "x=%.1f, y=%.1f, %.1f deg",
                        robotGlobalXCoordinatePosition, robotGlobalYCoordinatePosition, Math.toDegrees(robotOrientationRadians));
                telemetry.update();
            }
        }

        // Fix the angle if we didn't reach angle in the drive
        if(driveType == DRIVE_TO) {
            rotateToAngle(angleTarget, turnMax);
        }
    }

    /*---------------------------------------------------------------------------------*/
    /**
     * @param angleTarget  - The angle the robot should try to face when reaching destination.
     * @param turnMax - Highest value to use for turn speed.
     */
    public void rotateToAngle(double angleTarget,
                              double turnMax) {
        // Move the robot away from the wall.
        performEveryLoop();
        rotateToAngle(angleTarget, true);
        // Loop until we get to destination.
        performEveryLoop();
        while(!rotateToAngle(angleTarget, false) && opModeIsActive()) {
            performEveryLoop();
        }
    }

    /*---------------------------------------------------------------------------------*/
    public void herdForwardQuickly(double yTarget, double xTarget, double angleTarget, double speedMax ) {
        double errorMultiplier = 0.04;
        double speedMin = MIN_DRIVE_MAGNITUDE;
        double allowedError = 1.5;
        performEveryLoop();
        // Loop until we get to destination.
        while(!driveToXY(yTarget, xTarget, angleTarget, speedMin, speedMax, errorMultiplier, allowedError, DRIVE_TO)
                && opModeIsActive()) {
            performEveryLoop();
        }
    } // herdForwardQuickly

    /*---------------------------------------------------------------------------------*/
    /**
     * @param angleTarget  - The angle the robot should try to face when reaching destination.
     * @param resetDriveAngle - When we start a new drive, need to reset the starting drive angle.
     * @return - Boolean true we have reached destination, false we have not
     */
    protected double lastDriveAngle;
    protected boolean rotateToAngle(double angleTarget, boolean resetDriveAngle) {
        boolean reachedDestination = false;
        double xMovement, yMovement, turnMovement;
        double errorMultiplier = 0.016;
        double turnMin = MIN_SPIN_RATE;
        double deltaAngle = AngleWrapRadians(toRadians(angleTarget) - robotOrientationRadians);
        double turnSpeed = Math.toDegrees(deltaAngle) * errorMultiplier;

        // This should be set on the first call to start us on a new path.
        if(resetDriveAngle) {
            lastDriveAngle = deltaAngle;
        }

        // We are done if we are within 2.0 degrees
        if(Math.abs(Math.toDegrees(deltaAngle)) < 2.0) {
            // We have reached our destination if the angle is close enough
            robot.stopMotion();
            reachedDestination = true;
            // We are done when we flip signs.
        } else if(lastDriveAngle < 0) {
            // We have reached our destination if the delta angle sign flips from last reading
            if(deltaAngle >= 0) {
                robot.stopMotion();
                reachedDestination = true;
            } else {
                // We still have some turning to do.
                xMovement = 0;
                yMovement = 0;
                if(turnSpeed > -turnMin) {
                    turnSpeed = -turnMin;
                }
                turnMovement = turnSpeed;
                ApplyMovement(yMovement, xMovement, turnMovement);
            }
        } else {
            // We have reached our destination if the delta angle sign flips
            if(deltaAngle <= 0) {
                robot.stopMotion();
                reachedDestination = true;
            } else {
                // We still have some turning to do.
                xMovement = 0;
                yMovement = 0;
                if(turnSpeed < turnMin) {
                    turnSpeed = turnMin;
                }
                turnMovement = turnSpeed;
                ApplyMovement(yMovement, xMovement, turnMovement);
            }
        }
        lastDriveAngle = deltaAngle;

        return reachedDestination;
    }

    /**
     * @param xTarget           - The X field coordinate to go to.
     * @param yTarget           - The Y field coordinate to go to.
     * @param angleTarget - The angle the robot should try to face when reaching destination in degrees.
     * @param speedMin    - The minimum speed that allows movement.
     * @param speedMax    - Sets the maximum speed to drive.
     * @param errorMultiplier - Sets the proportional speed to slow down.
     * @param errorAllowed - Sets the allowable error to claim target reached.
     * @param driveType - Allows waypoint to be a drive through where the robot won't slow down.
     * @return - Boolean true we have reached destination, false we have not
     */
    protected boolean driveToXY(double yTarget, double xTarget, double angleTarget, double speedMin,
                                double speedMax, double errorMultiplier, double errorAllowed,
                                int driveType) {
        boolean reachedDestination = false;
        double xWorld = robotGlobalXCoordinatePosition;  // inches
        double yWorld = robotGlobalYCoordinatePosition;  // inches
        double xMovement = 0.0, yMovement = 0.0, turnMovement = 0.0;
        // Not sure why, but the x and y are backwards
        double deltaX = xTarget - xWorld;
        double deltaY = yTarget - yWorld;
        double driveAngle = Math.atan2(deltaY, deltaX);
        double deltaAngle = AngleWrapRadians(toRadians(angleTarget) - robotOrientationRadians);
        double magnitude = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        double driveSpeed;
        double turnSpeed = Math.toDegrees(deltaAngle) * errorMultiplier;
        // Have to convert from world angles to robot centric angles.
        double robotDriveAngle = driveAngle - robotOrientationRadians;

        // This will allow us to do multi-point routes without huge slowdowns.
        // Such use cases will be changing angles, or triggering activities at
        // certain points.
        if(!(driveType == DRIVE_THRU)) {
            driveSpeed = magnitude * errorMultiplier;
        } else {
            driveSpeed = speedMax;
        }

        if(driveSpeed < speedMin) {
            driveSpeed = speedMin;
        } else if (driveSpeed > speedMax) {
            driveSpeed = speedMax;
        }

        // Check if we passed through our point
        if(magnitude <= errorAllowed) {
            reachedDestination = true;
            if(!(driveType == DRIVE_THRU)) {
                robot.stopMotion();
            } else {
                // This can happen if the robot is already at error distance for drive through
                xMovement = driveSpeed * Math.cos(robotDriveAngle);
                yMovement = driveSpeed * Math.sin(robotDriveAngle);
                turnMovement = turnSpeed;
                ApplyMovement(yMovement, xMovement, turnMovement);
            }
        } else {
            xMovement = driveSpeed * Math.cos(robotDriveAngle);
            yMovement = driveSpeed * Math.sin(robotDriveAngle);
            turnMovement = turnSpeed;
            ApplyMovement(yMovement, xMovement, turnMovement);
        }

        boolean ODOMETRY_DEBUG = false;
        if( ODOMETRY_DEBUG ) {
            sleep(250);       // allow 0.25 seconds of progress (from prior loop)...
            robot.driveTrainMotorsZero(); // then stop and observe
            telemetry.addData("World X (inches)", "%.2f in", xWorld );
            telemetry.addData("World Y (inches)", "%.2f in", yWorld );
            telemetry.addData("Orientation (deg)","%.2f deg", Math.toDegrees(robotOrientationRadians) );
            telemetry.addData("distanceToPoint", "%.2f in", magnitude);
            telemetry.addData("angleToPoint", "%.4f deg", Math.toDegrees(driveAngle));
            telemetry.addData("deltaAngleToPoint", "%.4f deg", Math.toDegrees(deltaAngle));
            telemetry.addData("relative_x_to_point", "%.2f in", deltaX);
            telemetry.addData("relative_y_to_point", "%.2f in", deltaY);
            //telemetry.addData("robot_radian_err", "%.4f deg", Math.toDegrees(robot_radian_err));
            telemetry.addData("movement_x_power", "%.2f", xMovement);
            telemetry.addData("movement_y_power", "%.2f", yMovement);
            telemetry.addData("rotation_power", "%.2f", turnMovement);
            telemetry.update();
            sleep(5000);  // so we can read the output above
        } // ODOMETRY_DEBUG

        return reachedDestination;
    }
    /**
     * @param yTarget     - The Y field coordinate to go to.
     * @param xTarget     - The X field coordinate to go to.
     * @param angleTarget - The angle the robot should try to face when reaching destination in degrees.
     * @param speedMax    - Sets the speed when we are driving through the point.
     * @param driveType   - Slows the robot down to stop at destination coordinate.
     * @return - Boolean true we have reached destination, false we have not
     */
    protected boolean driveToXY(double yTarget, double xTarget, double angleTarget,
                                double speedMax, int driveType) {

        // Convert from cm to inches
        double errorMultiplier = 0.033;  // ramp down from 100% starting at 30" from target
        double speedMin = MIN_DRIVE_MAGNITUDE;
        double allowedError = (driveType == DRIVE_THRU) ? 2.50 : 0.5;

        return (driveToXY(yTarget, xTarget, angleTarget, speedMin, speedMax, errorMultiplier,
                allowedError, driveType));
    }

    // Odometry updates
    private long lastUpdateTime = 0;

    /**converts xMovement, movement_x, movement_turn into motor powers */
    public void ApplyMovement(double yMovement, double xMovement, double turnMovement) {
        long currTime = SystemClock.uptimeMillis();
        if(currTime - lastUpdateTime < 16){
            return;
        }
        lastUpdateTime = currTime;

        // 2.1 is the ratio between the minimum power to strafe, 0.19, and driving, 0.09.
//        double frontLeft = movement_y-movement_turn+movement_x*1.5;
//        double backLeft = movement_y-movement_turn-movement_x*1.5;
//        double backRight = movement_y+movement_turn+movement_x*1.5;
//        double frontRight = movement_y+movement_turn-movement_x*1.5;
        double frontRight = yMovement - xMovement + turnMovement;
        double frontLeft  = yMovement + xMovement - turnMovement;
        double backRight  = yMovement + xMovement + turnMovement;
        double backLeft   = yMovement - xMovement - turnMovement;

        //find the maximum of the powers
        double maxRawPower = Math.abs(frontLeft);
        if(Math.abs(backLeft) > maxRawPower){ maxRawPower = Math.abs(backLeft);}
        if(Math.abs(backRight) > maxRawPower){ maxRawPower = Math.abs(backRight);}
        if(Math.abs(frontRight) > maxRawPower){ maxRawPower = Math.abs(frontRight);}

        //if the maximum is greater than 1, scale all the powers down to preserve the shape
        double scaleDownAmount = 1.0;
        if(maxRawPower > 1.0){
            //when max power is multiplied by this ratio, it will be 1.0, and others less
            scaleDownAmount = 1.0/maxRawPower;
        }
        frontLeft *= scaleDownAmount;
        backLeft *= scaleDownAmount;
        backRight *= scaleDownAmount;
        frontRight *= scaleDownAmount;

        //now we can set the powers ONLY IF THEY HAVE CHANGED TO AVOID SPAMMING USB COMMUNICATIONS
        robot.driveTrainMotors(frontLeft, frontRight, backLeft, backRight);
    }

    /*--------------------------------------------------------------------------------------------*/
    /**
     * Move robot to specified target position/orientation
     * @param yTarget (inches)
     * @param xTarget (inches)
     * @param angleTarget (degrees; 0deg is straight ahead)
     * @param speedMax
     * @param turnMax
     * @param driveType - DRIVE_TO goes for accuracy and stops all motors; DRIVE_THRU does not
     */
/*
    public void driveToPosition( double yTarget, double xTarget, double angleTarget,
                                  double speedMax, double turnMax, int driveType ) {
        // Loop until we reach the target (or autonomous program aborts)
        while( opModeIsActive() ) {
            // Tend to any automatic mechanism movement updates
            performEveryLoop();
            // Power drivetrain motors to move to where we WANT to be
            if( moveToPosition( yTarget, xTarget, angleTarget, speedMax, turnMax, driveType ) ) {
                // If this is the final destination, then stop all motors
                if( driveType == DRIVE_TO ) {
                    robot.driveTrainMotorsZero();
//                  sleep(30000);   // for debugging individual movements
                }
                // Break out of the loop
                break;
            }
        } // opModeIsActive()
    } // driveToPosition
*/

    /*--------------------------------------------------------------------------------------------*/
    /**
     * Compute instantaneous motor powers needed to move toward the specified target position/orientation
     * NOTE that this system uses X and Y directions OPPOSITE of globalCoordinatePositionUpdate so
     * that we can have straight-forward be 0deg (-90deg left to +90deg right) instead of 0deg left.
     * @param xTarget (inches) <-- pass in desired Y target!
     * @param yTarget (inches) <-- pass in desired X target
     * @param angleTarget (degrees)
     * @param speedMax
     * @param turnMax
     * @param driveType (DRIVE_TO or DRIVE_THRU)
     * @return boolean true/false for DONE?
     */
/*  public boolean moveToPosition( double xTarget, double yTarget, double angleTarget,
                                    double speedMax, double turnMax, int driveType ) {
        // Convert current robot X,Y position from encoder-counts to inches
        double x_world = robotGlobalYCoordinatePosition;  // inches (X/Y backward! see notes)
        double y_world = robotGlobalXCoordinatePosition;  // inches
        double angle_world = robotOrientationRadians;     // radians
        // Compute distance and angle-offset to the target point
        double distanceToPoint   = Math.sqrt( Math.pow((xTarget - x_world),2.0) + Math.pow((yTarget - y_world),2.0) );
        double distToPointAbs    = Math.abs( distanceToPoint );
        double angleToPoint      = (distToPointAbs < 0.001)? angle_world : Math.atan2( (yTarget - y_world), (xTarget - x_world) ); // radians
        double deltaAngleToPoint = AngleWrapRadians( angleToPoint - angle_world );       // radians
        // What tolerance do we use to declare that we're at the target? (inches)
        double xy_tolerance = (driveType == DRIVE_TO)? 0.25 : (3.5 * speedMax); // 10%=0.35"; 100%=3.5"
        // Compute x & y components required to move toward point (with angle correction)
        double relative_x_to_point = Math.cos(deltaAngleToPoint) * distanceToPoint;
        double relative_y_to_point = Math.sin(deltaAngleToPoint) * distanceToPoint;
        // Compute absolute-value x and y distances for scaling
        double relative_x_abs = Math.abs( relative_x_to_point );
        double relative_y_abs = Math.abs( relative_y_to_point );
        // Compute full movement power that preserves the shape/ratios of the intended movement direction
        double movement_x_power = (relative_x_to_point / (relative_y_abs + relative_x_abs)) * speedMax;
        double movement_y_power = (relative_y_to_point / (relative_y_abs + relative_x_abs)) * speedMax;
        // Are we looking for SPEED (DRIVE_THRU) or do we need ACCURATE final stopping position (DRIVE_TO)?
        if( driveType == DRIVE_TO ) {
           // At what distance should we start to reduce from full-power driving?
           double x_slowdown_distance = 10.0 * speedMax;  // 10% power =  1  inch; 100% power = 10 inches
           double y_slowdown_distance =  7.5 * speedMax;  // 10% power = 3/4 inch; 100% power = 7.5 inches
           double x_min_power = (relative_x_to_point > 0.0)? 0.10 : -0.10;
           double y_min_power = (relative_y_to_point > 0.0)? 0.10 : -0.10;
           // If we're inside the tolerance then min power can be 0.0
           if( relative_x_abs <= xy_tolerance ) x_min_power = 0.0;
           if( relative_y_abs <= xy_tolerance ) y_min_power = 0.0;
           // Is it time to start applying that slow-down to forward movement?
           if( relative_x_abs <= x_slowdown_distance )
              movement_x_power = (relative_x_to_point/x_slowdown_distance) * speedMax + x_min_power;
           // Is it time to start applying that slow-down to lateral movement?
           if( relative_y_abs <= y_slowdown_distance )
              movement_y_power = (relative_y_to_point/y_slowdown_distance) * speedMax + y_min_power;
        } // DRIVE_TO
        // Compute robot orientation-angle error
        double robot_radian_err = AngleWrapRadians( Math.toRadians(angleTarget) - angle_world );  // radians
        // Determine the proper angle-error tolerance (within this tolerance rotation_power drops to 0%)
        double angle_tolerance = 0.25;  // degrees (not enforced for DRIVE_THRU, but used to retain angle alignment)
        double smallAngleSpeed = 0.09;  // (0.19 .. 0.10) due to min_turn_power
        // Once angle-error is below 15deg, scale-down rotational power based on remaining error, but don't
        // drop below 10% (0.10) until we're inside the angle tolerance (when rotation_power goes to zero).
        double small_rad_error = Math.abs( robot_radian_err / Math.toRadians(15.0) );
        double min_turn_power = (robot_radian_err <= Math.toRadians(angle_tolerance))? 0.00 : 0.10;
        double adjusted_turn_power = (small_rad_error <= 1.0)? (small_rad_error * smallAngleSpeed + min_turn_power) : turnMax;
        double rotation_power = (robot_radian_err > 0.0)? adjusted_turn_power : -adjusted_turn_power;
        // Translate X,Y,rotation power levels into mecanum wheel power values
        // Note that this is the same math used for tele-op robot-centric driving
        // except "x" and "y" for the odometry are opposite that of the controller joystick
        // (assumes right motors are FORWARD; left motors are  REVERSE; positive power is FORWARD)
        double frontRight = movement_x_power - movement_y_power - rotation_power;
        double frontLeft  = movement_x_power + movement_y_power + rotation_power;
        double backRight  = movement_x_power + movement_y_power - rotation_power;
        double backLeft   = movement_x_power - movement_y_power + rotation_power;
        // Determine the maximum motor power
        double maxWheelPower = Math.max( Math.max( Math.abs(backLeft),  Math.abs(backRight)  ),
                                         Math.max( Math.abs(frontLeft), Math.abs(frontRight) ) );
        // Ensure no wheel powers exceeds 1.0 or 100%
        if( maxWheelPower > 1.00 )
        {
            backLeft   /= maxWheelPower;
            backRight  /= maxWheelPower;
            frontLeft  /= maxWheelPower;
            frontRight /= maxWheelPower;
        }
        boolean ODOMETRY_DEBUG = false;
        if( ODOMETRY_DEBUG ) {
            sleep(100);       // allow 0.1 seconds of progress (from prior loop)...
            robot.driveTrainMotorsZero(); // then stop and observe
            telemetry.addData("World X (inches)", "%.2f in", x_world );
            telemetry.addData("World Y (inches)", "%.2f in", y_world );
            telemetry.addData("Orientation (deg)","%.2f deg", Math.toDegrees(angle_world) );
            telemetry.addData("distanceToPoint", "%.2f in", distanceToPoint);
            telemetry.addData("angleToPoint", "%.4f deg", Math.toDegrees(angleToPoint));
            telemetry.addData("deltaAngleToPoint", "%.4f deg", Math.toDegrees(angleToPoint));
            telemetry.addData("relative_x_to_point", "%.2f in", relative_x_to_point);
            telemetry.addData("relative_y_to_point", "%.2f in", relative_y_to_point);
            telemetry.addData("robot_radian_err", "%.4f deg", Math.toDegrees(robot_radian_err));
            telemetry.addData("movement_x_power", "%.2f", movement_x_power);
            telemetry.addData("movement_y_power", "%.2f", movement_y_power);
            telemetry.addData("rotation_power", "%.2f", rotation_power);
            telemetry.update();
            sleep(3500);  // so we can read the output above
        } // ODOMETRY_DEBUG
        // Have we reached the desired target location/orientation?
        if( driveType == DRIVE_TO ) {
           if( (distanceToPoint <= xy_tolerance) && (Math.abs(robot_radian_err) <= Math.toRadians(angle_tolerance)) )
               return true;
        } // DRIVE_TO
        else {
           // NOTE: Depending on the move_power and turn_power specified, and the size of the
           // two changes needed (inches-of-travel & degrees-of-turn), it's possible that one may 
           // achieve the DRIVE_THRU tolerance when the other has not.  We assume the x-y distance
           // is the primary objective (and that we shouldn't STOP and wait for the rotation angle
           // to catch up) so we only confirm compliance to one and not both motion objectives.
           // We define  a tight angle tolerance for DRIVE_THRU so that we still attempt to "maintain"
           // the correct angle (when  possible) during  other DRIVE_THRU movements.
           if( distanceToPoint <= xy_tolerance )
               return true;
        } // DRIVE_THRU
        // NO, WE'RE NOT DONE! Update motor power settings
//      telemetry.addData("distanceToPoint", "%.2f in (x=%.2f y=%.2f)",
//              distanceToPoint, relative_x_to_point, relative_y_to_point );
//      telemetry.addData("motors", "%.2f %.2f %.2f %.2f",
//              frontLeft, frontRight, backLeft, backRight );
//      telemetry.update();
        robot.driveTrainMotors( frontLeft, frontRight, backLeft, backRight );
        return false;
    } // moveToPosition
*/
    /**
     * Ensure angle is in the range of -PI to +PI (-180 to +180 deg)
     * @param angleRadians
     * @return
     */
    public double AngleWrapRadians( double angleRadians ){
        while( angleRadians < -Math.PI ) {
            angleRadians += 2.0*Math.PI;
        }
        while( angleRadians > Math.PI ){
            angleRadians -= 2.0*Math.PI;
        }
        return angleRadians;
    }

    /**
     * Ensure angle is in the range of -180 to +180 deg (-PI to +PI)
     * @param angleDegrees
     * @return
     */
    public double AngleWrapDegrees( double angleDegrees ){
        while( angleDegrees < -180 ) {
            angleDegrees += 360.0;
        }
        while( angleDegrees > 180 ){
            angleDegrees -= 360.0;
        }
        return angleDegrees;
    } // AngleWrapDegrees

    /*--------------------------------------------------------------------------------------------*/
    public void collectSpikemarkFromFar( int spikeMark, boolean isRed ) {
        double x, y0,y1,y2,y3, ang;   // drive-to coordinates
        // Define field location X value for selected spikemark (COMMON to both RED and BLUE)
        switch( spikeMark ) {
           case 1  : x = 25.5; break;
           case 2  : x = 49.3; break;
           case 3  : x = 73.0; break;
           default : x = 25.5; break;
        } // switch()
        // Define Y locations and angle (MIRRORED for RED vs. BLUE)
        y0  = (isRed)? -12.6 : +12.6;
        y1  = (isRed)? -17.7 : +17.7;
        y2  = (isRed)? -22.9 : +22.9;
        y3  = (isRed)? -29.9 : +29.9;
        ang = (isRed)? -90.0 : +90.0;
        // Begin to execute the collection movements
        if( opModeIsActive() ) {
            // Drive to starting position
            driveToPosition( x,y0,ang, DRIVE_SPEED_20, TURN_SPEED_15, DRIVE_TO);
            // Turn on collector
            robot.intakeMotor.setPower(0.90);
            // Drive into the 1st ball to collect it
            driveToPosition( x,y1,ang, DRIVE_SPEED_20, TURN_SPEED_15, DRIVE_TO);
            // Spindex it
            if( robot.isRobot2 ) {
              robot.spinServoSetPositionCR(SPIN_DECREMENT);
            } else {
              robot.spinServoSetPosition(SPIN_DECREMENT);
            }
            // Drive into the 2nd ball to collect it
            driveToPosition( x,y2,ang, DRIVE_SPEED_20, TURN_SPEED_15, DRIVE_TO);
            // Spindex it
            if( robot.isRobot2 ) {
              robot.spinServoSetPositionCR(SPIN_DECREMENT);
            } else {
                robot.spinServoSetPosition(SPIN_DECREMENT);
            }
            // Drive into the 3rd ball to collect it
            driveToPosition( x,y3,ang, DRIVE_SPEED_20, TURN_SPEED_15, DRIVE_TO);
        } // opModeIsActive
    } // collectSpikemarkFromFar

    /*--------------------------------------------------------------------------------------------*/
    // TODO: we can/should merge this with scorePreloadBallsFromFar()
    public void scoreThreeBallsFromFar( int obeliskID, boolean isRed, double shooterPower ) {
        if( opModeIsActive() ) {
            // Drive to shooting location
            if( isRed ) {
              driveToPosition( 10.8, -0.4, -142.3, DRIVE_SPEED_20, TURN_SPEED_15, DRIVE_TO);
            } else {
              driveToPosition( 10.8, +0.4, +142.3, DRIVE_SPEED_20, TURN_SPEED_15, DRIVE_TO);
            }
            // Start to ramp up the shooter
            robot.shooterMotor1.setPower( shooterPower );
            robot.shooterMotor2.setPower( shooterPower );
            // Swivel the turret toward the RED or BLUE goal (do this instead of parking at 142deg??)
//          robot.turretServo.setPosition( (isRed)? 0.55 : 0.43 ); // right toward RED or left toward BLUE
            sleep(2000); // Wait for flywheels to reach speed
            // Convert the obelisk value into a shooting order
            HardwareSwyftBot.SpindexerState[] shootOrder = getObeliskShootOrder(obeliskID);
            // Shoot all 3 balls
            for(int i=0; i<shootOrder.length; i++) {
                // rotate (if necessary) to the next position
               if( robot.isRobot2 ) {
                 robot.spinServoSetPositionCR( shootOrder[i] );
               } else {
                 robot.spinServoSetPosition( shootOrder[i] );
               }
               // wait for the rotation to complete, then launch that ball
               sleep(2000 );
              launchBall();
              if( !opModeIsActive() ) break;
              }
        } // opModeIsActive
    } // scoreThreeBallsFromFar

    /*--------------------------------------------------------------------------------------------*/
    public void scorePreloadBallsFromFar(int obeliskID, boolean isRed, double shooterPower ) {
        if( opModeIsActive() ) {
            // Start to ramp up the shooter
            robot.shooterMotor1.setPower( shooterPower );
            robot.shooterMotor2.setPower( shooterPower );
            // Back up 12" (assumes robot starts facing the wall, not the obelisk)
            driveToPosition(-12.0, 0.0, 0.0, DRIVE_SPEED_10, TURN_SPEED_15, DRIVE_TO);
            // Swivel the turret toward the RED or BLUE goal
            robot.turretServo.setPosition( (isRed)? 0.55 : 0.43 ); // right toward RED or left toward BLUE
            // Turn on collector to help retain balls during spindexing
            robot.intakeMotor.setPower(0.90);
            sleep(4000 ); // Wait a bit longer for flywheels to reach speed
            // Convert the obelisk value into a shooting order
            HardwareSwyftBot.SpindexerState[] shootOrder = getObeliskShootOrder(obeliskID);
            // Shoot all 3 preloaded balls
            for(int i=0; i<shootOrder.length; i++) {
                // rotate (if necessary) to the next position
                robot.spinServoSetPosition( shootOrder[i] );
                // wait for the rotation to complete, then launch that ball
                sleep(2000 );
                launchBall();
                if( !opModeIsActive() ) break;
            }
        } // opModeIsActive
    } // scorePreloadBalls

    //--------------------------------------------------------------------------------------------
    HardwareSwyftBot.SpindexerState[] getObeliskShootOrder(int obeliskID) {
        // Note: common OBELISK april tags for both RED & BLUE alliance
        //  21 = GPP (green purple purple)
        //  22 = PGP (purple green purple)
        //  23 = PPG (purple purple green)

        // Based on our preload pattern:
        // SPIN_P2 = purple
        // SPIN_P1 = purple
        // SPIN_P3 = green

        switch (obeliskID) {
            case 21:
                return new HardwareSwyftBot.SpindexerState[] {SPIN_P3, SPIN_P2, SPIN_P1};
            case 22:
                return new HardwareSwyftBot.SpindexerState[] {SPIN_P2, SPIN_P3, SPIN_P1};
            case 23:
                return new HardwareSwyftBot.SpindexerState[] {SPIN_P2, SPIN_P1, SPIN_P3};
            default:
                return new HardwareSwyftBot.SpindexerState[0];
        }
    } // getObeliskShootOrder

    //--------------------------------------------------------------------------------------------
    public void launchBall(){
        robot.startInjectionStateMachine();
        do {
            sleep(50);
            if( !opModeIsActive() ) break;
            performEveryLoop();
        } while (robot.liftServoBusyU || robot.liftServoBusyD);
    } // launchBall

} // AutonomousBase
