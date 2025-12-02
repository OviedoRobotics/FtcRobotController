package org.firstinspires.ftc.teamcode;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection;
import static java.lang.Thread.sleep;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/*
 * Hardware class for Swyft Robotics SWYFT DRIVE V2 chassis with 86mm mecanum wheels
 */
public class HardwareSwyftBot
{
    //====== REV CONTROL/EXPANSION HUBS =====
    LynxModule controlHub;
    LynxModule expansionHub;

    //====== INERTIAL MEASUREMENT UNIT (IMU) =====
    protected IMU imu          = null;
    public double headingAngle = 0.0;
    public double tiltAngle    = 0.0;

    //====== GOBILDA PINPOINT ODOMETRY COMPUTER ======
    GoBildaPinpointDriver odom;

    //====== MECANUM DRIVETRAIN MOTORS (RUN_USING_ENCODER) =====
    protected DcMotorEx frontLeftMotor     = null;
    public int          frontLeftMotorTgt  = 0;       // RUN_TO_POSITION target encoder count
    public int          frontLeftMotorPos  = 0;       // current encoder count
    public double       frontLeftMotorVel  = 0.0;     // encoder counts per second
    public double       frontLeftMotorAmps = 0.0;     // current power draw (Amps)

    protected DcMotorEx frontRightMotor    = null;
    public int          frontRightMotorTgt = 0;       // RUN_TO_POSITION target encoder count
    public int          frontRightMotorPos = 0;       // current encoder count
    public double       frontRightMotorVel = 0.0;     // encoder counts per second
    public double       frontRightMotorAmps= 0.0;     // current power draw (Amps)

    protected DcMotorEx rearLeftMotor      = null;
    public int          rearLeftMotorTgt   = 0;       // RUN_TO_POSITION target encoder count
    public int          rearLeftMotorPos   = 0;       // current encoder count
    public double       rearLeftMotorVel   = 0.0;     // encoder counts per second
    public double       rearLeftMotorAmps  = 0.0;     // current power draw (Amps)

    protected DcMotorEx rearRightMotor     = null;
    public int          rearRightMotorTgt  = 0;       // RUN_TO_POSITION target encoder count
    public int          rearRightMotorPos  = 0;       // current encoder count
    public double       rearRightMotorVel  = 0.0;     // encoder counts per second
    public double       rearRightMotorAmps = 0.0;     // current power draw (Amps)

    public final static double MIN_DRIVE_POW      = 0.03;    // Minimum speed to move the robot
    public final static double MIN_TURN_POW       = 0.03;    // Minimum speed to turn the robot
    public final static double MIN_STRAFE_POW     = 0.04;    // Minimum speed to strafe the robot
    protected double COUNTS_PER_MOTOR_REV  = 28.0;    // goBilda Yellow Jacket Planetary Gear Motor Encoders
    // TODO: update COUNTS/REV for SwyftDrive motors!!
    protected double DRIVE_GEAR_REDUCTION  = 12.7;    // SwyftDrive 12.7:1 (475rpm) gear ratio
    protected double MECANUM_SLIPPAGE      = 1.01;    // one wheel revolution doesn't achieve 6" x 3.1415 of travel.
    protected double WHEEL_DIAMETER_INCHES = 3.38583; // (86mm) -- for computing circumference
    protected double COUNTS_PER_INCH       = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION * MECANUM_SLIPPAGE) / (WHEEL_DIAMETER_INCHES * 3.1415);
    // The math above assumes motor encoders.  For REV odometry pods, the counts per inch is different
    protected double COUNTS_PER_INCH2      = 1738.4;  // 8192 counts-per-rev / (1.5" omni wheel * PI)

    //====== 2025 DECODE SEASON MECHANISM MOTORS (RUN_USING_ENCODER) =====
    protected DcMotorEx intakeMotor     = null;

    protected DcMotorEx shooterMotor1   = null;  // upper 
    protected DcMotorEx shooterMotor2   = null;  // lower
    public    double    shooterMotorVel = 0.0; // encoder counts per second

    public final static double SHOOTER_MOTOR_FAR  = 0.55;
    public final static double SHOOTER_MOTOR_MID  = 0.45;
    public final static double SHOOTER_MOTOR_AUTO = 0.45;

    //====== SHOOTER DEFLECTOR SERVO =====
    public Servo       shooterServo    = null;
    public AnalogInput shooterServoPos = null;

    public final static double SHOOTER_SERVO_INIT = 0.50;   // straight up
    public final static double SHOOTER_SERVO_INIT_ANGLE = 180.0;
    public final static double SHOOTER_SERVO_MIN = 0.50;
    public final static double SHOOTER_SERVO_MIN_ANGLE = 180.0;
    public final static double SHOOTER_SERVO_MAX = 0.50;
    public final static double SHOOTER_SERVO_MAX_ANGLE = 180.0;

    public double shooterServoCurPos = SHOOTER_SERVO_INIT;

    //====== TURRET 5-turn SERVOS =====
    public Servo       turretServo1    = null;
    public Servo       turretServo2    = null;
    public AnalogInput turretServoPos1 = null;
    public AnalogInput turretServoPos2 = null;

    public final static double TURRET_SERVO_INIT = 0.49;
    public final static double TURRET_SERVO_P90 = 0.73;
    public final static double TURRET_SERVO_N90 = 0.29;
    public final static double TURRET_SERVO_MAX = 0.93; // +180
    public final static double TURRET_SERVO_MIN = 0.07; // -180

    //====== SPINDEXER SERVO =====
    public Servo       spinServo    = null;
    public AnalogInput spinServoPos = null;

    public final static double SPIN_SERVO_P1 = 0.13;    // position 1
    public final static double SPIN_SERVO_P2 = 0.50;    // position 2 (also the INIT position)
    public final static double SPIN_SERVO_P3 = 0.88;    // position 3

    public enum SpindexerState {
        SPIN_P1,
        SPIN_P2,
        SPIN_P3,
        SPIN_INCREMENT,
        SPIN_DECREMENT
    }
    
    public SpindexerState spinServoCurPos = SpindexerState.SPIN_P2;

    //====== INJECTOR/LIFTER SERVO =====
    public Servo       liftServo      = null;
    public AnalogInput liftServoPos   = null;
    public boolean     liftServoBusyU = false;  // busy going UP (lifting)
    public boolean     liftServoBusyD = false;  // busy going DOWN (resetting)
    public ElapsedTime liftServoTimer = new ElapsedTime();

    public final static double LIFT_SERVO_INIT   = 0.49;
    public final static double LIFT_SERVO_RESET  = 0.49;
    public final static double LIFT_SERVO_INJECT = 0.31;

    /* local OpMode members. */
    protected HardwareMap hwMap = null;
    private final ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareSwyftBot(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, boolean isAutonomous ) throws InterruptedException {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Configure REV control/expansion hubs for bulk reads (faster!)
        for (LynxModule module : hwMap.getAll(LynxModule.class)) {
            if(module.isParent()) {
                controlHub = module;
            } else {
                expansionHub = module;
            }
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Locate the odometry controller in our hardware settings
        odom = hwMap.get(GoBildaPinpointDriver.class,"odom");    // Expansion Hub I2C port 1
        odom.setOffsets(0.0, 0.0, DistanceUnit.MM);   // odometry pod x,y locations relative center of robot
//      odom.setOffsets(0.00, 0.00, DistanceUnit.MM);      // odometry pod x,y locations relative center of robot  2 2
        odom.setEncoderResolution( GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD ); // 4bar pods
        odom.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                                  GoBildaPinpointDriver.EncoderDirection.REVERSED);
        if( isAutonomous ) {
            odom.resetPosAndIMU();
        }

        // Define and Initialize drivetrain motors
        frontLeftMotor  = hwMap.get(DcMotorEx.class,"FrontLeft");  // Expansion Hub port 0 (FORWARD)
        frontRightMotor = hwMap.get(DcMotorEx.class,"FrontRight"); // Control Hub   port 0 (reverse)
        rearLeftMotor   = hwMap.get(DcMotorEx.class,"RearLeft");   // Expansion Hub port 1 (FORWARD)
        rearRightMotor  = hwMap.get(DcMotorEx.class,"RearRight");  // Control Hub   port 1 (reverse)

        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        rearRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set all drivetrain motors to zero power
        driveTrainMotorsZero();

        // Set all drivetrain motors to run WITH encoders.
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set all drivetrain motors to brake when at zero power
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Define and Initialize intake motor
        intakeMotor  = hwMap.get(DcMotorEx.class,"IntakeMotor");  // Expansion Hub port 2
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setPower( 0.0 );
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Define and Initialize the two 6000rpm shooter motors
        // NOTE ON SPEED CONTROL:
        // - RUN_WITHOUT_ENCODER is open-loop control where setPower() directly sets voltage
        //   (proportional to raw speed). This ignores encoder data for regulation, so speed
        //   varies with load and/or battery voltage level.
        // - RUN_USING_ENCODER is closed-loop velocity control using a built-in PID loop.
        //   Here, setPower() requests a velocity (in encoder ticks per second, scaled by
        //   max speed), and the motor controller adjusts power to maintain it. The encoder
        //   provides consistent speed under varying conditions.
        shooterMotor1  = hwMap.get(DcMotorEx.class,"ShooterMotor1");  // Control Hub port 2  (upper)
        shooterMotor2  = hwMap.get(DcMotorEx.class,"ShooterMotor2");  // Control Hub port 3  (lower)
        shooterMotor1.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor1.setPower( 0.0 );
        shooterMotor2.setPower( 0.0 );
        shooterMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor1.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor2.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.FLOAT);
        // NOTE ON PIDF CONTROL:  The PID coefficients (10/3/0) are the defaults.
        // Proportional (P) is increased to 200 as a result of the large shooter mass.
        // The feed-forward value of 12 is used to maintain speed control under
        // load (meaning when the ball enters the shooter and slows down the flywheel)
//      PIDFCoefficients shooterPIDF = new PIDFCoefficients( 10.0, 3.0, 0.0, 12.0 );
        PIDFCoefficients shooterPIDF = new PIDFCoefficients( 200.0, 3.0, 0.0, 0.0 );
        shooterMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);
        shooterMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);

        // Initialize the servo on the shooter
        shooterServo    = hwMap.servo.get("shooterServo");          // servo port 0 (Control Hub)
//      shooterServoPos = hwMap.analogInput.get("shooterServoPos"); // Analog port ? (Control Hub)

        // Initialize the servos that rotate the turret
        turretServo1    = hwMap.servo.get("turretServo1");          // servo port 2 (Control Hub)
//      turretServoPos1 = hwMap.analogInput.get("turretServoPos1"); // Analog port ? (Control Hub)
//      turretServo2    = hwMap.servo.get("turretServo2");          // servo port ? (Control Hub)
//      turretServoPos1 = hwMap.analogInput.get("turretServoPos2"); // Analog port ? (Control Hub)

        // Initialize the servo on the spindexer
        spinServo    = hwMap.servo.get("spinServo");                // servo port 4 (Control Hub)
//      spinServoPos = hwMap.analogInput.get("spinServoPos");       // Analog port ? (Control Hub)

        // Initialize the servo for the injector/lifter
        liftServo    = hwMap.servo.get("liftServo");                // servo port 0 Expansion Hub)
//      liftServoPos = hwMap.analogInput.get("liftServoPos");       // Analog port ? (Expansion Hub)

        // Ensure all servos are in the initialize position (YES for auto; NO for teleop)
        if( isAutonomous ) {
           resetEncoders();
        }

        // Initialize REV Control Hub IMU
        initIMU();

    } /* init */

    /*--------------------------------------------------------------------------------------------*/
    public void resetEncoders() throws InterruptedException {
        // Initialize the injector servo first! (so it's out of the way for spindexer rotation)
        liftServo.setPosition(LIFT_SERVO_INIT);
        turretServo1.setPosition(TURRET_SERVO_INIT);
//      turretServo2.setPosition(TURRET_SERVO_INIT);
        shooterServo.setPosition(SHOOTER_SERVO_INIT);
        sleep(250);
        spinServoSetPosition(SpindexerState.SPIN_P3); // allows autonomous progression 3-2-1
    } // resetEncoders

    /*--------------------------------------------------------------------------------------------*/
    public void initIMU()
    {
        // Define and initialize REV Expansion Hub IMU
        // This needs to be changed to match the orientation on the robot
        // robot v1:
//        LogoFacingDirection logoDirection = LogoFacingDirection.RIGHT;
//        UsbFacingDirection usbDirection = UsbFacingDirection.UP;
        // robot v2:
        LogoFacingDirection logoDirection = LogoFacingDirection.LEFT;
        UsbFacingDirection usbDirection = UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu = hwMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    } // initIMU()

    /*--------------------------------------------------------------------------------------------*/
    public double headingIMU()
    {
        Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        headingAngle = angles.firstAngle;
        tiltAngle = angles.secondAngle;
        return -headingAngle;  // degrees (+90 is CW; -90 is CCW)
    } // headingIMU

    /*--------------------------------------------------------------------------------------------*/
    public void readBulkData() {
        // For MANUAL mode, we must clear the BulkCache once per control cycle
        expansionHub.clearBulkCache();
        controlHub.clearBulkCache();
        // Get a fresh set of values for this cycle
        //   getCurrentPosition() / getTargetPosition() / getTargetPositionTolerance()
        //   getPower() / getVelocity() / getCurrent()
        shooterMotorVel = shooterMotor1.getVelocity();
        // NOTE: motor mA data is NOT part of the bulk-read, so increases cycle time!
//      shooterMotorAmps = shooterMotor1.getCurrent(MILLIAMPS);
    } // readBulkData

    /*--------------------------------------------------------------------------------------------*/
    public void driveTrainMotors( double frontLeft, double frontRight, double rearLeft, double rearRight )
    {
        frontLeftMotor.setPower( frontLeft );
        frontRightMotor.setPower( frontRight );
        rearLeftMotor.setPower( rearLeft );
        rearRightMotor.setPower( rearRight );
    } // driveTrainMotors

    /*--------------------------------------------------------------------------------------------*/
    public void driveTrainMotorsZero()
    {
        frontLeftMotor.setPower( 0.0 );
        frontRightMotor.setPower( 0.0 );
        rearLeftMotor.setPower( 0.0 );
        rearRightMotor.setPower( 0.0 );
    } // driveTrainMotorsZero

    /*--------------------------------------------------------------------------------------------*/
    public void stopMotion() {
        // Stop all motion;
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);
    }

    /*--------------------------------------------------------------------------------------------*/
    /* setRunToPosition()                                                                         */
    /* - driveY -   true = Drive forward/back; false = Strafe right/left                          */
    /* - distance - how far to move (inches).  Positive is FWD/RIGHT                              */
    public void setRunToPosition( boolean driveY, double distance )
    {
        // Compute how many encoder counts achieves the specified distance
        int moveCounts = (int)(distance * COUNTS_PER_INCH);

        // These motors move the same for front/back or right/left driving
        frontLeftMotorTgt  = frontLeftMotorPos  +  moveCounts;
        frontRightMotorTgt = frontRightMotorPos + (moveCounts * ((driveY)? 1:-1));
        rearLeftMotorTgt   = rearLeftMotorPos   + (moveCounts * ((driveY)? 1:-1));
        rearRightMotorTgt  = rearRightMotorPos  +  moveCounts;

        // Configure target encoder count
        frontLeftMotor.setTargetPosition(  frontLeftMotorTgt  );
        frontRightMotor.setTargetPosition( frontRightMotorTgt );
        rearLeftMotor.setTargetPosition(   rearLeftMotorTgt   );
        rearRightMotor.setTargetPosition(  rearRightMotorTgt  );

        // Enable RUN_TO_POSITION mode
        frontLeftMotor.setMode(  DcMotor.RunMode.RUN_TO_POSITION );
        frontRightMotor.setMode( DcMotor.RunMode.RUN_TO_POSITION );
        rearLeftMotor.setMode(   DcMotor.RunMode.RUN_TO_POSITION );
        rearRightMotor.setMode(  DcMotor.RunMode.RUN_TO_POSITION );
    } // setRunToPosition

    /*--------------------------------------------------------------------------------------------*/
    public void spinServoSetPosition( SpindexerState position )
    {
        switch( position ) {
            case SPIN_P1 : spinServo.setPosition(SPIN_SERVO_P1);
                spinServoCurPos = SpindexerState.SPIN_P1;
                break;
            case SPIN_P2 : spinServo.setPosition(SPIN_SERVO_P2);
                spinServoCurPos = SpindexerState.SPIN_P2;
                break;
            case SPIN_P3 : spinServo.setPosition(SPIN_SERVO_P3);
                spinServoCurPos = SpindexerState.SPIN_P3;
                break;
            case SPIN_INCREMENT :
                if( spinServoCurPos == SpindexerState.SPIN_P1 ) {
                    spinServo.setPosition(SPIN_SERVO_P2);
                    spinServoCurPos = SpindexerState.SPIN_P2;
                }
                else if( spinServoCurPos == SpindexerState.SPIN_P2 ) {
                    spinServo.setPosition(SPIN_SERVO_P3);
                    spinServoCurPos = SpindexerState.SPIN_P3;
                } // else no room to increment further!
                break;
            case SPIN_DECREMENT :
                if( spinServoCurPos == SpindexerState.SPIN_P3 ) {
                    spinServo.setPosition(SPIN_SERVO_P2);
                    spinServoCurPos = SpindexerState.SPIN_P2;
                }
                else if( spinServoCurPos == SpindexerState.SPIN_P2 ) {
                    spinServo.setPosition(SPIN_SERVO_P1);
                    spinServoCurPos = SpindexerState.SPIN_P1;
                } // else no room to increment further!
                break;
            default:
                break;
        } // switch()
    } // spinServoSetPosition

    /*--------------------------------------------------------------------------------------------*/
    public void startInjectionStateMachine()
    {
        // Command the lift/injection servo to the INJECT position
        liftServo.setPosition( LIFT_SERVO_INJECT );
        // Start a timer (in case we need to timeout)
        liftServoTimer.reset();
        // Set a flag indicating the liftServo is busy lifting UP
        liftServoBusyU = true;
        liftServoBusyD = false; // ensure the reset flag is cleared
    } // startInjectionStateMachine

    /*--------------------------------------------------------------------------------------------*/
    public void processInjectionStateMachine()
    {
        // Process the LIFTING case (AxonMax+ no-load 60deg rotation = 115 msec
        if( liftServoBusyU ) {
            boolean servoFullyInjected = false;  // need Axon position feedback!!
            boolean servoTimeoutU = (liftServoTimer.milliseconds() > 750);
            // Has the injector servo reached the desired position? (or timed-out?)
            if( servoFullyInjected || servoTimeoutU ) {
              liftServoBusyU = false;  // the UP phase is complete
              // Begin the DOWN/reset phase
              liftServo.setPosition( LIFT_SERVO_RESET );
              liftServoTimer.reset();
              liftServoBusyD = true;
              }
        } // UP
        
        // Process the RESETTING case (AxonMax+ no-load 60deg rotation = 115 msec
        if( liftServoBusyD ) {
            boolean servoFullyReset = false;  // need Axon position feedback!!
            boolean servoTimeoutD = (liftServoTimer.milliseconds() > 500);
            // Has the injector servo reached the desired position? (or timed-out?)
            if( servoFullyReset || servoTimeoutD ) {
              liftServoBusyD = false;  // the DOWN phase is complete
              liftServoBusyU = false;  // ensure the flag is cleared
              }
        } // DOWN
                
    } // processInjectionStateMachine

    public void abortInjectionStateMachine()
    {
       // if we don't want to wait for injection
       liftServo.setPosition( LIFT_SERVO_RESET );
       liftServoTimer.reset();
       liftServoBusyD = true;        
    } // abortInjectionStateMachine

    public void waitForInjector()
    {
       // Query this before attempting to rotate the spindexer, so we don't
       // try to rotate while the injector is raised and blocking the rotation
       for( int i=0; i<5; i++ ) {
           if( !liftServoBusyU && !liftServoBusyD ) break;
           // wait 100msec and try again
           try {
               sleep(100);
           } catch (InterruptedException e) {
               throw new RuntimeException(e);
           }
       }
       // TODO:  we don't really have to wait until the injector servo is fully reset.
       // The spindexer is safe to turn once the servo is below a given angle.  Once
       // servo position feedback is hooked up, we can check the current angle and
       // return as soon as it is below that safe angle.
        
    } // waitForInjector

    /*--------------------------------------------------------------------------------------------*/

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    } /* waitForTick() */

} /* HardwareSwyftBot */
