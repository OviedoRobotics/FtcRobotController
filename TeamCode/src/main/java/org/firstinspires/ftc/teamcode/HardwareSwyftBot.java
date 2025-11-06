package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
    protected BNO055IMU imu    = null;
    public double headingAngle = 0.0;
    public double tiltAngle    = 0.0;

    //====== GOBILDA PINPOINT ODOMETRY COMPUTER ======
    GoBildaPinpointDriver odom;

    //====== MECANUM DRIVETRAIN MOTORS (RUN_USING_ENCODER) =====
    protected DcMotorEx frontLeftMotor  = null;
    protected DcMotorEx frontRightMotor = null;
    protected DcMotorEx rearLeftMotor   = null;
    protected DcMotorEx rearRightMotor  = null;

    //====== 2025 DECODE SEASON MECHANISM MOTORS (RUN_USING_ENCODER) =====
    protected DcMotorEx intakeMotor     = null;

    protected DcMotorEx shooterMotor1   = null;  // upper 
    protected DcMotorEx shooterMotor2   = null;  // lower

    public final static double SHOOTER_MOTOR_FAR  = 0.55;
    public final static double SHOOTER_MOTOR_MID  = 0.45;
    public final static double SHOOTER_MOTOR_AUTO = 0.45;

    //====== SHOOTER DEFLECTOR SERVO =====
    public Servo       shooterServo    = null;
    public AnalogInput shooterServoPos = null;

    public final static double SHOOTER_SERVO_INIT = 0.610;   // straight up
    public final static double SHOOTER_SERVO_INIT_ANGLE = 180.0;
    public final static double SHOOTER_SERVO_MIN = 0.399;
    public final static double SHOOTER_SERVO_MIN_ANGLE = 180.0;
    public final static double SHOOTER_SERVO_MAX = 0.640;
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

    public enum spindexerStateEnum {
        SPIN_P1,
        SPIN_P2,
        SPIN_P3,
        SPIN_INCREMENT,
        SPIN_DECREMENT
    }
    
    public spindexerStateEnum spinServoCurPos = spindexerStateEnum.SPIN_P2;

    //====== INJECTOR/LIFTER SERVO =====
    public Servo       liftServo    = null;
    public AnalogInput liftServoPos = null;

    public final static double LIFT_SERVO_INIT   = 0.50;
    public final static double LIFT_SERVO_RESET  = 0.50;
    public final static double LIFT_SERVO_INJECT = 0.72;

    /* local OpMode members. */
    protected HardwareMap hwMap = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareSwyftBot(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, boolean isAutonomous ) {
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
        odom = hwMap.get(GoBildaPinpointDriver.class,"odom");      // Expansion Hub I2C port 1
//      odom.setOffsets(-144.00, +88.00, DistanceUnit.MM); // odometry pod x,y locations relative center of robot
        odom.setOffsets(0.00, 0.00, DistanceUnit.MM);      // odometry pod x,y locations relative center of robot  2 2
        odom.setEncoderResolution( GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD ); // 4bar pods
        odom.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        if( isAutonomous ) {
            odom.resetPosAndIMU();
        }

        // Define and Initialize drivetrain motors
        frontLeftMotor  = hwMap.get(DcMotorEx.class,"FrontLeft");  // Expansion Hub port 0 (REVERSE)
        frontRightMotor = hwMap.get(DcMotorEx.class,"FrontRight"); // Control   Hub port 0 (forward)
        rearLeftMotor   = hwMap.get(DcMotorEx.class,"RearLeft");   // Expansion Hub port 1 (REVERSE)
        rearRightMotor  = hwMap.get(DcMotorEx.class,"RearRight");  // Control   Hub port 1 (forward)

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
        intakeMotor  = hwMap.get(DcMotorEx.class,"IntakeMotor");  // Expansion Hub port 0 
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
        // NOTE ON PIDF CONTROL:  The PID coefficnets (10/3/0) are the defaults.
        // The feed-forward value of 12 is used to maintain speed control under
        // load (meaning when the ball enters the shooter and slows down the flywheel)
        PIDFCoefficients shooterPIDF = new PIDFCoefficients( 10.0, 3.0, 0.0, 12.0 );
        shooterMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);
        shooterMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);

        // Initialize the servo on the shooter
        shooterServo    = hwMap.servo.get("shooterServo");          // servo port 0 (Control Hub)
        shooterServoPos = hwMap.analogInput.get("shooterServoPos"); // Analog port ? (Control Hub)

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
        initializeServos();

        // Initialize REV Control Hub IMU
        initIMU();

    } /* init */

    /*--------------------------------------------------------------------------------------------*/
    public void initializeServos()
    {
        // Initialize the injector servo first! (so it's out of the way for spindexer rotation)
        liftServo.setPosition(LIFT_SERVO_INIT);
        turretServo1.setPosition(TURRET_SERVO_INIT);
//      turretServo2.setPosition(TURRET_SERVO_INIT);
        shooterServo.setPosition(SHOOTER_SERVO_INIT);
        spinServoSetPosition(spindexerStateEnum.SPIN_P2);
    } // initializeServos

    /*--------------------------------------------------------------------------------------------*/
    public void initIMU()
    {
        // Define and initialize REV Expansion Hub IMU
        BNO055IMU.Parameters imu_params = new BNO055IMU.Parameters();
        imu_params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu_params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu_params.calibrationDataFile = "BNO055IMUCalibration.json"; // located in FIRST/settings folder
        imu_params.loggingEnabled = false;
        imu_params.loggingTag = "IMU";
        imu_params.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize( imu_params );
    } // initIMU()

    /*--------------------------------------------------------------------------------------------*/
    public double headingIMU()
    {
        Orientation angles = imu.getAngularOrientation( AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES );
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


        // NOTE: motor mA data is NOT part of the bulk-read, so increases cycle time!
//      viperMotorAmps      = viperMotor.getCurrent(MILLIAMPS);
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
    public void spinServoSetPosition( spindexerStateEnum position )
    {
        switch( position ) {
            case SPIN_P1 : spinServo.setPosition(SPIN_SERVO_P1);
                           spinServoCurPos = spindexerStateEnum.SPIN_P1;
                           break;
            case SPIN_P2 : spinServo.setPosition(SPIN_SERVO_P2);
                           spinServoCurPos = spindexerStateEnum.SPIN_P2;
                           break;
            case SPIN_P3 : spinServo.setPosition(SPIN_SERVO_P3);
                           spinServoCurPos = spindexerStateEnum.SPIN_P3;
                           break;
            case SPIN_INCREMENT :
                           if( spinServoCurPos == spindexerStateEnum.SPIN_P1 ) {
                               spinServo.setPosition(SPIN_SERVO_P2);
                               spinServoCurPos = spindexerStateEnum.SPIN_P2;
                           }
                           else if( spinServoCurPos == spindexerStateEnum.SPIN_P2 ) {
                               spinServo.setPosition(SPIN_SERVO_P3);
                               spinServoCurPos = spindexerStateEnum.SPIN_P3;
                           } // else no room to increment further!
                           break;
            case SPIN_DECREMENT :
                           if( spinServoCurPos == spindexerStateEnum.SPIN_P3 ) {
                               spinServo.setPosition(SPIN_SERVO_P2);
                               spinServoCurPos = spindexerStateEnum.SPIN_P2;
                           }
                           else if( spinServoCurPos == spindexerStateEnum.SPIN_P2 ) {
                               spinServo.setPosition(SPIN_SERVO_P1);
                               spinServoCurPos = spindexerStateEnum.SPIN_P1;
                           } // else no room to increment further!
            
                           break;
        default:
                break;
        } // switch()
    } // spinServoSetPosition

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
