package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection;
import static java.lang.Thread.sleep;

/*
 * Hardware class for Swyft Robotics SWYFT DRIVE V2 chassis with 86mm mecanum wheels
 */
public class HardwareSwyftBot
{
    //====== REV CONTROL/EXPANSION HUBS =====
    LynxModule controlHub;
    LynxModule expansionHub;
    
    public boolean isRobot1 = false;  // see IMU initialization below
    public boolean isRobot2 = false;

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

    //BRODY!!
    // Left Corner is (0,0) Facing obelisk is 90deg
    double startingRobotGlobalXPosition = 72; // x inches
    double startingRobotGlobalYPosition = 24; // y inches
    double startingRobotOrientationDegrees = 90; // field orientation in deg

    Pose2D startingPos = new Pose2D(DistanceUnit.INCH, startingRobotGlobalXPosition, startingRobotGlobalYPosition, AngleUnit.DEGREES, startingRobotOrientationDegrees);

    // Absolute Position of Robot on the field.
    double robotGlobalXCoordinatePosition       = 0;   // inches
    double robotGlobalYCoordinatePosition       = 0;   // inches
    double robotOrientationDegrees              = 0;   // degrees 90deg (facing obelisk)
    //BRODY!!

    //====== 2025 DECODE SEASON MECHANISM MOTORS (RUN_USING_ENCODER) =====
    protected DcMotorEx intakeMotor     = null;

    protected DcMotorEx shooterMotor1   = null;  // upper 
    protected DcMotorEx shooterMotor2   = null;  // lower
    public    double    shooterMotor1Vel = 0.0; // encoder counts per second
    public    double    shooterMotor2Vel = 0.0; // encoder counts per second
    public    double    shooterMotor1Amps= 0.0; // mA
    public    double    shooterMotor2Amps= 0.0; // mA

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
    public Servo       turretServo    = null;  // 1 servos! (controlled together via Y cable)
    public AnalogInput turretServoPos = null;

    // NOTE: Although the turret can spin to +180deg, the cable blocks the shooter hood exit
    // once you reach +55deg, so that's our effect MAX turret angle on the right side.
    public final static double TURRET_SERVO_MAX2 = 0.93; // +180 deg (turret max)
    public final static double TURRET_SERVO_P90  = 0.73; // +90 deg
    public final static double TURRET_SERVO_MAX  = 0.64; // +55deg
    public final static double TURRET_SERVO_INIT = 0.49; //   0 deg
    public final static double TURRET_SERVO_N90  = 0.29; // -90 deg
    public final static double TURRET_SERVO_MIN  = 0.06; // -180deg

    //====== SPINDEXER SERVO =====
    public Servo       spinServo    = null;
    public CRServo     spinServoCR  = null;
    public AnalogInput spinServoPos = null;

    public enum SpindexerTargetPosition {
        P1(47),
        P2(167),
        P3(287);

        public final double degrees;

        SpindexerTargetPosition(double degrees) {
            this.degrees = degrees;
        }
    }

    public SpindexerTargetPosition currentSpindexerTarget = SpindexerTargetPosition.P1;

    public double spindexerPowerSetting = 0.0;

    private void cycleSpindexerTarget(int direction) {
        // direction: +1 increments, -1 decrements (with wraparound)
        SpindexerTargetPosition[] values = SpindexerTargetPosition.values();
        int index = currentSpindexerTarget.ordinal() + direction;
        if (index < 0) index = values.length - 1;
        if (index >= values.length) index = 0;
        currentSpindexerTarget = values[index];
    }

//  public final static double SPIN_SERVO_P1 = 0.13;    // position 1 ROBOT1
//  public final static double SPIN_SERVO_P2 = 0.50;    // position 2 (also the INIT position)
//  public final static double SPIN_SERVO_P3 = 0.88;    // position 3
    public final static double SPIN_SERVO_P1 = 0.105;   // position 1 ROBOT2
    public final static double SPIN_SERVO_P2 = 0.49;    // position 2 (also the INIT position)
    public final static double SPIN_SERVO_P3 = 0.87;    // position 3

    public enum SpindexerState {
        SPIN_P1,
        SPIN_P2,
        SPIN_P3,
        SPIN_INCREMENT,
        SPIN_DECREMENT
    }
    
    public SpindexerState spinServoCurPos = SpindexerState.SPIN_P2;

    //====== EYELID SERVOS =====
    public Servo       rEyelidServo      = null;   // right eyelid
    public Servo       lEyelidServo      = null;   // left eyelid
    public boolean     rEyelidServoBusyU = false;  // busy going UP   (opening)
    public boolean     lEyelidServoBusyU = false;  // busy going UP   (opening)
    public boolean     rEyelidServoBusyD = false;  // busy going DOWN (closing)
    public boolean     lEyelidServoBusyD = false;  // busy going DOWN (closing)
    public ElapsedTime rEyelidServoTimer = new ElapsedTime();
    public ElapsedTime lEyelidServoTimer = new ElapsedTime();

    public final static double R_EYELID_SERVO_INIT = 0.570;  // ROBOT2 only
    public final static double R_EYELID_SERVO_UP   = 0.570;
    public final static double L_EYELID_SERVO_INIT = 0.440;
    public final static double L_EYELID_SERVO_UP   = 0.440;
    public final static double R_EYELID_SERVO_DOWN = 0.295;
    public final static double L_EYELID_SERVO_DOWN = 0.700;

    public enum EyelidState {
        EYELID_OPEN_BOTH,
        EYELID_OPEN_R,
        EYELID_OPEN_L,
        EYELID_CLOSED_BOTH,
        EYELID_CLOSED_R,
        EYELID_CLOSED_L
    }

    //====== INJECTOR/LIFTER SERVO =====
    public Servo       liftServo      = null;
    public AnalogInput liftServoPos   = null;
    public boolean     liftServoBusyU = false;  // busy going UP (lifting)
    public boolean     liftServoBusyD = false;  // busy going DOWN (resetting)
    public ElapsedTime liftServoTimer = new ElapsedTime();

//  public final static double LIFT_SERVO_INIT   = 0.490;  // ROBOT1
//  public final static double LIFT_SERVO_RESET  = 0.490;
    public final static double LIFT_SERVO_INIT   = 0.500;  // ROBOT2
    public final static double LIFT_SERVO_RESET  = 0.500;
    public final static double LIFT_SERVO_INJECT = 0.310;
    //   179 (184)  . . .    (236)  241           <-- 5deg tolerance on RESET and INJECT
    public final static double LIFT_SERVO_RESET_ANG  = 184.0;  // 0.500 = 179.5deg
    public final static double LIFT_SERVO_INJECT_ANG = 236.0;  // 0.310 = 241.7deg
    //====== MOTIF CONSTANTS =====
    public enum MotifOptions {
        MOTIF_GPP,  // GREEN, PURPLE, PURPLE
        MOTIF_PGP,  // PURPLE, GREEN, PURPLE
        MOTIF_PPG   // PURPLE, PURPLE, GREEN
    }

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

        // Initialize REV Control Hub IMU
        // NOTE: call this first so it defines whether we're ROBOT1 or ROBOT2
        initIMU();

        //--------------------------------------------------------------------------------------------
        // Locate the odometry controller in our hardware settings
        odom = hwMap.get(GoBildaPinpointDriver.class,"odom");  // Expansion Hub I2C port 1
        odom.setOffsets(-84.88, -169.47, DistanceUnit.MM);     // odometry pod x,y offsets relative center of robot
        odom.setEncoderResolution( GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD ); // 4bar pods
        odom.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                                  GoBildaPinpointDriver.EncoderDirection.REVERSED);
        if( isAutonomous ) {
            odom.resetPosAndIMU();
        }

        // defines initial pose of robot on field.  white tip of far tape facing obelisk: x = 72in., y = 24in., orientation = 90deg.
        odom.setPosition(startingPos);  //BRODY!!

        //--------------------------------------------------------------------------------------------
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

        //--------------------------------------------------------------------------------------------
        // Define and Initialize intake motor (left side on ROBOT1, right side on ROBOT2)
        intakeMotor  = hwMap.get(DcMotorEx.class,"IntakeMotor");  // Expansion Hub port 2
        intakeMotor.setDirection( (isRobot2)? DcMotor.Direction.REVERSE :  DcMotor.Direction.FORWARD);
        intakeMotor.setPower( 0.0 );
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //--------------------------------------------------------------------------------------------
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

        //--------------------------------------------------------------------------------------------
        // Initialize the servos that rotate the turret
        turretServo    = hwMap.servo.get("turretServo");            // servo port 2 (Control Hub)
//      turretServoPos = hwMap.analogInput.get("turretServoPos");   // Analog port ? (Control Hub)

        //--------------------------------------------------------------------------------------------
        // Initialize the servo on the spindexer
//      if( isRobot2 ) spinServoCR = hwMap.tryGet(CRServo.class, "spinServo");
        spinServo   = hwMap.tryGet(Servo.class, "spinServo");  // both ROBOT1 and ROBOT2 !!
        spinServoPos = hwMap.analogInput.get("spinServoPos");       // Analog port 1 (Control Hub)

        //--------------------------------------------------------------------------------------------
        // Initialize the servos for the spindexer eyelids
        rEyelidServo = hwMap.tryGet(Servo.class, "rEyelidServo");
        lEyelidServo = hwMap.tryGet(Servo.class, "lEyelidServo");

        //--------------------------------------------------------------------------------------------
        // Initialize the servo for the injector/lifter
        liftServo    = hwMap.servo.get("liftServo");                // servo port 0 Expansion Hub)
        liftServoPos = hwMap.analogInput.get("liftServoPos");       // Analog port 1 (Control Hub)

        // Ensure all servos are in the initialize position (YES for auto; NO for teleop)
        if( isAutonomous ) {
           resetEncoders();
        }

    } /* init */

    /*--------------------------------------------------------------------------------------------*/
    //BRODY!!
    // Resets odometry starting position and angle to zero accumulated encoder counts
    public void resetGlobalCoordinatePosition(){
        robotGlobalXCoordinatePosition = 0.0;  // This will get overwritten the first time
        robotGlobalYCoordinatePosition = 0.0;  // we call robot.odom.update()!
        robotOrientationDegrees        = 0.0;
    } // resetGlobalCoordinatePosition

    /*--------------------------------------------------------------------------------------------*/
    public void resetEncoders() throws InterruptedException {
        // Initialize the injector servo first! (so it's out of the way for spindexer rotation)
        liftServo.setPosition(LIFT_SERVO_INIT);
        if( isRobot2 ) {
            lEyelidServo.setPosition(L_EYELID_SERVO_INIT);
            rEyelidServo.setPosition(R_EYELID_SERVO_INIT);
        }
        turretServo.setPosition(TURRET_SERVO_INIT);
        shooterServo.setPosition(SHOOTER_SERVO_INIT);
        sleep(250);
        spinServoSetPosition(SpindexerState.SPIN_P3); // allows autonomous progression 3-2-1
        // Also reset our odometry starting position
        odom.resetPosAndIMU();
    } // resetEncoders

    /*--------------------------------------------------------------------------------------------*/
    public void initIMU()
    {
        // Determine if we're running on ROBOT1 (7592-C) or ROBOT2 (7592-D) based on IMU name
        // (we use tryGet() instead of the standard get() to avoid an exception when not found)
        imu = hwMap.tryGet(IMU.class, "imu-robot1");
        if( imu != null ) {
            isRobot1 = true;
        } // imu_robot1
        else {
            imu = hwMap.tryGet(IMU.class, "imu-robot2");
            if( imu != null ) {
                isRobot2 = true;
            }
        } // imu_robot2
        // Define and initialize REV Expansion Hub IMU                     ROBOT2 : ROBOT1
        LogoFacingDirection logoDirection = (isRobot2)?  LogoFacingDirection.LEFT : LogoFacingDirection.RIGHT;
        UsbFacingDirection  usbDirection = (isRobot2)? UsbFacingDirection.FORWARD : UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
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
        shooterMotor1Vel = shooterMotor1.getVelocity();
        shooterMotor2Vel = shooterMotor2.getVelocity();
        // NOTE: motor mA data is NOT part of the bulk-read, so increases cycle time!
//      shooterMotor1Amps = shooterMotor1.getCurrent(MILLIAMPS);
//      shooterMotor2Amps = shooterMotor1.getCurrent(MILLIAMPS);
    } // readBulkData

    /*--------------------------------------------------------------------------------------------*/
    public void shooterMotorsSetPower( double shooterPower )
    {
        // TODO: start a timer so we can measure  how long the ramp-up takes
        shooterMotor1.setPower( shooterPower );
        shooterMotor2.setPower( shooterPower );
    } // shooterMotorsSetPower

    //BRODY!!
    static double thetaMaxTurret = 375;
    static double thetaMinTurret = 0;
    static double thetaMaxFlapper = 355;
    static double thetaMinFlapper = 0;
    static double STARTING_HEADING = 90;
    static double TEST_LAUNCH_X = 6;
    static double TEST_LAUNCH_Y = 2;
    static double X_BIN_L = 0.6667; // in feet
    static double Y_BIN_L = 12;   // in feet
    static double LAUNCH_EXIT_SPEED = 22;
    static double Z_BIN = 3.23;
    static double Z_SHOOTER = 0.5;  // get actual measurement
//  static double TURRET_SERVO_RELATIVE_0_ANGLE = 180; // ROBOT1
    static double TURRET_SERVO_RELATIVE_0_ANGLE = 0;   // ROBOT2
    static double TURRET_SERVO_HORIZONTAL_POSITION = TURRET_SERVO_INIT; // position of turret servo when turret is aligned with the back of the robot
    static double TURRET_SERVO_HORIZONTAL_ANGLE_INIT = TURRET_SERVO_INIT*(thetaMaxTurret - thetaMinTurret);
    static double SHOOTER_SERVO_POS_VERTICAL = 0.64;

    static double SHOOTER_SERVO_HORIZONTAL_POSITION = 0.39;
    public double computeAlignedTurretPos() {
        double deltaServoPos = (computeTurretAngle())/(thetaMaxTurret - thetaMinTurret); // servo 0->1 is clockwise
        return (deltaServoPos > TURRET_SERVO_P90 || deltaServoPos < TURRET_SERVO_N90)? turretServo.getPosition() : deltaServoPos;
    }

    public double computeTurretAngle() {
        // absolute heading of the robot relative to the field. 90 is facing obelisk (ccw is positive)
        double driveTrainHeading = robotOrientationDegrees;
        double xR = robotGlobalXCoordinatePosition/12.0; // convert to feet
        double yR = robotGlobalYCoordinatePosition/12.0; // convert to feet
        double xB = X_BIN_L;
        double yB = Y_BIN_L;

        double deltaHeading = calculateHeadingChange(xR, yR, xB, yB, driveTrainHeading);

        return deltaHeading;
    }

    public double calculateHeadingChange(double xR, double yR, double xB, double yB, double heading) {
        double angleToTarget = Math.atan2(yB-yR, xB-xR); // in radians
        // in radians. servo clockwise direction is positive need to multiply by negative one.
        double delta = -(angleToTarget - (Math.toRadians(heading) + Math.toRadians(TURRET_SERVO_RELATIVE_0_ANGLE)));
        // determine angle that the turret servo needs
        // to turn to and account for the offset of the angle of the turret servo from the robot.
        delta = Math.toDegrees(delta);
        delta += TURRET_SERVO_HORIZONTAL_ANGLE_INIT;
        // Normalize to [0, 360]
        if(delta < 0) delta += 360;
        if(delta > 360) delta -= 360;
        return delta;
    }

    public double computeAlignedFlapperPos() {
        double deltaServoPos = computeLaunchAngle()/(thetaMaxFlapper - thetaMinFlapper) + SHOOTER_SERVO_HORIZONTAL_POSITION;
        return SHOOTER_SERVO_INIT;
        //return (deltaServoPos > SHOOTER_SERVO_POS_VERTICAL || deltaServoPos < SHOOTER_SERVO_HORIZONTAL_POSITION)? shooterServo.getPosition() : deltaServoPos;
    }

    public double computeLaunchAngle() {
        double v = LAUNCH_EXIT_SPEED;
        double d = Math.sqrt((Math.pow((X_BIN_L - robotGlobalXCoordinatePosition/12.0), 2) + Math.pow((Y_BIN_L - robotGlobalYCoordinatePosition/12.0),2)));
        double h = Z_BIN - Z_SHOOTER;
        double g = 32.174;  // ft/sec/sec gravitational constant

        double discriminant = v * v * v * v - g * (g * d * d + 2 * v * v * h);

        // Check if a real solution exists
        if (discriminant < 0) return 999.9;

        double sqrtTerm = Math.sqrt(discriminant);

        // Two possible tangent values
        double tanTheta1 = (v * v + sqrtTerm) / (g * d);
        double tanTheta2 = (v * v - sqrtTerm) / (g * d);

        // Compute angles in radians (2 of them)
        double theta1 = Math.atan(tanTheta1);
        double theta2 = Math.atan(tanTheta2);

        // Ensure thetaUp > thetaDown
        double thetaUp = Math.max(theta1, theta2);
        double thetaDown = Math.min(theta1, theta2);

        return Math.toDegrees(thetaUp);
    } // computeAbsoluteAngle
    //BRODY!!

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
    public double computeAxonAngle( double measuredVoltage )
    {
        final double DEGREES_PER_ROTATION = 360.0;  // One full rotation measures 360 degrees
        final double MAX_ANALOG_VOLTAGE   = 3.3;    // 3.3V maximum analog feedback output
        // NOTE: when vertical the angle is 38.1deg, when horizontal 129.0 (prior to offset below)
        double measuredAngle = (measuredVoltage / MAX_ANALOG_VOLTAGE) * DEGREES_PER_ROTATION;
        // Enforce that any wrap-around remains in the range of 0 to 360 degrees
        while( measuredAngle <   0.0 ) measuredAngle += 360.0;
        while( measuredAngle > 360.0 ) measuredAngle -= 360.0;
        return measuredAngle;
    } // computeAxonAngle
    
    /*--------------------------------------------------------------------------------------------*/
    public double getSpindexerAngle()
    {
      return computeAxonAngle( spinServoPos.getVoltage() );
    } // getSpindexerAngle

    /*--------------------------------------------------------------------------------------------*/
    public double computeSpindexerError(double targetDeg, double actualDeg) {
        // Shortest angular error considering wrap-around at 360°
        double diff = targetDeg - actualDeg;
        // Normalize to -180..+180
        while (diff > 180) diff -= 360;
        while (diff <= -180) diff += 360;
        return diff;
    } // getSpindexerError

    /*--------------------------------------------------------------------------------------------*/
    double spindexerProportionalControl(double errorDeg) {
        final double MIN_POWER_TO_ROTATE = 0.08; // 8% servo power
        double rawPower;
        if (Math.abs(errorDeg) <= 1.5 )  {
            return 0.0;  // we're within our 3deg tolerance; stop
        }
        // If we're far away, scale with a higher proportional control
        if( Math.abs(errorDeg) >= 60.0 )
            rawPower = errorDeg * 0.008;   // 120deg error = 0.97 power
        else
            rawPower = errorDeg * 0.004;   // 60deg error = 0.24 power
        // Ensure minimum power to overcome stiction
        if( Math.abs(rawPower) < MIN_POWER_TO_ROTATE ) {
            rawPower = Math.signum(rawPower) * MIN_POWER_TO_ROTATE;
        }
        return Range.clip(rawPower, -0.97, 0.97 );
    } // spindexerProportionalControl

    public void processSpindexerControl() {
        // read current angle (0 to 360)
        double currentDegrees = getSpindexerAngle();
        // compute angular error from our target
        double error = computeSpindexerError(currentSpindexerTarget.degrees, currentDegrees);
        // convert the angular error to a proportional servo power
        spindexerPowerSetting = spindexerProportionalControl(error);
        spinServoCR.setPower( spindexerPowerSetting );
    } // processSpindexerControl

    /*--------------------------------------------------------------------------------------------*/
    public double getInjectorAngle()
    {
      return computeAxonAngle( liftServoPos.getVoltage() );
    } // getInjectorAngle

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
    public void spinServoSetPositionCR( SpindexerState position )
    {
        switch( position ) {
            case SPIN_P1 :
                currentSpindexerTarget = SpindexerTargetPosition.P1;
                break;
            case SPIN_P2 :
                currentSpindexerTarget = SpindexerTargetPosition.P2;
                break;
            case SPIN_P3 :
                currentSpindexerTarget = SpindexerTargetPosition.P3;
                break;
            case SPIN_INCREMENT :
                cycleSpindexerTarget(+1);
                break;
            case SPIN_DECREMENT :
                cycleSpindexerTarget(-1);
                break;
            default:
                break;
        } // switch()
    } // spinServoSetPositionCR

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
        boolean servoFullyInjected, servoFullyReset, servoTimeoutU, servoTimeoutD;
        // Process the LIFTING case (AxonMax+ no-load 60deg rotation = 115 msec
        if( liftServoBusyU ) {
            // Are we "done" because the servo position is now close enough? (Axon position feedback)
            if (isRobot1) {
                servoFullyInjected = false;
            } else { // robot2 has Axon position feedback wired up
                servoFullyInjected = (getInjectorAngle() >= LIFT_SERVO_INJECT_ANG);
            }
            servoTimeoutU = (liftServoTimer.milliseconds() > 750);
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
            // Are we "done" because the servo position is now close enough? (Axon position feedback)
            if (isRobot1) {
                servoFullyReset = false;
            } else { // robot2 has Axon position feedback wired up
                servoFullyReset = (getInjectorAngle() <= LIFT_SERVO_RESET_ANG);
            }
            servoTimeoutD = (liftServoTimer.milliseconds() > 500);
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

    /*--------------------------------------------------------------------------------------------*/
    public void eyelidServoSetPosition( EyelidState position )
    {
        // eventually use the rEyelidServoTimer and rEyelidServoBusyU to time the movement
        // so software protections can be put in place regard operation in a "bad state"
        
        switch( position ) {
            case EYELID_OPEN_BOTH :
               rEyelidServo.setPosition( R_EYELID_SERVO_UP );
               lEyelidServo.setPosition( L_EYELID_SERVO_UP );
               break;
            case EYELID_OPEN_R :
               rEyelidServo.setPosition( R_EYELID_SERVO_UP );
               break;
            case EYELID_OPEN_L :
               lEyelidServo.setPosition( L_EYELID_SERVO_UP );
               break;
            case EYELID_CLOSED_BOTH :
               rEyelidServo.setPosition( R_EYELID_SERVO_DOWN );
               lEyelidServo.setPosition( L_EYELID_SERVO_DOWN );
               break;
            case EYELID_CLOSED_R :
               rEyelidServo.setPosition( R_EYELID_SERVO_DOWN );
               break;
            case EYELID_CLOSED_L :
               lEyelidServo.setPosition( L_EYELID_SERVO_DOWN );
               break;
            default :
               break;
        } // switch()

    } // eyelidServoSetPosition
        
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
