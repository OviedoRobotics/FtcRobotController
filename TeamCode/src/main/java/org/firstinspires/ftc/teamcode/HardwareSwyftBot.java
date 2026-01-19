package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
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
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

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
    
    public boolean isRobot1 = false;  // 7592-C (see IMU initialization below)
    public boolean isRobot2 = false;  // 7592-D

    //====== INERTIAL MEASUREMENT UNIT (IMU) =====
    protected IMU imu          = null;
    public double headingAngle = 0.0;
    public double tiltAngle    = 0.0;

    //====== GOBILDA PINPOINT ODOMETRY COMPUTER ======
    GoBildaPinpointDriver odom;

    //====== LIMELIGHT SMART CAMERA ======
    public  Limelight3A limelight;
    private LLResult    llResultLast;

    /**
     * https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html#square-field-inverted-alliance-area
     * We currently use a field orientation that is 180º rotated from the standard FTC field,
     * (+x -> Obelisk, -x -> audience | +y -> blue goal, -y -> red goal)
     * so we have to adjust the values returned from the limelight camera (and the yaw fed back into it).
     */
    private static final boolean ROTATE_LIMELIGHT_FIELD_180 = true;

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

    // Absolute Position of Robot on the field.
    double robotGlobalXCoordinatePosition       = 0;   // inches
    double robotGlobalYCoordinatePosition       = 0;   // inches
    double robotOrientationDegrees              = 0;   // degrees

    double robotGlobalXvelocity                 = 0;   // inches/sec
    double robotGlobalYvelocity                 = 0;   // inches/sec
    double robotAngleVelocity                   = 0;   // degrees/sec

    double limelightFieldXpos     = 0;
    double limelightFieldYpos     = 0;
    double limelightFieldAngleDeg = 0;

    double limelightFieldXstd     = 0;
    double limelightFieldYstd     = 0;
    double limelightFieldAnglestd = 0;

    //====== 2025 DECODE SEASON MECHANISM MOTORS (RUN_USING_ENCODER) =====
    protected DcMotorEx intakeMotor     = null;

    public final static double INTAKE_FWD_COLLECT = +0.90;  // aggressively collect
    public final static double INTAKE_REV_REJECT  = -0.40;  // gently reject overcollect so don't send flying

    protected DcMotorEx shooterMotor1   = null;  // upper 
    protected DcMotorEx shooterMotor2   = null;  // lower
    public    double    shooterMotor1Vel = 0.0;  // encoder counts per second
    public    double    shooterMotor2Vel = 0.0;  // encoder counts per second
    public    double    shooterTargetVel = 0.0;  // encoder counts per second
    public    double    shooterMotor1Amps= 0.0;  // mA
    public    double    shooterMotor2Amps= 0.0;  // mA

    public    double      shooterMotorsSet   = 0.0;
    public    boolean     shooterMotorsReady = false; // Have we reached the target velocity?
    public    ElapsedTime shooterMotorsTimer = new ElapsedTime();
    public    double      shooterMotorsTime  = 0.0;   // how long it took to reach "ready" (msec)

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
    public Servo       turretServo     = null;  // 2 servos! (controlled together via Y cable)
    public AnalogInput turretServoPos1 = null;
    public AnalogInput turretServoPos2 = null;

    public double     turretServoSet    = 0.0;  // 5-turn servo commanded setpoint
    public double     turretServoGet    = 0.0;  // 5-turn servo queried setpoint
    public double     turretServoPos    = 0.0;  // 5-turn servo position (analog feedback)
    public boolean    turretServoIsBusy = false; // are we still moving toward position?

    // NOTE: Although the turret can spin to +180deg, the cable blocks the shooter hood exit
    // once you reach +55deg, so that's our effect MAX turret angle on the right side.
    public final static double TURRET_SERVO_MAX2 = 0.93; // +180 deg (turret max)
    public final static double TURRET_SERVO_P90  = 0.73; // +90 deg
    public final static double TURRET_SERVO_MAX  = 0.64; // +53deg
    public final static double TURRET_SERVO_INIT = 0.49; //   0 deg
    public final static double TURRET_SERVO_N90  = 0.29; // -90 deg
    public final static double TURRET_SERVO_MIN  = 0.06; // -180deg
    public final static double TURRET_CTS_PER_DEG = (TURRET_SERVO_P90 - TURRET_SERVO_N90)/180.0;

    public final static double TURRET_R1_OFFSET = -0.008; // ROBOT1 offset to align with reference
    public final static double TURRET_R2_OFFSET =  0.000; // ROBOT2 offset to align with reference

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

    //===== ROBOT1 spindexer servo positions:
    public final static double SPIN_SERVO_P1_R1 = 0.130;  // position 1
    public final static double SPIN_SERVO_P2_R1 = 0.500;  // position 2 (also the INIT position)
    public final static double SPIN_SERVO_P3_R1 = 0.880;  // position 3
    //===== ROBOT2 spindexer servo positions:
    public final static double SPIN_SERVO_P1_R2 = 0.105;  // position 1
    public final static double SPIN_SERVO_P2_R2 = 0.490;  // position 2 (also the INIT position)
    public final static double SPIN_SERVO_P3_R2 = 0.870;  // position 3
    //===== These get populated after IMU init, when we know if we're ROBOT1 or ROBOT2
    public double SPIN_SERVO_P1;    // position 1
    public double SPIN_SERVO_P2;    // position 2 (also the INIT position)
    public double SPIN_SERVO_P3;    // position 3

    public enum SpindexerState {
        SPIN_P1,
        SPIN_P2,
        SPIN_P3,
        SPIN_INCREMENT,
        SPIN_DECREMENT
    }
    
    public SpindexerState spinServoCurPos = SpindexerState.SPIN_P2;  // commanded spindexer enum
    public double         spinServoSetPos = 0.0;  // commanded spindexer position
    public boolean        spinServoInPos  = true; // have we reached the commanded position
    public ElapsedTime    spinServoTimer  = new ElapsedTime();
    public double         spinServoTime   = 0.0;  // msec to get into position

    //====== INJECTOR/LIFTER SERVO =====
    public Servo       liftServo      = null;
    public AnalogInput liftServoPos   = null;
    public boolean     liftServoBusyU = false;  // busy going UP (lifting)
    public boolean     liftServoBusyD = false;  // busy going DOWN (resetting)
    public ElapsedTime liftServoTimer = new ElapsedTime();

    //===== ROBOT1 injector/lift servo positions:
    public final static double LIFT_SERVO_INIT_R1   = 0.520;
    public final static double LIFT_SERVO_RESET_R1  = 0.520;
    public final static double LIFT_SERVO_INJECT_R1 = 0.330;
      //   173 (178)  . . .    (239)  235           <-- 5deg tolerance on RESET and INJECT
    public final static double LIFT_SERVO_RESET_ANG_R1  = 178.3;  // 0.520 = 173.3deg
    public final static double LIFT_SERVO_INJECT_ANG_R1 = 230.2;  // 0.330 = 235.2deg
    //===== ROBOT2 injector/lift servo positions:
    public final static double LIFT_SERVO_INIT_R2   = 0.510;
    public final static double LIFT_SERVO_RESET_R2  = 0.510;
    public final static double LIFT_SERVO_INJECT_R2 = 0.320;
      //   176.95 (181)  . . .    (234)  238.7           <-- 5deg tolerance on RESET and INJECT
    public final static double LIFT_SERVO_RESET_ANG_R2  = 181.0;  // 0.510 = 176.95deg
    public final static double LIFT_SERVO_INJECT_ANG_R2 = 234.0;  // 0.320 = 238.7deg
    //===== These get populated after IMU init, when we know if we're ROBOT1 or ROBOT2
    public double LIFT_SERVO_INIT;
    public double LIFT_SERVO_RESET;
    public double LIFT_SERVO_INJECT;
    public double LIFT_SERVO_RESET_ANG;
    public double LIFT_SERVO_INJECT_ANG;

    //====== LED CONTROLLERS (controlled via SERVO signals) =====
    public Servo  ledServo = null;   // goBilda RGB LED

    public final static double LED_INIT   = 0.000;  // off
    public final static double LED_RED    = 0.279;
    public final static double LED_GREEN  = 0.488;
    public final static double LED_BLUE   = 0.611;
    public final static double LED_PURPLE = 0.723;

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
        initIMU( isAutonomous );

        // define the spindexer servo positions (fine-tuned uniquely for each robot)
        SPIN_SERVO_P1 = (isRobot1)? SPIN_SERVO_P1_R1 : SPIN_SERVO_P1_R2;
        SPIN_SERVO_P2 = (isRobot1)? SPIN_SERVO_P2_R1 : SPIN_SERVO_P2_R2;
        SPIN_SERVO_P3 = (isRobot1)? SPIN_SERVO_P3_R1 : SPIN_SERVO_P3_R2;

        // define the shooter lift/injector servo positions (fine-tuned uniquely for each robot)
        LIFT_SERVO_INIT       = (isRobot1)? LIFT_SERVO_INIT_R1 : LIFT_SERVO_INIT_R2;
        LIFT_SERVO_RESET      = (isRobot1)? LIFT_SERVO_RESET_R1 : LIFT_SERVO_RESET_R2;
        LIFT_SERVO_INJECT     = (isRobot1)? LIFT_SERVO_INJECT_R1 : LIFT_SERVO_INJECT_R2;
        LIFT_SERVO_RESET_ANG  = (isRobot1)? LIFT_SERVO_RESET_ANG_R1 : LIFT_SERVO_RESET_ANG_R2;
        LIFT_SERVO_INJECT_ANG = (isRobot1)? LIFT_SERVO_INJECT_ANG_R1 : LIFT_SERVO_INJECT_ANG_R2;

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

        //--------------------------------------------------------------------------------------------
        // Locate the limelight3a camera in our hardware settings
        // NOTE: Control Hub is assigned eth0 address 172.29.0.1 by limelight DHCP server
        limelight = hwMap.get(Limelight3A.class, "limelight");

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
        intakeMotor = hwMap.get(DcMotorEx.class,"IntakeMotor");
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
        PIDFCoefficients shooterPIDF = new PIDFCoefficients( 280.0, 0.0, 0.0, 16.0 );
        shooterMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);
        shooterMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);

        // Initialize the servo on the shooter
        shooterServo    = hwMap.servo.get("shooterServo");          // servo port 0 (Control Hub)
//      shooterServoPos = hwMap.analogInput.get("shooterServoPos"); // Analog port ? (Control Hub)

        //--------------------------------------------------------------------------------------------
        // Initialize the servos that rotate the turret
        turretServo     = hwMap.servo.get("turretServo");            // servo port 2 (Control Hub)
        turretServoPos1 = hwMap.tryGet(AnalogInput.class, "turretServoPos1");
        turretServoPos2 = hwMap.tryGet(AnalogInput.class, "turretServoPos2");

        //--------------------------------------------------------------------------------------------
        // Initialize the servo on the spindexer
//      if( isRobot2 ) spinServoCR = hwMap.tryGet(CRServo.class, "spinServo");
        spinServo   = hwMap.tryGet(Servo.class, "spinServo");
        spinServoPos = hwMap.analogInput.get("spinServoPos");

        //--------------------------------------------------------------------------------------------
        // Initialize the servo for the injector/lifter
        liftServo    = hwMap.servo.get("liftServo");                // servo port 0 Expansion Hub)
        liftServoPos = hwMap.analogInput.get("liftServoPos");       // Analog port 1 (Control Hub)

        //--------------------------------------------------------------------------------------------
        // Initialize servo control of the goBilda LED
        ledServo = hwMap.tryGet(Servo.class, "ledServo");

        // Ensure all servos are in the initialize position (YES for auto; NO for teleop)
        if( isAutonomous ) {
           resetEncoders();
        }

    } /* init */

    /*--------------------------------------------------------------------------------------------*/
    // Resets odometry starting position and angle to the specified starting orientation
    // Needed to either start at zero for Teleop if we haven't run Autonomous first, or to
    // transfer any offset from autonomous to teleop if the frame of reference differs.
    public void resetGlobalCoordinatePosition( double posX, double posY, double posAngleDegree ){
//      robot.odom.resetPosAndIMU();   // don't need full recalibration; just reset our position in case of any movement
        setPinpointFieldPosition( posX, posY, posAngleDegree ); // in case we don't run autonomous first!
        robotGlobalXCoordinatePosition = posX;  // This will get overwritten the first time
        robotGlobalYCoordinatePosition = posY;  // we call robot.odom.update()!
        robotOrientationDegrees        = posAngleDegree;
    } // resetGlobalCoordinatePosition

    /*--------------------------------------------------------------------------------------------*/
    public void resetEncoders() throws InterruptedException {
        // Initialize the injector servo first! (so it's out of the way for spindexer rotation)
        liftServo.setPosition(LIFT_SERVO_INIT);
        turretServoSetPosition( TURRET_SERVO_INIT );
        shooterServo.setPosition(SHOOTER_SERVO_INIT);
        sleep(250);
        spinServoSetPosition(SpindexerState.SPIN_P3); // allows autonomous progression 3-2-1
        // Also initialize/calibrate the pinpoint odometry computer
        odom.resetPosAndIMU();
        imu.resetYaw();
    } // resetEncoders

    /*--------------------------------------------------------------------------------------------*/
    public void initIMU( boolean isAutonomous )
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
        // Define and initialize REV Expansion Hub IMU (common for both ROBOT1 and ROBOT2)
        LogoFacingDirection logoDirection = LogoFacingDirection.LEFT;
        UsbFacingDirection  usbDirection  = UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        if( isAutonomous ) {
            imu.resetYaw();
        }

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
        boolean shooterMotor1Ready = (Math.abs(shooterMotor1Vel - shooterTargetVel) < 20)? true:false;
        boolean shooterMotor2Ready = (Math.abs(shooterMotor2Vel - shooterTargetVel) < 20)? true:false;
        shooterMotorsReady = shooterMotor1Ready && shooterMotor2Ready; // FIXME: is this a good threshold?
        if( shooterMotorsReady && (shooterMotorsTime == 0) ) {
            shooterMotorsTime = shooterMotorsTimer.milliseconds();
        }

        // Where has the turret been commanded to?
        turretServoGet   = turretServo.getPosition();
        // Where is the turret currently located?  (average the two feedback values)
        turretServoPos   = (getTurretPosition(true) + getTurretPosition(false))/2.0;
        boolean turretInPos = (Math.abs(turretServoPos - turretServoSet) < 0.01)? true:false;
        if(turretServoIsBusy && turretInPos ) { // FIXME: is this a good threshold?
            turretServoIsBusy = false;
        }
        // NOTE: motor mA data is NOT part of the bulk-read, so increases cycle time!
//      shooterMotor1Amps = shooterMotor1.getCurrent(MILLIAMPS);
//      shooterMotor2Amps = shooterMotor1.getCurrent(MILLIAMPS);
        // Has the spindexer reached the commanded position?
        double spindexerError = Math.abs( spinServoSetPos - getSpindexerPos() );
        if( !spinServoInPos && (spindexerError < 0.030) ) {
            spinServoTime = spinServoTimer.milliseconds();
            spinServoInPos = true;
        }
    } // readBulkData

    /*--------------------------------------------------------------------------------------------*/
    public void shooterMotorsSetPower( double shooterPower )
    {
        shooterMotor1.setPower( shooterPower );
        shooterMotor2.setPower( shooterPower );
        shooterMotorsSet = shooterPower;
        shooterTargetVel = computeShooterVelocity(shooterPower);
        // reset our "ready" flag and start a timer
        shooterMotorsReady = false;
        shooterMotorsTimer.reset();
        shooterMotorsTime = 0.0;
    } // shooterMotorsSetPower

    /*--------------------------------------------------------------------------------------------*/
    public void limelightPipelineSwitch( int pipeline_number )
    {
        limelight.pipelineSwitch( pipeline_number );
    } // limelightPipelineSwitch

    /*--------------------------------------------------------------------------------------------*/
    public void limelightStart()
    {
        // Start polling for data (skipping this has getLatestResult() return null results)
        limelight.start();
    } // limelightStart

    /*--------------------------------------------------------------------------------------------*/
    public void limelightStop()
    {
        // Start polling for data (skipping this has getLatestResult() return null results)
        limelight.stop();
    } // limelightStop

    //BRODY!!
    static double thetaMaxTurret = 375;
    static double thetaMinTurret = 0;
    static double thetaMaxFlapper = 355;
    static double thetaMinFlapper = 0;
    static double X_BIN_L = 0.6667; // in feet
    static double Y_BIN_L = 12;   // in feet
    static double LAUNCH_EXIT_SPEED = 22;
    static double Z_BIN = 3.23;
    static double Z_SHOOTER = 0.5;  // get actual measurement
    static double TURRET_SERVO_RELATIVE_0_ANGLE = 0;
    static double TURRET_SERVO_HORIZONTAL_ANGLE_INIT = TURRET_SERVO_INIT*(thetaMaxTurret - thetaMinTurret);

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
    public void turretServoSetPosition( double targetPosition )
    {
        if( isRobot1 ) {
            targetPosition += TURRET_R1_OFFSET;
        }
        if( isRobot2 ) {
            targetPosition += TURRET_R2_OFFSET;
        }
        turretServo.setPosition(targetPosition);
        
        // Store this setting so we can track progress of the turret motion
        turretServoSet    = targetPosition;
        turretServoIsBusy = true;
    } // turretServoSetPosition

    /*--------------------------------------------------------------------------------------------*/
    // currently limited to +/- 90deg
    public void setTurretAngle( double targetAngleDegrees )
    {
        // convert degrees into servo position setting centered around the init position.
        double targetAngleCounts = -(targetAngleDegrees * TURRET_CTS_PER_DEG) + TURRET_SERVO_INIT;
        // make sure it's within our safe range
        if( targetAngleCounts < TURRET_SERVO_N90 ) targetAngleCounts = TURRET_SERVO_N90;
        if( targetAngleCounts > TURRET_SERVO_MAX ) targetAngleCounts = TURRET_SERVO_MAX;
        // set both turret servos (connected on Y cable)
        turretServoSetPosition( targetAngleCounts );
    } // setTurretAngle

    /*--------------------------------------------------------------------------------------------*/
    // Due to the complexity of 5-turn servos and two robots (4 total position sensors) we don't  
    // attempt to convert the servo position from a 0..1 value to an actual 0..360deg value.  All
    // we need to know is that whatever position commanded (0..1) has been achieved according to
    // the analog feedback, and we can do that in the 0..1 domain
    // INPUT:  analog1?  (do we want to know the current feedback based on servo1 or servo2 sensor?
    public double getTurretPosition( boolean analog1 )
    {   // NOTE: the analog position feedback for the 5-turn AndyMark servos differs from Axon 3.3V
        final double MAX_ANALOG_VOLTAGE   = 2.88; // Volts: maximum analog feedback (for 1.0)
        final double MIN_ANALOG_VOLTAGE   = 0.46; // Volts: minimum analog feedback (for 0.0) 1.66V = 0.5
        double measuredVoltage, scaledVoltage, positionFeedback;  // NOTE: 0.267 = -90   0.667 = +90
        // Which feedback does the user want?
        if( analog1 ) {
            measuredVoltage = (turretServoPos1 == null)? 0.0 : turretServoPos1.getVoltage();
        } else {
            measuredVoltage = (turretServoPos2 == null)? 0.0 : turretServoPos2.getVoltage();
        }
        // Convert min..max voltage into a 0..1 scale
        scaledVoltage = (measuredVoltage - MIN_ANALOG_VOLTAGE)/(MAX_ANALOG_VOLTAGE - MIN_ANALOG_VOLTAGE);
        // Ensure we remain within the 0.0 to 1.0 range
        scaledVoltage = Math.max(0.0, Math.min(1.0, scaledVoltage));
        // Invert, since voltage goes down as the 0...1 servo position setting goes up
        positionFeedback = 1.0 - scaledVoltage;
        return positionFeedback;
    } // getTurretPosition

    /*--------------------------------------------------------------------------------------------*/
    public void updatePinpointFieldPosition() {
        // Request an update from the Pinpoint odometry computer (single I2C read)
        odom.update();
        // Parse for x/y/angle position data
        Pose2D pos = odom.getPosition();  // x,y pos in inch; heading in degrees
        robotGlobalXCoordinatePosition = pos.getX(DistanceUnit.INCH);
        robotGlobalYCoordinatePosition = pos.getY(DistanceUnit.INCH);
        robotOrientationDegrees        = pos.getHeading(AngleUnit.DEGREES);
        // Parse for velocities (inches/sec, degrees/sec)
        robotGlobalXvelocity = odom.getVelX(DistanceUnit.INCH);
        robotGlobalYvelocity = odom.getVelY(DistanceUnit.INCH);
        robotAngleVelocity   = odom.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
        // Currently unused:
        // - Status         = odom.getDeviceStatus()
        // - Reference Rate = odom.getFrequency()
    } // updatePinpointFieldPosition

    /*--------------------------------------------------------------------------------------------*/
    public void setPinpointFieldPosition( double X, double Y, double headingDeg ) {
        Pose2D newFieldPosition = new Pose2D(DistanceUnit.INCH, X, Y, AngleUnit.DEGREES, headingDeg );
        odom.setPosition( newFieldPosition );
        robotGlobalXCoordinatePosition = X;
        robotGlobalYCoordinatePosition = Y;
        robotOrientationDegrees        = headingDeg;
    } // setPinpointFieldPosition

    /*--------------------------------------------------------------------------------------------*/
    public void updateLimelightFieldPosition() {
        // To get the most accurate estimate of field position from the limelight (using the
        // built-in field map and feedback from the Apriltags mounted on the red/blue goals)
        // we tell the limelight the current robot/camera orientation angle.
        double yawAngle = rotate180Yaw( robotOrientationDegrees );  // Rotate frame of reference!
        limelight.updateRobotOrientation( yawAngle );   // takes effect on next cycle...
        // Lets see if the limelight camera can see the Apriltag (to provide updated field location data)
        LLResult llResult = limelight.getLatestResult();
        if( llResult == null ) {
            // Nothing to process this cycle
            return;
        }
        if (llResultLast != null && (llResultLast == llResult) ) {
            // Already processed that one
            return;
        }
        if( !llResult.isValid() ) {
            // Can't see the AprilTag from here (clear our results)
            limelightFieldXpos     = 0;    limelightFieldXstd     = 0;
            limelightFieldYpos     = 0;    limelightFieldYstd     = 0;
            limelightFieldAngleDeg = 0;    limelightFieldAnglestd = 0;
            return;
        }
        int STALENESS_LIMIT_MS = 30;
        if( llResult.getStaleness() < STALENESS_LIMIT_MS ) {
            llResultLast = llResult;
            // Parse Limelight result for MegaTag2 robot pose data
            Pose3D   limelightBotpose = llResult.getBotpose_MT2();
            double[] stddev           = llResult.getStddevMt2();
            if (limelightBotpose != null) {
                // Obtain the X/Y position data
                Position limelightPosition = limelightBotpose.getPosition();
                // Translate X and Y into the odometry frame of reference
                double posX = rotate180XY( limelightPosition.x );
                double posY = rotate180XY( limelightPosition.y );
                // update our global tracking variables
                limelightFieldXpos = limelightPosition.unit.toInches(posX);
                limelightFieldXstd = stddev[0];
                limelightFieldYpos = limelightPosition.unit.toInches(posY);
                limelightFieldYstd = stddev[1];
                // Obtain the angle data
                YawPitchRollAngles limelightOrientation = limelightBotpose.getOrientation();
                limelightFieldAngleDeg = rotate180Yaw( limelightOrientation.getYaw(AngleUnit.DEGREES) );
                limelightFieldAnglestd = stddev[5];
            }
        } else {  // limelight data is stale, don't trust it
            limelightFieldXpos     = 0;    limelightFieldXstd     = 0;
            limelightFieldYpos     = 0;    limelightFieldYstd     = 0;
            limelightFieldAngleDeg = 0;    limelightFieldAnglestd = 0;
        }
    } // updateLimelightFieldPosition

    /*--------------------------------------------------------------------------------------------*/
    // The current pinpoint odometry is configured with a different +X/+Y/+angle than Limelight field
    private static double rotate180Yaw(double yaw) {
        if (!ROTATE_LIMELIGHT_FIELD_180) return yaw;
        double rotated = yaw + 180;
        double wrap = (rotated + 180) % 360;
        double shift = wrap - 180;
        return shift;
    } // rotate180Yaw

    private static double rotate180XY(double xy) {
        return ((ROTATE_LIMELIGHT_FIELD_180)? -xy : xy);
    } // rotate180XY

    /*--------------------------------------------------------------------------------------------*/
    public double getShootDistance(Alliance alliance) {
        double currentX = robotGlobalXCoordinatePosition;
        double currentY = robotGlobalYCoordinatePosition;
        // Positions for targets based on values from ftc2025DECODE.fmap
        double targetX = (alliance == Alliance.BLUE)? +60.0 : +60.0;  // 6ft = 72"
        double targetY = (alliance == Alliance.BLUE)? +60.0 : -60.0;  // 6ft = 72"
        // Compute distance to target point inside the goal
        double deltaX = targetX - currentX;
        double deltaY = targetY - currentY;
        double distance = Math.sqrt( Math.pow(deltaX,2) + Math.pow(deltaY,2) );
        return distance;
    } // getShootDistance

    /*--------------------------------------------------------------------------------------------*/
    // Convert distance from goal (inches) into a power setting for our shooter motors.
    // Four our shooter and field layout, the value should be between 0.45 and 0.59
    public double computeShooterPower(double x) {
        // power = 0.051 + (-2.53E-03)x + 3.9E-05x^2 + -1.21E-07x^3
        double shooterPower = 0.51 + -2.53E-3 * x + 3.9E-5 * Math.pow(x,2) + -1.21E-7 * Math.pow(x,3);
        shooterPower = Math.max(shooterPower, 0.45); // We should never be below 0.45
        shooterPower = Math.min(shooterPower, 0.60); // We should never exceed 0.60
        return shooterPower;
    } // computeShooterPower

    // Compute the expected shooter motor velocity [ticks/sec] for the specified power setting
    private double computeShooterVelocity(double shooterMotorsSet) {
        // velocity = -43396x^3 + 69296x^2 - 34252x + 6395.3
        double x = shooterMotorsSet;
        double velocity = 6395.3 + (-34252 * x) + (69296 * Math.pow(x,2)) + (-43396 * Math.pow(x,3));
        velocity = Math.max(velocity, 1040); // We should never be below 1040
        velocity = Math.min(velocity, 1400); // We should never exceed 1400
        return velocity;
    } // computeShooterVelocity

    /*--------------------------------------------------------------------------------------------*/
    public double getShootAngleDeg(Alliance alliance) {
        double currentX = robotGlobalXCoordinatePosition;
        double currentY = robotGlobalYCoordinatePosition;
        // Rotated field positions for targets based on values from ftc2025DECODE.fmap
        double targetX = (alliance == Alliance.BLUE)? +60.0 : +60.0;  // 6ft = 72"
        double targetY = (alliance == Alliance.BLUE)? +57.0 : -58.0;  // 6ft = 72"
        // Compute distance to target point inside the goal
        double deltaX = targetX - currentX;
        double deltaY = targetY - currentY;
        // Compute the angle assuming the robot is facing forward at 0 degrees
        double targetFromStraight = Math.toDegrees( Math.atan2(deltaY,deltaX) );
        // Adjust for the current robot orientation
        double shootAngle = targetFromStraight - robotOrientationDegrees;
        return shootAngle;
    } // getShootAngleDeg

    /*--------------------------------------------------------------------------------------------*/
    public double computeAxonPos( double measuredVoltage )
    {
        final double MAX_ANALOG_VOLTAGE   = 3.3;    // 3.3V maximum analog feedback output
        double measuredPos = (measuredVoltage / MAX_ANALOG_VOLTAGE);
        return measuredPos;
    } // computeAxonPos

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
    public double getSpindexerPos()
    {
        return computeAxonPos( spinServoPos.getVoltage() );
    } // getSpindexerAngle

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
            case SPIN_P1 :
                spinServoCurPos = SpindexerState.SPIN_P1;
                spinServoSetPos = SPIN_SERVO_P1;
                spinServo.setPosition(spinServoSetPos);
                break;
            case SPIN_P2 :
                spinServoCurPos = SpindexerState.SPIN_P2;
                spinServoSetPos = SPIN_SERVO_P2;
                spinServo.setPosition(spinServoSetPos);
                break;
            case SPIN_P3 :
                spinServoCurPos = SpindexerState.SPIN_P3;
                spinServoSetPos = SPIN_SERVO_P3;
                spinServo.setPosition(spinServoSetPos);
                break;
            case SPIN_INCREMENT :
                if( spinServoCurPos == SpindexerState.SPIN_P1 ) {
                    spinServoCurPos = SpindexerState.SPIN_P2;
                    spinServoSetPos = SPIN_SERVO_P2;
                    spinServo.setPosition(spinServoSetPos);
                }
                else if( spinServoCurPos == SpindexerState.SPIN_P2 ) {
                    spinServoCurPos = SpindexerState.SPIN_P3;
                    spinServoSetPos = SPIN_SERVO_P3;
                    spinServo.setPosition(spinServoSetPos);
                } // else no room to increment further!
                break;
            case SPIN_DECREMENT :
                if( spinServoCurPos == SpindexerState.SPIN_P3 ) {
                    spinServoCurPos = SpindexerState.SPIN_P2;
                    spinServoSetPos = SPIN_SERVO_P2;
                    spinServo.setPosition(spinServoSetPos);
                }
                else if( spinServoCurPos == SpindexerState.SPIN_P2 ) {
                    spinServoCurPos = SpindexerState.SPIN_P1;
                    spinServoSetPos = SPIN_SERVO_P1;
                    spinServo.setPosition(spinServoSetPos);
                } // else no room to increment further!
                break;
            default:
                break;
        } // switch()

        // reset our flag and start a timer
        spinServoInPos = false;
        spinServoTimer.reset();
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
            servoFullyInjected = (getInjectorAngle() >= LIFT_SERVO_INJECT_ANG);
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
