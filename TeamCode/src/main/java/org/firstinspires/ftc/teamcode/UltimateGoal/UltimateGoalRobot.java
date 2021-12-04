
package org.firstinspires.ftc.teamcode.UltimateGoal;

import android.os.SystemClock;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.HardwareDrivers.MaxSonarI2CXL;
import org.firstinspires.ftc.teamcode.HelperClasses.WayPoint;
import org.firstinspires.ftc.teamcode.RobotUtilities.MovementVars;
import org.firstinspires.ftc.teamcode.RobotUtilities.MyPosition;

import java.util.Arrays;

import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.copySign;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.min;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

/**
 *Created by 7592 Roarbots
 */
public class UltimateGoalRobot
{
//    public static double ULTRASONIC_POWERSHOT_LEFT_DISTANCE = 58.0;
    public static double ULTRASONIC_POWERSHOT_LEFT_DISTANCE = 53.0;
    public boolean rangeSensorsEnabled = false;  // enable only when needed (takes time!)
    public static WayPoint finalAutoPosition;
    public static int finalRightEncoder;
    public static int finalLeftEncoder;
    public static int finalStrafeEncoder;
    public static boolean autoExecuted;
    public double ultrasonicCorrection = 0.0;

    /* Public OpMode members. */
    public final static double WOBBLE_ARM_MIN = 0.150;
//    public final static double WOBBLE_ARM_MIN = 0.300;
    public final static double WOBBLE_ARM_MAX = 3.2;

    // Regular high goal shots
    public final static double SHOOT_VELOCITY = 1180;

    // Quad auto high goal stack shots
    public final static double QUAD_HIGH_SHOT_VELOCITY = 1160;

    // Quad auto long initial shot
    public final static double LONG_SHOT_VELOCITY = 1100;
    // public final static double LONG_SHOT_VELOCITY = 1080;

    // Power shots
    public final static double SHOOT_POWERSHOT_VELOCITY = 980;

    public final static double SHOOT_VELOCITY_ERROR = 40;
    public final static double THROTTLE_TIMEOUT = 7000;
    public final static double STRAFE_MULTIPLIER = 1.5;
    public final static double SLOW_STRAFE_MULTIPLIER = 1.5;
    public final static double MIN_FOUNDATION_SPIN_RATE = 0.19;
    public final static double MIN_FOUNDATION_DRIVE_RATE = 0.18;
    public final static double MIN_FOUNDATION_STRAFE_RATE = 0.19;
    public final static double MIN_SPIN_RATE = 0.07;
    public final static double MIN_DRIVE_RATE = 0.07;
    public final static double MIN_STRAFE_RATE = 0.06;
    public final static double MIN_DRIVE_MAGNITUDE = Math.sqrt(MIN_DRIVE_RATE*MIN_DRIVE_RATE+MIN_DRIVE_RATE*MIN_DRIVE_RATE);
    public final static double MIN_FOUNDATION_DRIVE_MAGNITUDE = Math.sqrt(MIN_FOUNDATION_DRIVE_RATE*MIN_FOUNDATION_DRIVE_RATE+MIN_FOUNDATION_DRIVE_RATE*MIN_FOUNDATION_DRIVE_RATE);

    // Robot Controller Config Strings
    public final static String IMU = "imu";
    public final static String FRONT_LEFT_MOTOR = "FrontLeft";
    public final static String FRONT_RIGHT_MOTOR = "FrontRight";
    public final static String REAR_LEFT_MOTOR = "RearLeft";
    public final static String REAR_RIGHT_MOTOR = "RearRight";
    public final static String INTAKE_MOTOR = "Intake";
    public final static String WOBBLE_MOTOR = "Wobble";
    public final static String SHOOTER_MOTOR = "Shooter";
    public final static String EMPTY_MOTOR = "Empty";
    public final static String CLAW_LEFT_SERVO = "ClawLeft";
    public final static String CLAW_RIGHT_SERVO = "ClawRight";
    public final static String FLAP_SERVO = "Flap";
    public final static String INJECTOR_SERVO = "Injector";
    public final static String INTAKE_PUSHER_SERVO = "IntakePusher";
    public final static String ARM_POT = "ArmPot";
    public String hub1;
    public String hub2;

    public enum FLAP_POSITION {
        POWERSHOT,
        HIGH_GOAL
    }
    public FLAP_POSITION flapPosition;
    public final static double FLAP_POWERSHOT = 0.620;
    public double powerShotOffset = 0.0;
    public final static double FLAP_HIGH_GOAL = 0.509;
//    public final static double FLAP_HIGH_GOAL = 0.507;
    public double highGoalOffset = 0.0;
    public double flapAngle;

    LynxModule controlHub;
    LynxModule expansionHub;

    private MaxSonarI2CXL sonarRangeL = null;   // Must include MaxSonarI2CXL.java in teamcode folder
    private MaxSonarI2CXL sonarRangeR = null;

    // These motors have the odometry encoders attached
    protected DcMotorEx intake = null;
    protected DcMotorEx wobble = null;
    protected DcMotorEx empty = null;

    // Other motors
    protected DcMotorEx frontLeft = null;
    protected DcMotorEx frontRight = null;
    protected DcMotorEx rearLeft = null;
    protected DcMotorEx rearRight = null;
    protected DcMotorEx shooter = null;

    // Servos
    protected Servo flap = null;
    protected Servo clawLeft = null;
    protected Servo clawRight = null;
    protected Servo injector = null;
    protected CRServo intakePusher = null;

    // Sensors
    protected BNO055IMU imu = null;
    protected AnalogInput armPot = null;

    // Tracking variables
    private static final int encoderClicksPerSecond = 2800;
    protected double frontLeftMotorPower = 0.0;
    protected double rearLeftMotorPower = 0.0;
    protected double frontRightMotorPower = 0.0;
    protected double rearRightMotorPower = 0.0;
    protected double shooterMotorTargetVelocity = 0;
    protected double intakeMotorPower = 0.0;
    protected double wobbleMotorPower = 0.0;

    public boolean defaultInputShaping = false;
    protected boolean imuRead = false;
    protected double imuValue = 0.0;
    protected double strafeMultiplier = STRAFE_MULTIPLIER;

    // Servo Timer variables
    private ElapsedTime clawTimer;
    private ElapsedTime flapTimer;
    private ElapsedTime injectTimer;
    private ElapsedTime wobbleTimer;

    public static boolean encodersReset = false;
    public boolean forceReset = false;
    public boolean disableDriverCentric = true;

    public WayPoint shootingError = new WayPoint(0, 0, Math.toRadians(0), 1.0);
    public WayPoint teleHighGoal;

    public WayPoint telePowerShotRight;
    public WayPoint telePowerShotCenter;
    public WayPoint telePowerShotLeft;
    public WayPoint powerShotCorrection = new WayPoint(0.0, 0.0, 0.0, 1.0);

    // Sonar variables
    private int      sonarRangeLIndex   = 0;                          // 0...4 (SampCnt-1)
    private double[] sonarRangeLSamples = {0,0,0,0,0};                // continuous sampling data (most recent 5)
    private int      sonarRangeLSampCnt = sonarRangeLSamples.length;  // 5
    private double   sonarRangeLMedian  = 0.0;                        // CM (divide by 2.54 for INCHES)
    public  double   sonarRangeLStdev   = 0.0;

    private int      sonarRangeRIndex   = 0;                          // 0...4 (SampCnt-1)
    private double[] sonarRangeRSamples = {0,0,0,0,0};                // continuous sampling data (most recent 5)
    private int      sonarRangeRSampCnt = sonarRangeRSamples.length;  // 5
    private double   sonarRangeRMedian  = 0.0;                        // CM (divide by 2.54 for INCHES)
    public  double   sonarRangeRStdev   = 0.0;

    /* local OpMode members. */
    protected HardwareMap hwMap  =  null;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // The hubs are used for resetting reads for bulk reads.
        for (LynxModule module : hwMap.getAll(LynxModule.class)) {
            if(module.isParent()) {
                controlHub = module;
            } else {
                expansionHub = module;
            }
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Define and Initialize Motors
        frontLeft = hwMap.get(DcMotorEx.class, FRONT_LEFT_MOTOR);
        frontRight = hwMap.get(DcMotorEx.class, FRONT_RIGHT_MOTOR);
        rearLeft = hwMap.get(DcMotorEx.class, REAR_LEFT_MOTOR);
        rearRight = hwMap.get(DcMotorEx.class, REAR_RIGHT_MOTOR);
        intake = hwMap.get(DcMotorEx.class, INTAKE_MOTOR);
        wobble = hwMap.get(DcMotorEx.class, WOBBLE_MOTOR);
        empty = hwMap.get(DcMotorEx.class, EMPTY_MOTOR);
        shooter = hwMap.get(DcMotorEx.class, SHOOTER_MOTOR);


        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearLeft.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.REVERSE);

        // Changing these values affect the direction of the encoder reads.
        wobble.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);
        empty.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        setAllDriveZero();

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wobble.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        empty.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set the stop mode
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wobble.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        empty.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Define and initialize servos
        intakePusher = hwMap.get(CRServo.class, INTAKE_PUSHER_SERVO);
        flap = hwMap.get(Servo.class, FLAP_SERVO);
        injector = hwMap.get(Servo.class, INJECTOR_SERVO);
        clawLeft = hwMap.get(Servo.class, CLAW_LEFT_SERVO);
        clawRight = hwMap.get(Servo.class, CLAW_RIGHT_SERVO);

        injector.setPosition(INJECTOR_HOME);
        clawLeft.setPosition(CLAW_LEFT_CLOSED);
        clawRight.setPosition(CLAW_RIGHT_CLOSED);
        clawClosed = true;
        setShooterFlapPowerShot();

        // Define and initialize timers.
        clawTimer = new ElapsedTime();
        flapTimer = new ElapsedTime();
        injectTimer = new ElapsedTime();
        wobbleTimer = new ElapsedTime();

        // Let's try to tweak the PIDs
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400,
                1, 1, 0, MotorControlAlgorithm.PIDF));

        // Define and initialize sensors
        armPot = hwMap.get(AnalogInput.class, ARM_POT);
        initIMU();

        //Instantiate Maxbotics ultrasonic range sensors (sensors wired to I2C ports)
        sonarRangeL = hwMap.get( MaxSonarI2CXL.class, "left_ultrasonic" );
        sonarRangeR = hwMap.get( MaxSonarI2CXL.class, "right_ultrasonic" );
    }

    public int getLeftEncoderWheelPosition() { return intake.getCurrentPosition();
    }

    public int getRightEncoderWheelPosition() {
        return wobble.getCurrentPosition();
    }

    public int getStrafeEncoderWheelPosition() {
        return empty.getCurrentPosition();
    }

    public void setInputShaping(boolean inputShapingEnabled) {
        defaultInputShaping = inputShapingEnabled;
    }

    public void initIMU()
    {
        // Init IMU code
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, IMU);
        imu.initialize(parameters);
    }

    public void resetReads() {
        //controlHub.clearBulkCache();
        expansionHub.clearBulkCache();
        // The IMU is handled separately because it uses I2C which is not part of the bulk read.
        imuRead = false;
    }

    public double readIMU()
    {
        if(!imuRead) {
            // Read IMU Code
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            imuValue = (double)angles.firstAngle;
            imuRead = true;
        }

        return imuValue;
    }

    public void setShooterFlapPowerShot() {
        flapAngle = FLAP_POWERSHOT + powerShotOffset;
        flap.setPosition(flapAngle);
        flapPosition = FLAP_POSITION.POWERSHOT;
    }

    public void setShooterFlapHighGoal() {
        flapAngle = FLAP_HIGH_GOAL + highGoalOffset;
        flap.setPosition(flapAngle);
        flapPosition = FLAP_POSITION.HIGH_GOAL;
    }

    public void shooterOn() {
        shooter.setVelocity(shooterMotorTargetVelocity);
    }

    public void shooterOnLongShotHighGoal() {
        if(shooterMotorTargetVelocity != LONG_SHOT_VELOCITY) {
            shooter.setVelocity(LONG_SHOT_VELOCITY);
            shooterMotorTargetVelocity = LONG_SHOT_VELOCITY;
        }
    }

    public void shooterOnQuadHighGoal() {
        if(shooterMotorTargetVelocity != QUAD_HIGH_SHOT_VELOCITY) {
            shooter.setVelocity(QUAD_HIGH_SHOT_VELOCITY);
            shooterMotorTargetVelocity = QUAD_HIGH_SHOT_VELOCITY;
        }
    }
    public void shooterOnHighGoal() {
        if(shooterMotorTargetVelocity != SHOOT_VELOCITY) {
            shooter.setVelocity(SHOOT_VELOCITY);
            shooterMotorTargetVelocity = SHOOT_VELOCITY;
        }
    }

    public void shooterOnPowershot() {
        if(shooterMotorTargetVelocity != SHOOT_POWERSHOT_VELOCITY) {
            shooter.setVelocity(SHOOT_POWERSHOT_VELOCITY);
            shooterMotorTargetVelocity = SHOOT_POWERSHOT_VELOCITY;
        }
    }

    public void shooterOff() {
        if(shooterMotorTargetVelocity != 0) {
            shooter.setVelocity(0);
            shooterMotorTargetVelocity = 0;
        }
    }

    public void setIntakeIn() {
        setIntakeMotorPower(1.0);
        intakePusher.setPower(1.0);
    }

    public void setIntakeOut() {
        setIntakeMotorPower(-1.0);
        intakePusher.setPower(-1.0);
    }

    public void setIntakeOff() {
        setIntakeMotorPower(0.0);
        intakePusher.setPower(0.0);
    }

    public void setIntakeMotorPower(double power) {
        if (abs(power - intakeMotorPower) > 0.005) {
            intakeMotorPower = power;
            intake.setPower(power);
        }
    }

    public void setWobbleMotorPower(double power) {
        if (abs(power - wobbleMotorPower) > 0.005) {
            wobbleMotorPower = power;
            wobble.setPower(power);
        }
    }

    public void setFrontLeftMotorPower(double power)
    {
        if(abs(power - frontLeftMotorPower) > 0.005)
        {
            frontLeftMotorPower = power;
            frontLeft.setPower(power);
        }
    }

    public void setRearLeftMotorPower(double power)
    {
        if(abs(power - rearLeftMotorPower) > 0.005)
        {
            rearLeftMotorPower = power;
            rearLeft.setPower(power);
        }
    }

    public void setFrontRightMotorPower(double power)
    {
        if(abs(power - frontRightMotorPower) > 0.005)
        {
            frontRightMotorPower = power;
            frontRight.setPower(power);
        }
    }

    public void setRearRightMotorPower(double power)
    {
        if(abs(power - rearRightMotorPower) > 0.005)
        {
            rearRightMotorPower = power;
            rearRight.setPower(power);
        }
    }

    public void setAllDrive(double power) {
        setFrontLeftMotorPower(power);
        setFrontRightMotorPower(power);
        setRearRightMotorPower(power);
        setRearLeftMotorPower(power);
    }

    public void setAllDriveZero()
    {
        setAllDrive(0.0);
    }

    /**
     *
     * @param xPower - -1.0 to 1.0 power in the X axis
     * @param yPower - -1.0 to 1.0 power in the Y axis
     * @param spin - -1.0 to 1.0 power to rotate the robot, reduced to MAX_SPIN_RATE
     * @param angleOffset - The offset from the gyro to run at, such as drive compensation
     */
    public void drive(double xPower, double yPower, double spin, double angleOffset, boolean inputShaping) {
        double gyroAngle = angleOffset;
        if(!disableDriverCentric) {
            gyroAngle += readIMU();
        }

        double joystickMagnitude = sqrt(xPower*xPower + yPower*yPower);
        double driveAngle = atan2(yPower, xPower);
        double robotDriveAngle = driveAngle - Math.toRadians(gyroAngle) + Math.toRadians(90);
        double newPower = driverInputShaping(joystickMagnitude, inputShaping);

        MovementVars.movement_turn = driverInputSpinShaping(spin, inputShaping);
        MovementVars.movement_x = newPower * cos(robotDriveAngle);
        MovementVars.movement_y = newPower * sin(robotDriveAngle);

		ApplyMovement();
    }

    protected double driverInputShaping( double valueIn, boolean inputShaping) {
        double aValue = 0.77;
        double valueOut = 0.0;

        if(valueIn == 0.0) {
            valueOut = 0.0;
        } else {
            if (inputShaping) {
                valueOut = aValue * Math.pow(valueIn, 3) + (1 - aValue) * valueIn;
                valueOut = Math.copySign(max(MIN_DRIVE_RATE, abs(valueOut)), valueOut);
            } else {
                valueOut = valueIn;
            }
        }

        return valueOut;
    }

    protected double driverInputSpinShaping( double valueIn, boolean inputShaping) {
        double aValue = 0.77;
        double valueOut;

        if(valueIn == 0.0) {
            valueOut = 0.0;
        } else {
            if (inputShaping) {
                valueOut = aValue * Math.pow(valueIn, 3) + (1 - aValue) * valueIn;
                valueOut = Math.copySign(max(MIN_SPIN_RATE, abs(valueOut)), valueOut);
            } else {
                valueOut = valueIn;
            }
        }

        return valueOut;
    }

    public void disableDriveEncoders()
    {
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetEncoders()
    {
        int sleepTime = 0;
        int encoderCount = frontLeft.getCurrentPosition();

        // The Odometry Encoders
        wobble.setMode(RunMode.STOP_AND_RESET_ENCODER);
        empty.setMode(RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(RunMode.STOP_AND_RESET_ENCODER);

        shooter.setMode(RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(RunMode.STOP_AND_RESET_ENCODER);

        while((encoderCount != 0) && (sleepTime < 1000)) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) { break; }
            sleepTime += 10;
            resetReads();
            encoderCount = frontLeft.getCurrentPosition();
        }

        // The Odometry Encoders
        wobble.setMode(RunMode.RUN_WITHOUT_ENCODER);
        empty.setMode(RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(RunMode.RUN_WITHOUT_ENCODER);

        shooter.setMode(RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(RunMode.RUN_USING_ENCODER);
        frontRight.setMode(RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(RunMode.RUN_USING_ENCODER);
        rearRight.setMode(RunMode.RUN_USING_ENCODER);
    }

    /**
     * @param targetAngle  - The angle the robot should try to face when reaching destination.
     * @param pullingFoundation - If we are pulling the foundation.
     * @param resetDriveAngle - When we start a new drive, need to reset the starting drive angle.
     * @return - Boolean true we have reached destination, false we have not
     */
    public double lastDriveAngle;
    public boolean rotateToAngle(double targetAngle, boolean resetDriveAngle) {
        boolean reachedDestination = false;
        double errorMultiplier = 0.016;
        double minSpinRate = MIN_SPIN_RATE;
        double deltaAngle = MyPosition.AngleWrap(targetAngle - MyPosition.worldAngle_rad);
        double turnSpeed = -Math.toDegrees(deltaAngle) * errorMultiplier;

        // This should be set on the first call to start us on a new path.
        if(resetDriveAngle) {
            lastDriveAngle = deltaAngle;
        }

        // We are done if we are within 2 degrees
        if(abs(Math.toDegrees(deltaAngle)) < 0.5) {
            // We have reached our destination if the angle is close enough
            setAllDriveZero();
            reachedDestination = true;
        } else {
            // We still have some turning to do.
            MovementVars.movement_x = 0;
            MovementVars.movement_y = 0;
            turnSpeed = copySign(max(minSpinRate, abs(turnSpeed)), turnSpeed);
            MovementVars.movement_turn = turnSpeed;
            ApplyMovement();
        }
        lastDriveAngle = deltaAngle;

        return reachedDestination;
    }

    public double calculateLinearDriveSlowdown(double distance, double minSpeed, double maxSpeed, boolean passThrough) {
        double driveSpeed = 0.0;
        final double fullThrottleMinDistance = 100.0;

        // Full speed above fullThrottleMinRange
        if(passThrough || distance >= fullThrottleMinDistance) {
            driveSpeed = maxSpeed;
        } else {
            driveSpeed = distance / fullThrottleMinDistance;
        }

        driveSpeed = max(min(driveSpeed, maxSpeed), minSpeed);

        return driveSpeed;
    }

    public double calculateCubicDriveSlowdown(double distance, double minSpeed, double maxSpeed, boolean passThrough) {
        double driveSpeed = 0.0;
        final double curveSlope = 0.3;
        final double fullThrottleMinDistance = 80.0;

        // Full speed above fullThrottleMinRange
        if(passThrough || distance >= fullThrottleMinDistance) {
            driveSpeed = maxSpeed;
        } else {
            double valueIn = distance / fullThrottleMinDistance;
            driveSpeed = curveSlope * Math.pow(valueIn, 3) + (1.0 - curveSlope) * valueIn;
        }

        driveSpeed = max(min(driveSpeed, maxSpeed), minSpeed);

        return driveSpeed;
    }
    /**
     * @param x           - The X field coordinate to go to.
     * @param y           - The Y field coordinate to go to.
     * @param targetAngle - The angle the robot should try to face when reaching destination in radians.
     * @param minSpeed    - The minimum speed that allows movement.
     * @param maxSpeed    - Sets the maximum speed to drive.
     * @param errorMultiplier - Sets the proportional speed to slow down.
     * @param allowedError - Sets the allowable error to claim target reached.
     * @param passThrough - Allows waypoint to be a drive through where the robot won't slow down.
     * @return - Boolean true we have reached destination, false we have not
     */
    public boolean driveToXY(double x, double y, double targetAngle, double minSpeed,
                             double maxSpeed, double errorMultiplier, double allowedError,
                             boolean passThrough) {
        boolean reachedDestination = false;
        double deltaX = x - MyPosition.worldXPosition;
        double deltaY = y - MyPosition.worldYPosition;
        double driveAngle = Math.atan2(deltaY, deltaX);
        double deltaAngle = MyPosition.AngleWrap(targetAngle - MyPosition.worldAngle_rad);
        double magnitude = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        double driveSpeed = calculateLinearDriveSlowdown(magnitude, minSpeed, maxSpeed, passThrough);
//        double driveSpeed = calculateCubicDriveSlowdown(magnitude, minSpeed, maxSpeed, passThrough);
        // Apparently last season angle was positive CW, this season CCW is positive.
        double turnSpeed = -Math.toDegrees(deltaAngle) * errorMultiplier;
        // Have to convert from world angles to robot centric angles.
        double robotDriveAngle = driveAngle - MyPosition.worldAngle_rad + Math.toRadians(-90);

        // Check if we passed through our point
        if(magnitude <= allowedError) {
            reachedDestination = true;
            if(!passThrough) {
                setAllDriveZero();
            } else {
                // This can happen if the robot is already at error distance for drive through
                MovementVars.movement_x = driveSpeed * Math.cos(robotDriveAngle);
                MovementVars.movement_y = driveSpeed * Math.sin(robotDriveAngle);
                MovementVars.movement_turn = turnSpeed;
                ApplyMovement();
            }
        } else {
            MovementVars.movement_x = driveSpeed * Math.cos(robotDriveAngle);
            MovementVars.movement_y = driveSpeed * Math.sin(robotDriveAngle);
            MovementVars.movement_turn = turnSpeed;
            ApplyMovement();
        }

        return reachedDestination;
    }

    // Odometry updates
    private long lastUpdateTime = 0;

    /**converts movement_y, movement_x, movement_turn into motor powers */
    public void ApplyMovement() {
        long currTime = SystemClock.uptimeMillis();
        if(currTime - lastUpdateTime < 16){
            return;
        }
        lastUpdateTime = currTime;

        // 2.1 is the ratio between the minimum power to strafe, 0.19, and driving, 0.09.
        double tl_power_raw = MovementVars.movement_y-MovementVars.movement_turn+MovementVars.movement_x*strafeMultiplier;
        double bl_power_raw = MovementVars.movement_y-MovementVars.movement_turn-MovementVars.movement_x*strafeMultiplier;
        double br_power_raw = -MovementVars.movement_y-MovementVars.movement_turn-MovementVars.movement_x*strafeMultiplier;
        double tr_power_raw = -MovementVars.movement_y-MovementVars.movement_turn+MovementVars.movement_x*strafeMultiplier;

        //find the maximum of the powers
        double maxRawPower = abs(tl_power_raw);
        if(abs(bl_power_raw) > maxRawPower){ maxRawPower = abs(bl_power_raw);}
        if(abs(br_power_raw) > maxRawPower){ maxRawPower = abs(br_power_raw);}
        if(abs(tr_power_raw) > maxRawPower){ maxRawPower = abs(tr_power_raw);}

        //if the maximum is greater than 1, scale all the powers down to preserve the shape
        double scaleDownAmount = 1.0;
        if(maxRawPower > 1.0){
            //when max power is multiplied by this ratio, it will be 1.0, and others less
            scaleDownAmount = 1.0/maxRawPower;
        }
        tl_power_raw *= scaleDownAmount;
        bl_power_raw *= scaleDownAmount;
        br_power_raw *= scaleDownAmount;
        tr_power_raw *= scaleDownAmount;

        //now we can set the powers ONLY IF THEY HAVE CHANGED TO AVOID SPAMMING USB COMMUNICATIONS
        setFrontLeftMotorPower(tl_power_raw);
        setFrontRightMotorPower(tr_power_raw);
        setRearRightMotorPower(br_power_raw);
        setRearLeftMotorPower(bl_power_raw);
    }

    // This function keeps a running tab on how many loops the shooter has been at target
    // velocity.
    public void updateShooterStability() {
        // Verify this loop is at target velocity.
        if(abs(shooter.getVelocity() - shooterMotorTargetVelocity) <= SHOOT_VELOCITY_ERROR) {
            sequentialStableVelocityChecks++;
        } else {
            sequentialStableVelocityChecks = 0;
        }
    }

    /*--------------------------------------------------------------------------------------------*/
    /* NOTE ABOUT RANGE SENSORS:                                                                  */
    /* The REV 2m Range Sensor is really only 1.2m (47.2") maximum in DEFAULT mode. Depending on  */
    /* reflectivity of the surface encountered, it can be even shorter.  For example, the black   */
    /* metal paint on the field wall is highly absorptive, so we only get reliable range readings */
    /* out to 12" or so.  In contrast, the Maxbotics ultrasonic range sensors have a minimum      */
    /* range of 20 cm (about 8").  A combined Autonomous solution that requires both short (< 8") */
    /* and long (> 12-47") requires *both* REV Time-of-Flight (tof) range sensors and Maxbotics   */
    /* Ultrasonic range sensors. Also note that if you mount either ToF/Ultrasonic sensor too low */
    /* on the robot you'll get invalid distance readings due to reflections off the field tiles   */
    /* due to "fanout" of both laser/ultrasonic signals the further you get from the robot.       */
    /*--------------------------------------------------------------------------------------------*/

    // ULTRASONIC READINGS: The ultrasonic driver can be queried in two different update modes:
    // a) getDistanceSync()  sends a new ping and WAITS 50msec for the return
    // b) getDistanceAsync() sends a new ping and RETURNS IMMEDIATELY with the most recent valiue

    public double updateSonarRangeL() {
        // Query the current range sensor reading as the next sample to our LEFT range dataset
//      sonarRangeLSamples[ sonarRangeLIndex ] = sonarRangeL.getDistanceSync();
        sonarRangeLSamples[ sonarRangeLIndex ] = sonarRangeL.getDistanceAsync();
        if( ++sonarRangeLIndex >= sonarRangeLSampCnt ) sonarRangeLIndex = 0;
        // Create a duplicate copy that's sorted
        double[] sonarRangeLSorted = sonarRangeLSamples;
        Arrays.sort(sonarRangeLSorted);
        // Determine the running median (middle value of the last 5; assumes sonarRangeLSampCnt=5)
        sonarRangeLMedian = sonarRangeLSorted[2];
        // Compute the standard deviation of the collection of readings
        sonarRangeLStdev = stdevSonarRangeL();
        return sonarRangeLMedian;
    } // updateSonarRangeL

    private double stdevSonarRangeL(){
        double sum1=0.0, sum2=0.0, mean;
        for( int i=0; i<sonarRangeLSampCnt; i++ ) {
            sum1 += sonarRangeLSamples[i];
        }
        mean = sum1 / (double)sonarRangeLSampCnt;
        for( int i=0; i<sonarRangeLSampCnt; i++ ) {
            sum2 += Math.pow( (sonarRangeLSamples[i] - mean), 2.0);
        }
        return Math.sqrt( sum2 / (double)sonarRangeLSampCnt );
    } // stdevSonarRangeL

    /*--------------------------------------------------------------------------------------------*/
    public double updateSonarRangeR() {
        // Query the current range sensor reading as the next sample to our RIGHT range dataset
//      sonarRangeRSamples[ sonarRangeRIndex ] = sonarRangeR.getDistanceSync();
        sonarRangeRSamples[ sonarRangeRIndex ] = sonarRangeR.getDistanceAsync();
        if( ++sonarRangeRIndex >= sonarRangeRSampCnt ) sonarRangeRIndex = 0;
        // Create a duplicate copy that's sorted
        double[] sonarRangeRSorted = sonarRangeRSamples;
        Arrays.sort(sonarRangeRSorted);
        // Determine the running median (middle value of the last 5; assumes sonarRangeRSampCnt=5)
        sonarRangeRMedian = sonarRangeRSorted[2];
        // Compute the standard deviation of the collection of readings
        sonarRangeRStdev = stdevSonarRangeR();
        return sonarRangeRMedian;
    } // updateSonarRangeR

    private double stdevSonarRangeR(){
        double sum1=0.0, sum2=0.0, mean;
        for( int i=0; i<sonarRangeRSampCnt; i++ ) {
            sum1 += sonarRangeRSamples[i];
        }
        mean = sum1 / (double)sonarRangeRSampCnt;
        for( int i=0; i<sonarRangeRSampCnt; i++ ) {
            sum2 += Math.pow( (sonarRangeRSamples[i] - mean), 2.0);
        }
        return Math.sqrt( sum2 / (double)sonarRangeRSampCnt );
    } // stdevSonarRangeR

    /** Grab activity closes or opens the wobble arm claw. **/
    public final static double CLAW_TIME = 500.0;
    protected final static double CLAW_DIFFERENTIAL = 0.23;
    // Claw Closed positions
    public final static double CLAW_LEFT_CLOSED = 0.43;
    public final static double CLAW_RIGHT_CLOSED = 0.11;

    // Claw opened positions
    public final static double CLAW_LEFT_OPEN = CLAW_LEFT_CLOSED - CLAW_DIFFERENTIAL;
    public final static double CLAW_RIGHT_OPEN = CLAW_RIGHT_CLOSED + CLAW_DIFFERENTIAL;
    public boolean clawClosed = false;
    public enum GRABBING {
        IDLE,
        CLOSING
    }

    public GRABBING grabState = GRABBING.IDLE;
    public void startClawToggle(boolean open) {
        if(grabState == GRABBING.IDLE) {
            if(open) {
                clawLeft.setPosition(CLAW_LEFT_OPEN);
                clawRight.setPosition(CLAW_RIGHT_OPEN);
                clawClosed = false;
            } else {
                clawLeft.setPosition(CLAW_LEFT_CLOSED);
                clawRight.setPosition(CLAW_RIGHT_CLOSED);
                clawClosed = true;
            }
            clawTimer.reset();
            grabState = GRABBING.CLOSING;
        }
    }

    public void performClawToggle() {
        switch(grabState) {
            case CLOSING:
                if(clawTimer.milliseconds() >= CLAW_TIME) {
                    grabState = GRABBING.IDLE;
                }
                break;
            case IDLE:
            default:
                break;
        }
    }

    /** Inject activity pushes a disk into the shooter and resets th
     * e injector. **/
    public final static double INJECTOR_FIRE_TIME = 200.0;
    public final static double INJECTOR_RESET_TIME = 200.0;
    public final static double INJECTOR_HOME_TIME = 100.0;
    public final static double SHOOTER_THROTTLE_DELAY = 5000.0;
//    public final static double INJECTOR_HOME = 0.53;
//    public final static double INJECTOR_RESET = 0.450;
//    public final static double INJECTOR_FIRE = 0.850;
    // What we were using, shifted to 0.
//    public final static double INJECTOR_HOME = 0.0;
//    public final static double INJECTOR_RESET = -0.08;
//    public final static double INJECTOR_FIRE = 0.32;
    // 1.875 Multiplier for range 300 degrees to 160 for Savox
//    public final static double INJECTOR_HOME = 0.0;
//    public final static double INJECTOR_RESET = -0.15;
//    public final static double INJECTOR_FIRE = 0.6;
    public final static double INJECTOR_HOME = 0.73;
//    public final static double INJECTOR_RESET = 0.83;
    public final static double INJECTOR_RESET = 0.77;
//    public final static double INJECTOR_FIRE = 0.25;
    public final static double INJECTOR_FIRE = 0.50;
    public final static int VELOCITY_SUCCESS_CHECKS = 100;
    public boolean disableVelocityCheck = false;
    public int sequentialStableVelocityChecks = 0;
    public enum INJECTING {
        IDLE,
        THROTTLING_UP,
        FIRING,
        RESETTING,
        HOMING
    }

    public INJECTING injectState = INJECTING.IDLE;
    public void startInjecting() {
        if(injectState == INJECTING.IDLE) {
            // If the shooter isn't on, fire it up.
            shooterOn();
            injectTimer.reset();
            injectState = INJECTING.THROTTLING_UP;
        }
    }

    public void performInjecting() {
        switch(injectState) {
            case THROTTLING_UP:
                boolean throttledUp = false;
                // If the stable velocity isn't working, just wait for a small timer.
                // Will have to reset the velocity check by driver input.
                if(disableVelocityCheck) {
                    if(injectTimer.milliseconds() >= SHOOTER_THROTTLE_DELAY) {
                        throttledUp = true;
                    }
                } else {
                    // This means we can not reach target velocity, so disable
                    // velocity targeting and just use a timer in the future.
                    if(injectTimer.milliseconds() > THROTTLE_TIMEOUT) {
                        disableVelocityCheck = true;
                        throttledUp = true;
                    } else {
                        // Verify this loop is at target velocity.
                        if(sequentialStableVelocityChecks >= VELOCITY_SUCCESS_CHECKS) {
                            throttledUp = true;
                        }
                    }

                }
                if(throttledUp) {
                    injector.setPosition(INJECTOR_FIRE);
                    injectTimer.reset();
                    injectState = INJECTING.FIRING;
                }
                break;
            case FIRING:
                if(injectTimer.milliseconds() >= INJECTOR_FIRE_TIME) {
                    injector.setPosition(INJECTOR_RESET);
                    injectTimer.reset();
                    injectState = INJECTING.RESETTING;
                }
                break;
            case RESETTING:
                if(injectTimer.milliseconds() >= INJECTOR_RESET_TIME) {
                    injector.setPosition(INJECTOR_HOME);
                    injectTimer.reset();
                    injectState = INJECTING.HOMING;
                }
                break;
            case HOMING:
                if(injectTimer.milliseconds() >= INJECTOR_HOME_TIME) {
                    injectState = INJECTING.IDLE;
                }
                break;
            case IDLE:
            default:
                break;
        }
    }

    /** In case we want to create a short cut to fire 3 as quickly as possible. **/
    public enum TRIPLE_INJECTING {
        IDLE,
        FIRING_ONE,
        FIRING_TWO,
        FIRING_THREE
//        FIRING_FOUR
    }
    public TRIPLE_INJECTING tripleInjectState = TRIPLE_INJECTING.IDLE;
    public void startTripleInjecting() {
        if (tripleInjectState == TRIPLE_INJECTING.IDLE) {
            startInjecting();
            tripleInjectState = TRIPLE_INJECTING.FIRING_ONE;
        }
    }

    public void performTripleInjecting() {
        switch(tripleInjectState) {
            case FIRING_ONE:
                if(injectState == INJECTING.IDLE) {
                    tripleInjectState = TRIPLE_INJECTING.FIRING_TWO;
                    startInjecting();
                }
                break;
            case FIRING_TWO:
                if(injectState == INJECTING.IDLE) {
                    tripleInjectState = TRIPLE_INJECTING.FIRING_THREE;
                    startInjecting();
                }
                break;
            case FIRING_THREE:
                if(injectState == INJECTING.IDLE) {
                    tripleInjectState = TRIPLE_INJECTING.IDLE;
//                    startInjecting();
                }
                break;
//            case FIRING_FOUR:
//                if(injectState == INJECTING.IDLE) {
//                    shooterOff();
//                    tripleInjectState = TRIPLE_INJECTING.IDLE;
//                }
//                break;
            case IDLE:
            default:
                break;
        }
    }

    public enum INJECTING_JIGGLE {
        IDLE,
        RESETTING,
        HOMING
    }

    public INJECTING_JIGGLE injectJiggleState = INJECTING_JIGGLE.IDLE;
    public void startInjectingJiggle() {
        if(injectJiggleState == INJECTING_JIGGLE.IDLE) {
            injectJiggleState = INJECTING_JIGGLE.RESETTING;
            injector.setPosition(INJECTOR_RESET);
            injectTimer.reset();
        }
    }

    public void performInjectingJiggle() {
        switch(injectJiggleState) {
            case RESETTING:
                if(injectTimer.milliseconds() >= INJECTOR_RESET_TIME) {
                    injector.setPosition(INJECTOR_HOME);
                    injectTimer.reset();
                    injectJiggleState = INJECTING_JIGGLE.HOMING;
                }
                break;
            case HOMING:
                if(injectTimer.milliseconds() >= INJECTOR_HOME_TIME) {
                    injectJiggleState = INJECTING_JIGGLE.IDLE;
                }
                break;
            case IDLE:
            default:
                break;
        }
    }

    /** Moves the wobble arm to the specified position. **/
    public static double WOBBLE_ARM_STOWED = WOBBLE_ARM_MIN;
//    public static double WOBBLE_ARM_RUNNING = 1.400;
//    public static double WOBBLE_ARM_DEPLOYING = 2.100;
//    public static double WOBBLE_ARM_GRABBING = 3.100;
    public static double WOBBLE_ARM_RUNNING = 1.200;
    public static double WOBBLE_ARM_DEPLOYING = 2.000;
    public static double WOBBLE_ARM_GRABBING = 2.700;
    public static double WOBBLE_ARM_ERROR = 0.1;
    public static double WOBBLE_ARM_REFINING = 0.005;
    public enum WOBBLE_ARM_ROTATOR {
        IDLE,
        MOVING,
        REFINING
    }
    protected double targetPosition;
    public WOBBLE_ARM_ROTATOR armMovement = WOBBLE_ARM_ROTATOR.IDLE;
    public void startRotatingArm(double newPosition) {
        targetPosition = newPosition;
        armMovement = WOBBLE_ARM_ROTATOR.MOVING;
        if(targetPosition == WOBBLE_ARM_STOWED) {
            startClawToggle(false);
        }
    }

    public void performRotatingArm() {
        switch(armMovement) {
            case MOVING:
                if(abs(armPot.getVoltage() - targetPosition) > WOBBLE_ARM_ERROR) {
                    if(armPot.getVoltage() > targetPosition) {
                        setWobbleMotorPower(-1.0);
                    } else {
                        setWobbleMotorPower(1.0);
                    }
                } else {
                    setWobbleMotorPower(0.0);
                    armMovement = WOBBLE_ARM_ROTATOR.REFINING;
                }
                break;
            case REFINING:
                if(abs(armPot.getVoltage() - targetPosition) > WOBBLE_ARM_REFINING) {
                    if(armPot.getVoltage() > targetPosition) {
                        setWobbleMotorPower(-0.5);
                    } else {
                        setWobbleMotorPower(0.5);
                    }
                } else {
                    setWobbleMotorPower(0.0);
                    armMovement = WOBBLE_ARM_ROTATOR.IDLE;
                }
                break;
            case IDLE:
            default:
                break;
        }
    }

    /** Rotates to the specified location and shoots 1 ring for powershot and 3 for high goal. **/
    protected WayPoint rotationDestination;
    public enum ROTATE_ALIGNMENT_STATE {
        IDLE,
        ANGLE_ALIGNMENT,
        FIRE
    }
    public ROTATE_ALIGNMENT_STATE rotateAlignmentState = ROTATE_ALIGNMENT_STATE.IDLE;
    public void startRotateAligning(WayPoint rotationCoordinates) {
        if (rotateAlignmentState == ROTATE_ALIGNMENT_STATE.IDLE) {
            rotationDestination = rotationCoordinates;
            rotationDestination.x += shootingError.x;
            rotationDestination.y += shootingError.y;
            rotationDestination.angle += shootingError.angle;

            rotateToAngle(rotationDestination.angle, true);
            rotateAlignmentState = ROTATE_ALIGNMENT_STATE.ANGLE_ALIGNMENT;
        }
    }

    public void performRotateAligning() {
        switch(rotateAlignmentState) {
            case ANGLE_ALIGNMENT:
                if(rotateToAngle(rotationDestination.angle, false)) {
                    startInjecting();
                    rotateAlignmentState = ROTATE_ALIGNMENT_STATE.FIRE;
                }
                break;
            case FIRE:
                if(injectState == INJECTING.IDLE) {
                    rotateAlignmentState = ROTATE_ALIGNMENT_STATE.IDLE;
                }
                break;
            case IDLE:
            default:
                break;
        }
    }

    public void stopRotateAligning() {
        rotateAlignmentState = ROTATE_ALIGNMENT_STATE.IDLE;
        injector.setPosition(INJECTOR_HOME);
    }

    /** Moves to the specified location and shoots 1 ring for powershot and 3 for high goal. **/
    protected WayPoint shootingDestination;
    public enum SHOT_ALIGNMENT_STATE {
        IDLE,
        DRIVE_TO_POSITION,
        ANGLE_ALIGNMENT,
        FIRE
    }
    public SHOT_ALIGNMENT_STATE shotAlignmentState = SHOT_ALIGNMENT_STATE.IDLE;
    public FLAP_POSITION shooterFlapTarget;
    public void startShotAligning(WayPoint alignmentCoordinates, boolean shootingPowershot) {
        if (shotAlignmentState == SHOT_ALIGNMENT_STATE.IDLE) {
            shootingDestination = alignmentCoordinates;
            shootingDestination.x += shootingError.x;
            shootingDestination.y += shootingError.y;
            shootingDestination.angle += shootingError.angle;
            shooterFlapTarget = FLAP_POSITION.POWERSHOT;
            // If the shooter isn't on, fire it up.
            // Make sure shooter flap is in the right position.
            if(shootingPowershot) {
                shooterOnPowershot();
//                setShooterFlapPowerShot();
            } else {
                shooterOnHighGoal();
//                setShooterFlapHighGoal();
            }
//            shooterOn();
            driveToXY(shootingDestination.x,
                    shootingDestination.y,
                    shootingDestination.angle,
                    MIN_DRIVE_MAGNITUDE,
                    shootingDestination.speed, 0.014, 2.0, false);
            shotAlignmentState = SHOT_ALIGNMENT_STATE.DRIVE_TO_POSITION;
        }
    }

    public void performShotAligning() {
        switch(shotAlignmentState) {
            case DRIVE_TO_POSITION:
                if(driveToXY(shootingDestination.x, shootingDestination.y, shootingDestination.angle, MIN_DRIVE_MAGNITUDE,
                        1.0, 0.014, 2.0, false)) {
                    // We have reached the position, need to rotate to angle.
                    rotateToAngle(shootingDestination.angle, true);
                    shotAlignmentState = SHOT_ALIGNMENT_STATE.ANGLE_ALIGNMENT;
                }
                break;
            case ANGLE_ALIGNMENT:
                if(rotateToAngle(shootingDestination.angle, false)) {
                    startInjecting();
                    shotAlignmentState = SHOT_ALIGNMENT_STATE.FIRE;
                }
                break;
            case FIRE:
                if(injectState == INJECTING.IDLE) {
                    shotAlignmentState = SHOT_ALIGNMENT_STATE.IDLE;
                }
                break;
            case IDLE:
            default:
                break;
        }
    }

    public void stopShotAligning() {
        shotAlignmentState = SHOT_ALIGNMENT_STATE.IDLE;
        injector.setPosition(INJECTOR_HOME);
    }

    /** Moves to the specified location and shoots 3 ring for powershots. **/
    protected WayPoint shootingPowershotDestination = new WayPoint(0, 0, 0, 0);
    public enum POWERSHOT_ALIGNMENT_STATE {
        IDLE,
        DRIVE_TO_ULTRA_POSITION,
        ULTRA_ANGLE_ALIGNMENT,
        ULTRASONIC_MEASURE,
        DRIVE_TO_POSITION1,
        ANGLE_ALIGNMENT1,
        FIRE1,
        ANGLE_ALIGNMENT2,
        FIRE2,
        ANGLE_ALIGNMENT3,
        FIRE3
    }
    public POWERSHOT_ALIGNMENT_STATE powershotAlignmentState = POWERSHOT_ALIGNMENT_STATE.IDLE;
    private ElapsedTime ultrasonicTimer = new ElapsedTime();
    protected double POWERSHOT_LEFT_ANGLE_DIFF = Math.toRadians(5.0);
    protected double POWERSHOT_RIGHT_ANGLE_DIFF = Math.toRadians(4.0);
    protected double ULTRASONIC_ALIGNMENT_ANGLE = Math.toRadians(88.0);
    public double angleCorrection = 0.0;
    public void startPowershotAligning(WayPoint alignmentCoordinates, boolean shootingPowershot) {
        if (powershotAlignmentState == POWERSHOT_ALIGNMENT_STATE.IDLE) {

            // When we drop wobble goal, the robot must be flat against the wall,
            // We know this is angle 0 degrees
//            angleCorrection = MyPosition.worldAngle_rad;
            angleCorrection = Math.toRadians(-8.0);
            shootingPowershotDestination.angle = alignmentCoordinates.angle;
            shootingPowershotDestination.y = alignmentCoordinates.y;
            shootingPowershotDestination.x = alignmentCoordinates.x;
            shootingPowershotDestination.speed = alignmentCoordinates.speed;

            shooterFlapTarget = FLAP_POSITION.POWERSHOT;
            // If the shooter isn't on, fire it up.
            // Make sure shooter flap is in the right position.
            shooterOnPowershot();
            driveToXY(shootingPowershotDestination.x,
                    shootingPowershotDestination.y,
                    ULTRASONIC_ALIGNMENT_ANGLE - angleCorrection,
                    MIN_DRIVE_MAGNITUDE,
                    shootingPowershotDestination.speed, 0.014, 2.0, false);
            rangeSensorsEnabled = true;
            powershotAlignmentState = POWERSHOT_ALIGNMENT_STATE.DRIVE_TO_ULTRA_POSITION;
        }
    }

    public void performPowershotAligning() {
        switch(powershotAlignmentState) {
            case DRIVE_TO_ULTRA_POSITION:
                if(driveToXY(shootingPowershotDestination.x, shootingPowershotDestination.y, ULTRASONIC_ALIGNMENT_ANGLE - angleCorrection, MIN_DRIVE_MAGNITUDE,
                        shootingPowershotDestination.speed, 0.014, 2.0, false)) {
                    // We have reached the position, need to rotate to angle.
                    rotateToAngle(ULTRASONIC_ALIGNMENT_ANGLE - angleCorrection, true);
                    powershotAlignmentState = POWERSHOT_ALIGNMENT_STATE.ULTRA_ANGLE_ALIGNMENT;
                }
                break;
            case ULTRA_ANGLE_ALIGNMENT:
                if(rotateToAngle(ULTRASONIC_ALIGNMENT_ANGLE - angleCorrection, false)) {
                    powershotAlignmentState = POWERSHOT_ALIGNMENT_STATE.ULTRASONIC_MEASURE;
                    ultrasonicTimer.reset();
                }
                break;
            case ULTRASONIC_MEASURE:
                if(ultrasonicTimer.milliseconds() >= 500) {
                    // correction value
                    ultrasonicCorrection = ULTRASONIC_POWERSHOT_LEFT_DISTANCE - sonarRangeLMedian;
                    shootingPowershotDestination.x += ultrasonicCorrection;
                    driveToXY(shootingPowershotDestination.x, shootingPowershotDestination.y,
                            shootingPowershotDestination.angle - angleCorrection, MIN_DRIVE_MAGNITUDE,
                            shootingPowershotDestination.speed, 0.014, 2.0, false);
                    rangeSensorsEnabled = false;
                    powershotAlignmentState = POWERSHOT_ALIGNMENT_STATE.DRIVE_TO_POSITION1;
                }
                break;
            case DRIVE_TO_POSITION1:
                if(driveToXY(shootingPowershotDestination.x, shootingPowershotDestination.y,
                        shootingPowershotDestination.angle - angleCorrection, MIN_DRIVE_MAGNITUDE,
                        shootingPowershotDestination.speed, 0.014, 2.0, false)) {
                    // We have reached the position, need to rotate to angle.
                    rotateToAngle(shootingPowershotDestination.angle - angleCorrection, true);
                    powershotAlignmentState = POWERSHOT_ALIGNMENT_STATE.ANGLE_ALIGNMENT1;
                }
                break;
            case ANGLE_ALIGNMENT1:
                if(rotateToAngle(shootingPowershotDestination.angle - angleCorrection, false)) {
                    powershotAlignmentState = POWERSHOT_ALIGNMENT_STATE.FIRE1;
                    startInjecting();
                }
                break;
            case FIRE1:
                if(injectState == INJECTING.IDLE) {
                    powershotAlignmentState = POWERSHOT_ALIGNMENT_STATE.ANGLE_ALIGNMENT2;
                    // We have reached the position, need to rotate to angle.
                    rotateToAngle(shootingPowershotDestination.angle + POWERSHOT_LEFT_ANGLE_DIFF - angleCorrection,
                            true);
                }
                break;
            case ANGLE_ALIGNMENT2:
                if(rotateToAngle(shootingPowershotDestination.angle + POWERSHOT_LEFT_ANGLE_DIFF - angleCorrection,
                        false)) {
                    startInjecting();
                    powershotAlignmentState = POWERSHOT_ALIGNMENT_STATE.FIRE2;
                }
                break;
            case FIRE2:
                if(injectState == INJECTING.IDLE) {
                    powershotAlignmentState = POWERSHOT_ALIGNMENT_STATE.ANGLE_ALIGNMENT3;
                    // We have reached the position, need to rotate to angle.
                    rotateToAngle(shootingPowershotDestination.angle - POWERSHOT_RIGHT_ANGLE_DIFF - angleCorrection,
                            true);
                }
                break;
            case ANGLE_ALIGNMENT3:
                if(rotateToAngle(shootingPowershotDestination.angle - POWERSHOT_RIGHT_ANGLE_DIFF - angleCorrection,
                        false)) {
                    startInjecting();
                    powershotAlignmentState = POWERSHOT_ALIGNMENT_STATE.FIRE3;
                }
                break;
            case FIRE3:
                if(injectState == INJECTING.IDLE) {
                    powershotAlignmentState = POWERSHOT_ALIGNMENT_STATE.IDLE;
                }
                break;
            case IDLE:
            default:
                break;
        }
    }

    public void stopPowershotAligning() {
        powershotAlignmentState = POWERSHOT_ALIGNMENT_STATE.IDLE;
        injector.setPosition(INJECTOR_HOME);
    }
}