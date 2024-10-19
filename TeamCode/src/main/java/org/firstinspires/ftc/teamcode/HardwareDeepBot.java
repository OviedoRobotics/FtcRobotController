package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import static java.lang.Double.min;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class HardwareDeepBot {
    /* Public OpMode members. */
    protected DcMotorEx leftFront = null;
    protected DcMotorEx rightFront = null;
    protected DcMotorEx leftBack = null;
    protected DcMotorEx rightBack = null;

    protected DcMotorEx swingArm = null;
    protected DcMotorEx viperMotor = null;
    //====== ODOMETRY ENCODERS (encoder values only!) =====
    protected DcMotorEx rightOdometer      = null;
    public int          rightOdometerCount = 0;       // current encoder count
    public int          rightOdometerPrev  = 0;       // previous encoder count

    protected DcMotorEx leftOdometer       = null;
    public int          leftOdometerCount  = 0;       // current encoder count
    public int          leftOdometerPrev   = 0;       // previous encoder count

    protected DcMotorEx strafeOdometer      = null;
    public int          strafeOdometerCount = 0;      // current encoder count
    public int          strafeOdometerPrev  = 0;

    public Servo clawServo1 = null;
    HardwareMap hwMap = null;
    BNO055IMU imu = null;
    private ElapsedTime runTime = new ElapsedTime();

    public double CLAW1_SERVO_INIT = 1.0;
    public double CLAW1_SERVO_GRAB = 0.5;
    public double CLAW1_SERVO_EXPAND = 0.7;

    public double VIPER_COUNTS_PER_MOTOR_REV = 384.5;
    public double VIPER_GEAR_REDUCTION    = 1.0;
    public double VIPER_GEAR_DIAMETER_INCHES =  0.0;
    public double VIPER_COUNTS_PER_INCH = VIPER_COUNTS_PER_MOTOR_REV*VIPER_GEAR_REDUCTION/(VIPER_GEAR_DIAMETER_INCHES*3.1415);

    public double ARM_GEAR_REDUCTION = 28;
    public double ARM_TICKS_PER_REVOLUTION = 145;

    public double ARM_COUNTS_PER_DEGREE = ARM_TICKS_PER_REVOLUTION*ARM_GEAR_REDUCTION/360;

    public int ARM_HIGH   = 2692; // Encoder count when arm is at the high basket scoring position (currently at max encoder count)
    public int ARM_GROUND = 0; // Encoder count when arm is at the floor level
    public int ARM_PARALLEL = 5; // Encoder count when arm is parallel to floor
    public double ARM_RAISE_POWER = 0.8; // Motor power to raise arm
    public double ARM_LOWER_POWER = 0.5; // Motor power to lower arm
    public double ARM_RETRACT_INCHES = 10; //  length of the arm in inches when it’s fully retracted
    public int ARM_EXTENSION_COUNTS = 0; //  target encoder position for the length of the slides.
    //public double CURRENT_ARM_INCHES = ARM_RETRACT_INCHES + ARM_EXTENSION_INCHES; //e total length of the arm in inches at any given time.
    public double ARM_EXTENSION_INCHES = 0.0;
    public double ARM_FLOOR_HEIGHT = 5.0;
    public double ARM_LEFT_POSITION = 8.0; // TODO: change name
    public double ARM_RIGHT_POSITION = 10.0; // TODO: change name
    public double ARM_EXTENSION_TOLERANCE = 0.5;
    public double ARM_EXTENSION_TOLERANCE_MAX = 3;
    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFront = hwMap.get(DcMotorEx.class, "left_front");
        rightFront = hwMap.get(DcMotorEx.class, "right_front");
        leftBack = hwMap.get(DcMotorEx.class, "left_back");
        rightBack = hwMap.get(DcMotorEx.class, "right_back");
        clawServo1 = hwMap.get(Servo.class, "claw_intake");
        leftFront.setDirection(DcMotorEx.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftBack.setDirection(DcMotorEx.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        // Set all motors to zero power
        setMotorPowers(0, 0, 0, 0);

        // Set all drivetrain motors to run with encoders.
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set all drivetrain motors to brake when at zero power
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        viperMotor = hwMap.get(DcMotorEx.class, "viper_motor");
        viperMotor.setDirection(DcMotor.Direction.FORWARD);
        viperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        viperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperMotor.setPower( 0.0 );

        swingArm.setDirection(DcMotor.Direction.FORWARD);
        swingArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        swingArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        swingArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        swingArm.setPower( 0.0 );

        clawServo1.setPosition(CLAW1_SERVO_INIT);

        strafeOdometer = hwMap.get(DcMotorEx.class,"OdomStrafe");  // Expansion Hub port 3 (encoder only; no motor)
        strafeOdometer.setDirection(DcMotor.Direction.FORWARD);
        strafeOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafeOdometer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        strafeOdometer.setPower( 0.0 );

        rightOdometer  = hwMap.get(DcMotorEx.class,"OdomRight");   // Control Hub port 3
        rightOdometer.setDirection(DcMotor.Direction.FORWARD);
        rightOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdometer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightOdometer.setPower( 0.0 );

        leftOdometer  = hwMap.get(DcMotorEx.class,"OdomLeft");   // Control Hub port 3
        leftOdometer.setDirection(DcMotor.Direction.FORWARD);
        leftOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftOdometer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftOdometer.setPower( 0.0 );

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }


    public void setDriveTrainMode(DcMotorEx.RunMode mode) {
        leftFront.setMode(mode);
        rightFront.setMode(mode);
        leftBack.setMode(mode);
        rightBack.setMode(mode);
    }

    public void setMotorPowers(double LFpow, double RFpow, double LBpow, double RBpow) {
        leftFront.setPower(LFpow);
        rightFront.setPower(RFpow);
        leftBack.setPower(LBpow);
        rightBack.setPower(RBpow);
    }

    public void setMotorPowers(double pow) {
        setMotorPowers(pow, pow, pow, pow);
    }

    public void startArmPivotAutomatic(int liftTarget){
        // Range-check the target
        if (liftTarget < ARM_GROUND) liftTarget = ARM_GROUND;
        if (liftTarget > ARM_HIGH) liftTarget = ARM_HIGH;
        // Configure target encoder count
        swingArm.setTargetPosition(liftTarget);
        swingArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runTime.reset();
        double motorPower = (swingArm.getCurrentPosition() < liftTarget)? ARM_RAISE_POWER : ARM_LOWER_POWER;
        swingArm.setPower(motorPower);
        while(swingArm.isBusy()){
            telemetry.addData("Running to ", "%5f" , liftTarget);
            telemetry.addData("Currently at ", "%5f", swingArm.getCurrentPosition());
            telemetry.update();
        }
        runTime.reset();
        swingArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        swingArm.setPower(0);
        maxSafeExtensionCounts(liftTarget - ARM_PARALLEL);
    }

    public void startArmPivotManual(double armPow, double vipPower, boolean liftingUp) {
        int armCount = swingArm.getCurrentPosition() - ARM_PARALLEL;
        int curVipPosition = viperMotor.getCurrentPosition();
        double maxPosition = maxSafeExtensionCounts(armCount);
        double angle = calculateArmAngle(armCount);
        if ((angle < 90 && liftingUp) || (angle > 90 && !liftingUp)) { // extending
            if (maxPosition - curVipPosition > ARM_EXTENSION_TOLERANCE) {
                viperMotor.setPower(vipPower);
                swingArm.setPower(armPow);
            } else viperMotor.setPower(0); //if we cant extend, set viper power to 0
        } else { // retracting
            if (maxPosition - curVipPosition > ARM_EXTENSION_TOLERANCE_MAX) {
                viperMotor.setPower(0);
                swingArm.setPower(armPow);
            } // retracted to far
            else if (maxPosition - curVipPosition > ARM_EXTENSION_TOLERANCE) {
                viperMotor.setPower(-vipPower);
                swingArm.setPower(armPow);
            } else {
                swingArm.setPower(0); // give time to retract (extended to far)
            }
        }
    }
    public double calculateArmAngle(int armHorizontalCount){
        return armHorizontalCount / ARM_COUNTS_PER_DEGREE;
    } // angle from horizontal

    public int viperExtensionPosition(double viperExtensionInches){
        ARM_EXTENSION_COUNTS = (int) (viperExtensionInches * VIPER_COUNTS_PER_INCH);
        //CURRENT_ARM_INCHES = ARM_RETRACT_INCHES + ARM_EXTENSION_INCHES;
        return ARM_EXTENSION_COUNTS;
    } // viper extension encoder count position converted from the extension of the slides in inches

    public int maxSafeExtensionCounts(int armHorizontalCount){
        double curAngle = calculateArmAngle(armHorizontalCount);
        boolean rightSide = curAngle < 90;
        boolean down = curAngle < 0 || curAngle > 180;
        double extensionH; //max horizontal extension
        double extensionV; //max vertical extension
        if (!down && rightSide) {
            ARM_EXTENSION_INCHES = (42 - ARM_LEFT_POSITION) * (1 / cos(curAngle)) - ARM_RETRACT_INCHES; // maximum amount we can extend
        } // extending up and to the right
        else if (!down && !rightSide) {
            ARM_EXTENSION_INCHES = (42 - ARM_RIGHT_POSITION) * (1 / cos(curAngle)) - ARM_RETRACT_INCHES; // maximum amount we can extend
        } // extending up and to the left
        else if (down && rightSide) {
            extensionH = (42 - ARM_LEFT_POSITION) * (1 / cos(curAngle)) - ARM_RETRACT_INCHES;
            extensionV = (ARM_FLOOR_HEIGHT) * (1 / sin(curAngle)) - ARM_RETRACT_INCHES;
            ARM_EXTENSION_INCHES = min(extensionH, extensionV);
        } // extending down and to the right
        else {
            extensionH = (42 - ARM_RIGHT_POSITION) * (1 / cos(curAngle)) - ARM_RETRACT_INCHES;
            extensionV = (ARM_FLOOR_HEIGHT) * (1 / sin(curAngle)) - ARM_RETRACT_INCHES;
            ARM_EXTENSION_INCHES = min(extensionH, extensionV);
        } // extending down and to the left
        return viperExtensionPosition(ARM_EXTENSION_INCHES); // returns the maximum encoder position the viper motor can extend to
    }
}