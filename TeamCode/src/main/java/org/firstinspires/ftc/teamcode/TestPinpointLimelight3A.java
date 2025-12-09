package org.firstinspires.ftc.teamcode;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection;
import static org.firstinspires.ftc.teamcode.GoBildaPinpointDriver.EncoderDirection;
import static org.firstinspires.ftc.teamcode.GoBildaPinpointDriver.GoBildaOdometryPods;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

/**
 * Test program to compare localization results from Limelight with GoBilda Pinpoint
 */
@TeleOp(name = "Pinpoint+Limelight3A", group = "Test")
//@Disabled
public class TestPinpointLimelight3A extends LinearOpMode {
    //====== INERTIAL MEASUREMENT UNIT (IMU) =====
    protected IMU imu = null;

    //====== MECANUM DRIVETRAIN MOTORS (RUN_USING_ENCODER) =====
    DcMotorEx frontLeftMotor = null;
    DcMotorEx frontRightMotor = null;
    DcMotorEx rearLeftMotor = null;
    DcMotorEx rearRightMotor = null;

    //====== GOBILDA PINPOINT ODOMETRY COMPUTER ======
    GoBildaPinpointDriver odom = null;

    //====== Limelight Camera ======
    Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        initIMU();
        initDrivetrain();
        initLimelight();
        initPinpointOdometry();
        alignPinpointToLimelight();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();

        alignPinpointToLimelight();

        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(), (int) status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            reportBothPoses();

            telemetry.update();
        }

        limelight.stop();
        driveTrainMotorsZero();
    }

    //----------------------------------------------------------------------------------------------
    public void initIMU() {
        // Define and initialize REV Expansion Hub IMU
        LogoFacingDirection logoDirection = LogoFacingDirection.RIGHT; // robot1
//      LogoFacingDirection logoDirection = LogoFacingDirection.LEFT;  // robot2
        UsbFacingDirection usbDirection = UsbFacingDirection.UP;       // robot1
//      UsbFacingDirection usbDirection = UsbFacingDirection.FORWARD;  // robot2
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    } // initIMU

    //----------------------------------------------------------------------------------------------
    void initDrivetrain() {
        // Query hardware info
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "FrontLeft");  // REVERSE
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "FrontRight"); // forward
        rearLeftMotor = hardwareMap.get(DcMotorEx.class, "RearLeft");   // REVERSE
        rearRightMotor = hardwareMap.get(DcMotorEx.class, "RearRight");  // forward

        // Set motor position-power direction
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        rearLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set all drivetrain motors to zero power
        driveTrainMotorsZero();
    } // initDrivetrain

    //----------------------------------------------------------------------------------------------
    public void driveTrainMotorsZero() {
        frontLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        rearLeftMotor.setPower(0.0);
        rearRightMotor.setPower(0.0);
    } // driveTrainMotorsZero

    //----------------------------------------------------------------------------------------------
    void initLimelight() {
        // NOTE: Control Hub is assigned eth0 address 172.29.0.1 by limelight DHCP server
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(5);
        limelight.pipelineSwitch(7); // pipeline 7 has been configured for AprilTags
        // Start polling for data.  If you neglect to call start(), getLatestResult() will return null
        limelight.start();
    } // initLimelight

    //----------------------------------------------------------------------------------------------
    void initPinpointOdometry() {
        // Locate the odometry controller in our hardware settings
        odom = hardwareMap.get(GoBildaPinpointDriver.class, "odom");   // Control Hub I2C port 3
//      odom.setOffsets(-144.0, +88.0, DistanceUnit.MM); // odometry pod x,y offsets relative center of robot
        odom.setOffsets(0.0, 0.0, DistanceUnit.MM); // odometry pod x,y offsets relative center of robot
        odom.setEncoderResolution(GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odom.setEncoderDirections(EncoderDirection.REVERSED, EncoderDirection.REVERSED);
        odom.resetPosAndIMU();
    } // initPinpointOdometry

    //----------------------------------------------------------------------------------------------
    void alignPinpointToLimelight() throws InterruptedException {
        // TODO: An absolute measure of robot camera angle relative to the field is needed if
        // we want to configure the Limelight with angle data relative to the field AprilTags.
        // REV Control Hub IMU always initializes to zero (is that how our robot starts??)
        // Pinpoint IMU angle can be set to match the robot starting angle, but then just
        // configure the Limelight robot yaw to the starting angle of the robot on the field.
        double robotYaw = imu.getRobotYawPitchRollAngles().getYaw(); // Should start at 0, facing towards Obelisk
        limelight.updateRobotOrientation(rotate180Yaw(robotYaw));  // ROTATE to match FTC field orientation
        Thread.sleep(10);
        // Query Limelight for Apriltag-based field location data
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            double captureLatency = llResult.getCaptureLatency();
            double targetingLatency = llResult.getTargetingLatency();
            double parseLatency = llResult.getParseLatency();
            telemetry.addData("Limelight Latency (msec)", captureLatency + targetingLatency);
            telemetry.addData("Parse Latency (msec)", parseLatency);
            // Parse Limelight result for MegaTag2 robot pose data
            Pose3D limelightBotpose = llResult.getBotpose_MT2();
            if (limelightBotpose != null) {
                Position limelightPosition = limelightBotpose.getPosition();
                YawPitchRollAngles limelightOrientation = limelightBotpose.getOrientation();
                // https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html#square-field-inverted-alliance-area
                // We want our pinpoint orientation rotated 180º from standard FTC orientation, so negate everything from limelight.
                double posX = -limelightPosition.x;
                double posY = -limelightPosition.y;
                double angDeg = rotate180Yaw(limelightOrientation.getYaw(AngleUnit.DEGREES));

                telemetry.addData("Limelight(Apriltag)", "x=%.2f y=%.2f %.2f deg",
                        limelightPosition.unit.toInches(posX),
                        limelightPosition.unit.toInches(posY),
                        angDeg);
                // Align Pinpoint Odometry position/angle to match Limelight Apriltag reading
                odom.setPosX(posX, limelightPosition.unit);
                odom.setPosY(posY, limelightPosition.unit);
                odom.setHeading(angDeg, AngleUnit.DEGREES);
                Thread.sleep(10);
                // Read back Pinpoint data
                odom.update();
                Pose2D pos = odom.getPosition();  // x,y pos in inch; heading in degrees
                telemetry.addData("Pinpoint Odometry", "x=%.2f y=%.2f  %.2f deg",
                        pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
            }
        } else {
            telemetry.addData("Limelight", "No data available");
        }
    } // alignPinpointToLimelight

    //----------------------------------------------------------------------------------------------
    void reportBothPoses() throws InterruptedException {
        // Query Pinpoint Odometry field location
        odom.update();
        Pose2D pos = odom.getPosition();  // x,y pos in inch; heading in degrees
        double odomX = pos.getX(DistanceUnit.INCH);
        double odomY = pos.getY(DistanceUnit.INCH);
        double odomAngle = pos.getHeading(AngleUnit.DEGREES);
        telemetry.addData("Pinpoint location", "x=%.2f y=%.2f  %.2f deg", odomX, odomY, odomAngle);
        // TODO: Until the REV Control Hub IMU angles are set to align with the field/robot, use the
        //  Pinpoint odometry heading to inform the limelight camera of it's orientation
//      double robotYaw = imu.getRobotYawPitchRollAngles().getYaw();
        limelight.updateRobotOrientation(rotate180Yaw(odomAngle)); // Rotate orientation!
        Thread.sleep(10);

        // Query Limelight for Apriltag-based field location data
        LLResult llResult = limelight.getLatestResult();
        if ((llResult != null) && llResult.isValid()) {
            // Parse Limelight result for MegaTag2 robot pose data
            Pose3D limelightBotpose = llResult.getBotpose_MT2();
            if (limelightBotpose != null) {
                Position limelightPosition = limelightBotpose.getPosition();
                YawPitchRollAngles limelightOrientation = limelightBotpose.getOrientation();
                // https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html#square-field-inverted-alliance-area
                // We want our pinpoint orientation rotated 180º from standard FTC orientation, so negate everything from limelight.
                double posX = -limelightPosition.x;
                double posY = -limelightPosition.y;
                double angDeg = rotate180Yaw(limelightOrientation.getYaw(AngleUnit.DEGREES));
                telemetry.addData("Limelight(Apriltag)", "x=%.2f y=%.2f %.2f deg", posX, posY, angDeg);
            }
            List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
            }
            double captureLatency = llResult.getCaptureLatency();
            double targetingLatency = llResult.getTargetingLatency();
            double parseLatency = llResult.getParseLatency();
            telemetry.addData("Limelight Latency (msec)", captureLatency + targetingLatency);
            telemetry.addData("Parse Latency (msec)", parseLatency);
        } else {
            telemetry.addData("Limelight", "No data available");
        }
    } // reportBothPoses

    private static double rotate180Yaw(double yaw) {
        double rotated = yaw + 180;
        double wrap = (rotated + 180) % 360;
        double shift = wrap - 180;
        return shift;
    }

} // TestPinpointLimelight3A
