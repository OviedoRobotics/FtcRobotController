package org.firstinspires.ftc.teamcode;

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

import java.util.List;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection;

/**
 * Test program to compare localization results from Limelight with GoBilda Pinpoint
 */
@TeleOp(name = "Sensor: Pinpoint+Limelight3A", group = "Sensor")
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
        initOdometry();
        updateOdometry();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(), (int) status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            reportPoses();

            telemetry.update();
        }

        limelight.stop();
        driveTrainMotorsZero();
    }

    public void initIMU() {
        // Define and initialize REV Expansion Hub IMU
        // This needs to be changed to match the orientation on the robot
        // robot v1:
//        LogoFacingDirection logoDirection = LogoFacingDirection.RIGHT;
//        UsbFacingDirection usbDirection = UsbFacingDirection.UP;
        // robot v2:
        LogoFacingDirection logoDirection = LogoFacingDirection.LEFT;
        UsbFacingDirection usbDirection = UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

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
    }

    public void driveTrainMotorsZero() {
        frontLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        rearLeftMotor.setPower(0.0);
        rearRightMotor.setPower(0.0);
    }

    void initLimelight() {
        // NOTE: Control Hub is assigned eth0 address 172.29.0.1 by limelight DHCP server
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(1);
        // Start polling for data.  If you neglect to call start(), getLatestResult() will return null
        limelight.start();
    }

    void initOdometry() {
        // Locate the odometry controller in our hardware settings
        odom = hardwareMap.get(GoBildaPinpointDriver.class, "odom");   // Control Hub I2C port 3
        odom.setOffsets(-144.0, +88.0, DistanceUnit.MM); // odometry pod x,y offsets relative center of robot
        odom.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odom.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odom.resetPosAndIMU();
    }

    void updateOdometry() {
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            double captureLatency = llResult.getCaptureLatency();
            double targetingLatency = llResult.getTargetingLatency();
            double parseLatency = llResult.getParseLatency();
            telemetry.addData("Limelight Latency (msec)", captureLatency + targetingLatency);
            telemetry.addData("Parse Latency (msec)", parseLatency);
            // Better to use controller or pinpoint IMU?
            double robotYaw = imu.getRobotYawPitchRollAngles().getYaw();
            limelight.updateRobotOrientation(robotYaw);
            Pose3D botpose = llResult.getBotpose_MT2();
            if (botpose != null) {
                Position position = botpose.getPosition();
                odom.setPosX(position.x, position.unit);
                odom.setPosY(position.y, position.unit);
            }
        } else {
            telemetry.addData("Limelight", "No data available");
        }
    }

    void reportPoses() {
        odom.update();
        Pose2D pos = odom.getPosition();  // x,y pos in inch; heading in degrees
        telemetry.addData("Odometry", "x=%.2f y=%.2f  %.2f deg",
                pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), Math.toDegrees(pos.getHeading(AngleUnit.RADIANS)));


        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();
            double parseLatency = result.getParseLatency();
            telemetry.addData("Limelight Latency (msec)", captureLatency + targetingLatency);
            telemetry.addData("Parse Latency (msec)", parseLatency);

            Pose3D botpose = result.getBotpose_MT2();
            if (botpose != null) {
                Position position = botpose.getPosition();
                telemetry.addData("Limelight Position", "x=%.2f y=%.2f z=%.2f",
                        position.unit.toInches(position.x), position.unit.toInches(position.y), position.unit.toInches(position.z));
            }
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
            }
        } else {
            telemetry.addData("Limelight", "No data available");
        }
    }
}
