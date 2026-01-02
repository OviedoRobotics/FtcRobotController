package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
    HardwareSwyftBot robot = new HardwareSwyftBot();

    LimelightFusedPinpointOdometry llodo = null;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, true); // using isAutonomous=true to ensure encoders/turret are reset.
        robot.limelightStart(); // must be started before next class is used
        llodo = new LimelightFusedPinpointOdometry(robot.limelight, robot.odom, telemetry, 0.0);
        llodo.updatePipeline(Alliance.BLUE); // DEFAULT
        llodo.alignPinpointToLimelightEveryLoop(true);

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();

        llodo.alignPinpointToLimelightEveryLoop(true);

        while (opModeIsActive()) {

            // Pressing the RED circle button toggles the RED apriltag limelight pipeline
            if( gamepad1.circleWasPressed()  || gamepad2.circleWasPressed()) {
                llodo.updatePipeline(Alliance.RED);
            }

            // Pressing the BLUE cross button toggles the BLUE apriltag limelight pipeline
            if( gamepad1.crossWasPressed() || gamepad2.crossWasPressed() ) {
                llodo.updatePipeline(Alliance.BLUE);
            }

            LLStatus status = robot.limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(), (int) status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            reportBothPoses();

            // Pressing the TRIANGLE button adjusts turret angle to the limelight blue/red target angle
            // NOTE: since the limelight is mounted on the drivetrain, not the turret, the limelight
            // solution doesn't shift to zero once the turret rotates.  Until the robot drivetrain itself
            // is changed, the limelight angle will continue to report the same answer.
            if( gamepad1.triangleWasPressed() || gamepad2.triangleWasPressed() ) {
                //targetTurret();
            }

            telemetry.addLine("-----------------------------");
            telemetry.addLine("CIRCLE - switch to RED target pipeline");
            telemetry.addLine("CROSS  - switch to BLUE target pipeline");
            telemetry.addLine("TRIANGLE - aim turret based on limelight");
            telemetry.update();
        }

        llodo.stop();
        robot.driveTrainMotorsZero();
    }

    //----------------------------------------------------------------------------------------------
    void reportBothPoses() throws InterruptedException {
        // Query Pinpoint Odometry field location
        robot.odom.update();
        Pose2D pos = robot.odom.getPosition();  // x,y pos in inch; heading in degrees
        double odomX = pos.getX(DistanceUnit.INCH);
        double odomY = pos.getY(DistanceUnit.INCH);
        double odomAngle = pos.getHeading(AngleUnit.DEGREES);
        telemetry.addData("Pinpoint location", "x=%.2f y=%.2f  %.2f deg", odomX, odomY, odomAngle);
        robot.limelight.updateRobotOrientation(rotate180Yaw(odomAngle)); // Rotate orientation!
        Thread.sleep(10);

        // Query Limelight for Apriltag-based field location data
        LLResult llResult = robot.limelight.getLatestResult();
        if ((llResult != null) && llResult.isValid()) {
            // Parse Limelight result for MegaTag2 robot pose data
            Pose3D limelightBotpose = llResult.getBotpose_MT2();
            if (limelightBotpose != null) {
                Position           limelightPosition    = limelightBotpose.getPosition();
                YawPitchRollAngles limelightOrientation = limelightBotpose.getOrientation();
                // https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html#square-field-inverted-alliance-area
                // We want our pinpoint orientation rotated 180� from standard FTC orientation, so negate everything from limelight.
                double posX   = -limelightPosition.unit.toInches(limelightPosition.x);  // Pinpoint +X (forward) matches Limelight +X (forward)
                double posY   = -limelightPosition.unit.toInches(limelightPosition.y);  // Pinpoint +Y (left) opposite of Limelight +Y (right)
                double angDeg = rotate180Yaw(limelightOrientation.getYaw(AngleUnit.DEGREES));
                double[] stddev = llResult.getStddevMt2();
                telemetry.addData("Limelight(Apriltag)", "x=%.2f y=%.2f %.2f deg", posX, posY, angDeg);
                telemetry.addData("LL StdDev (x,y,yaw)", "x=%.2f y=%.2f %.2f deg", stddev[0], stddev[1], stddev[5]);
            }
            List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("Fiducial", "ID: %d, X: %.2f deg, Y: %.2f deg", fr.getFiducialId(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
            }
            double captureLatency   = llResult.getCaptureLatency();
            double targetingLatency = llResult.getTargetingLatency();
            double parseLatency     = llResult.getParseLatency();
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
