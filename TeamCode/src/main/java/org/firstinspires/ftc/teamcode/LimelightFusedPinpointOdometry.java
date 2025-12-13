package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

/**
 * Class that uses a Limelight localization pipeline to keep the pinpoint odometry location accurate.
 */
public class LimelightFusedPinpointOdometry {
    /**
     * https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html#square-field-inverted-alliance-area
     * We currently use a field orientation that is 180º rotated from the standard FTC field,
     * (+x -> Obelisk, -x -> audience | +y -> blue goal, -y -> red goal)
     * so we have to adjust the values returned from the limelight camera (and the yaw fed back into it).
     */
    private static final boolean ROTATE_LIMELIGHT_FIELD_180 = true;
    private static final int STALENESS_LIMIT_MS = 5;
    private final Limelight3A limelight;
    private final GoBildaPinpointDriver odom;
    private final Telemetry telemetry;
    private final double robotStartingYawDegrees;
    private Alliance alliance;

    private LLResult lastUsedResult;

    public LimelightFusedPinpointOdometry(Limelight3A limelight, GoBildaPinpointDriver odom, Telemetry telemetry, double robotStartingYawDegrees) {
        assert limelight != null;
        assert odom != null;
        assert telemetry != null;
        this.limelight = limelight;
        this.odom = odom;
        this.telemetry = telemetry;
        this.robotStartingYawDegrees = robotStartingYawDegrees;
    }

    public void startPipeline(Alliance alliance) {
        assert alliance != null;
        int allianceLocalizationPipeline = alliance == Alliance.BLUE ? 6 : 7;
        boolean result = limelight.pipelineSwitch(allianceLocalizationPipeline);
        assert result;
        // limelight.setPollRateHz(100); // default is every 10ms
        limelight.start();
        limelight.updateRobotOrientation(rotate180Yaw(robotStartingYawDegrees));
    }

    public void stop() {
        limelight.stop();
    }

    /**
     * Using the latest limelight data, (which may not have targeting info if the camera isn't facing the target) return details for where to shoot for the configured alliance.
     *
     * @return FiducialResult for target which includes targetXDegrees for aiming and targetYDegrees for adjusting motor speed.
     */
    public LLResultTypes.FiducialResult getShootTarget() {
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            int targetApriltag = alliance == Alliance.BLUE ? 20 : 24;
            List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                if (fr.getFiducialId() == targetApriltag) {
                    return fr;
                }
            }
        }
        return null;
    }

    /**
     * Should be called on every loop. Actual updates will be rate limited and only happen if a valid location is identified.
     */
    void alignPinpointToLimelightEveryLoop(boolean forceUpdateOdometry) {
        // Query Pinpoint Odometry field location
        if (forceUpdateOdometry)
            odom.update(); // Assume this is already called earlier this loop?
        Pose2D pos = odom.getPosition();  // x,y pos in inch; heading in degrees
        double odomX = pos.getX(DistanceUnit.INCH);
        double odomY = pos.getY(DistanceUnit.INCH);
        double odomAngle = pos.getHeading(AngleUnit.DEGREES);
        // It won't take effect this loop, but update for the next time the limelight is queried.
        limelight.updateRobotOrientation(rotate180Yaw(odomAngle)); // Rotate orientation!

        // Check Limelight for Apriltag-based field location data
        LLResult llResult = limelight.getLatestResult();
        if (lastUsedResult != null && lastUsedResult == llResult) {
            // Already processed.
            return;
        }
        if (llResult != null && llResult.isValid() && llResult.getStaleness() < STALENESS_LIMIT_MS) {
            lastUsedResult = llResult;
            telemetry.addData("Limelight Latency (msec)", llResult.getCaptureLatency() + llResult.getTargetingLatency());
            telemetry.addData("Parse Latency (msec)", llResult.getParseLatency());
            // Parse Limelight result for MegaTag2 robot pose data
            Pose3D limelightBotpose = llResult.getBotpose_MT2();
            double[] stddev = llResult.getStddevMt2();
            if (limelightBotpose != null) {
                Position limelightPosition = limelightBotpose.getPosition();
                YawPitchRollAngles limelightOrientation = limelightBotpose.getOrientation();
                double posX = rotate180XY(limelightPosition.x);
                double posY = rotate180XY(limelightPosition.y);
                double angDeg = rotate180Yaw(limelightOrientation.getYaw(AngleUnit.DEGREES));

                telemetry.addData("LL StdDev (x,y,yaw)", "x=%.2f y=%.2f %.2fº", stddev[0], stddev[1], stddev[5]);
                telemetry.addData("Limelight(Apriltag)", "x=%.2f y=%.2f %.2fº",
                        limelightPosition.unit.toInches(posX),
                        limelightPosition.unit.toInches(posY),
                        angDeg);
                // Align Pinpoint Odometry position/angle to match Limelight Apriltag reading
                odom.setPosX(posX, limelightPosition.unit);
                odom.setPosY(posY, limelightPosition.unit);
                odom.setHeading(angDeg, AngleUnit.DEGREES);
            }
            telemetry.addData("Pinpoint Odometry", "x=%.2f y=%.2f  %.2fº", odomX, odomY, odomAngle);
        } else {
            telemetry.addData("Limelight", "No data available");
            telemetry.addData("Pinpoint Odometry", "x=%.2f y=%.2f  %.2fº", odomX, odomY, odomAngle);
        }
    }

    private static double rotate180XY(double xy) {
        return ROTATE_LIMELIGHT_FIELD_180 ? -xy : xy;
    }

    private static double rotate180Yaw(double yaw) {
        if (!ROTATE_LIMELIGHT_FIELD_180) return yaw;
        double rotated = yaw + 180;
        double wrap = (rotated + 180) % 360;
        double shift = wrap - 180;
        return shift;
    }
}
