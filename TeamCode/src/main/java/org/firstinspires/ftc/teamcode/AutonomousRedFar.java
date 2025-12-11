/* FTC Team 7572 - Version 1.0 (11/07/2024)
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.List;

/**
 */
@Autonomous(name="Red Far", group="7592", preselectTeleOp = "Teleop-Red")
//@Disabled
public class AutonomousRedFar extends AutonomousBase {

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drivetrain.
    static final boolean DRIVE_Y = true;    // Drive forward/backward
    static final boolean DRIVE_X = false;   // Drive right/left (not DRIVE_Y)

    double pos_y=robotGlobalYCoordinatePosition, pos_x=robotGlobalXCoordinatePosition, pos_angle=robotOrientationRadians;  // Allows us to specify movement ABSOLUTELY

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize robot hardware (autonomous mode)
        telemetry.addData("State", "Initializing (please wait)");
        telemetry.update();
        robot.init(hardwareMap,true);
        redAlliance  = true;

        // NOTE: Control Hub is assigned eth0 address 172.29.0.1 by limelight DHCP server
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();  // Start polling for data (skipping this has getLatestResult() return null results)

        // Wait for the game to start (driver presses PLAY).  While waiting, poll for options
        while (!isStarted()) {
            // Do we need to change any of the other autonomous options?
            processAutonomousInitMenu(false);  // not auto5 start position
            // Process limelight for obelisk detection
            processLimelightObelisk();
            // Pause briefly before looping
            idle();
        } // !isStarted

        limelight.stop();
        resetGlobalCoordinatePosition();
        scoringZones = 0;

        // Start the autonomous timer so we know how much time is remaining when cycling samples
        autonomousTimer.reset();

        //---------------------------------------------------------------------------------
        // AUTONOMOUS ROUTINE:  The following method is our main autonomous.
//      unitTestOdometryDrive();
        mainAutonomous( obeliskID );
        //---------------------------------------------------------------------------------

        // Ensure spindexer servo stops in case we exit while the spindexer is rotating
        robot.spinServoCR.setPower(0.0);

        telemetry.addData("Program", "Complete");
        telemetry.update();

    } /* runOpMode() */

    /*--------------------------------------------------------------------------------------------*/
    // TEST CODE: Verify gyro/encoder-based motion functions against a tape measure
    private void testGyroDrive() {
        double startAngle;
        gyroDrive(DRIVE_SPEED_20, DRIVE_Y, 12.0, 999.9, DRIVE_THRU ); // Drive FWD 12" along current heading
        gyroDrive(DRIVE_SPEED_20, DRIVE_X, 12.0, 999.9, DRIVE_TO  ); // Strafe RIGHT 12" along current heading
        // What is our starting angle?
        startAngle = getAngle();
        gyroTurn(TURN_SPEED_20, (startAngle + 45) );   // Turn CW 45 degrees
    } // testGyroDrive

    /*--------------------------------------------------------------------------------------------*/
    // TEST CODE: Verify odometry-based motion functions against a tape measure
    private void unitTestOdometryDrive() {
        double driveTime;
        telemetry.addData("Target", "x=24.0, y=0.0f, 0.00 deg (50%)");
        // reset our timer and drive forward 20"
        autonomousTimer.reset();
        driveToPosition(24.0, 0.0, 0.0, DRIVE_SPEED_50, TURN_SPEED_40, DRIVE_TO);
        driveTime = autonomousTimer.milliseconds() / 1000.0;
        performEveryLoop();  // ensure our odometry is updated
        telemetry.addData("Odometry", "x=%.2f, y=%.2f, %.2f deg", robotGlobalXCoordinatePosition, robotGlobalYCoordinatePosition, Math.toDegrees(robotOrientationRadians));
        telemetry.addData("Drive Time", "%.3f sec", driveTime);
        telemetry.update();
        sleep(3000);
        telemetry.addData("Target", "x=24.0, y=24.0f, 0.00 deg (50%)");
        // reset our timer and drive forward 20"
        autonomousTimer.reset();
        driveToPosition(24.0, 240, 0.0, DRIVE_SPEED_50, TURN_SPEED_40, DRIVE_TO);
        driveTime = autonomousTimer.milliseconds() / 1000.0;
        performEveryLoop();  // ensure our odometry is updated
        telemetry.addData("Odometry", "x=%.2f, y=%.2f, %.2f deg", robotGlobalXCoordinatePosition, robotGlobalYCoordinatePosition, Math.toDegrees(robotOrientationRadians));
        telemetry.addData("Drive Time", "%.3f sec", driveTime);
        telemetry.update();
        sleep(3000);
        telemetry.addData("Target", "x=24.0, y=24.0f, 90 deg (50%)");
        // reset our timer and drive forward 20"
        autonomousTimer.reset();
        driveToPosition(24.0, 240, 90.0, DRIVE_SPEED_50, TURN_SPEED_50, DRIVE_TO);
        driveTime = autonomousTimer.milliseconds() / 1000.0;
        performEveryLoop();  // ensure our odometry is updated
        telemetry.addData("Odometry", "x=%.2f, y=%.2f, %.2f deg", robotGlobalXCoordinatePosition, robotGlobalYCoordinatePosition, Math.toDegrees(robotOrientationRadians));
        telemetry.addData("Drive Time", "%.3f sec", driveTime);
        telemetry.update();
        sleep(24000);
    } // unitTestOdometryDrive

    /*--------------------------------------------------------------------------------------------*/
    /* Autonomous Red Far:                                                                        */
    /*   1 Starting point                                                                         */
    /*   2 Score preloads                                                                         */
    /*   3 Collect from tick marks (1, 2)                                                         */
    /*   4 Drive back to launch zone                                                              */
    /*   5 Score collected balls                                                                  */
    /*--------------------------------------------------------------------------------------------*/
    private void mainAutonomous( int obeliskID ) {

        // Do we start with an initial delay?
        if( startDelaySec > 0 ) {
            sleep( startDelaySec * 1000 );
        }

        // Score Preload Balls from the FAR zone
        scorePreloadBallsFromFar( obeliskID, redAlliance, 0.55 );
        
        // Collect and Score 1st spike mark
        collectSpikemarkFromFar( 1, redAlliance );
        scoreThreeBallsFromFar( obeliskID, redAlliance, 0.55 );

        // Collect and Score 2nd spike mark
        collectSpikemarkFromFar( 2, redAlliance );
        scoreThreeBallsFromFar( obeliskID, redAlliance, 0.55 );
            
        // Collect and Score 3rd spike mark
        collectSpikemarkFromFar( 3, redAlliance );
        scoreThreeBallsFromFar( obeliskID, redAlliance, 0.55 );

        // Drive away from the score line for the MOVEMENT points
        driveToPosition(32.0, 0.0, 0.0, DRIVE_SPEED_30, TURN_SPEED_30, DRIVE_TO);

        // ensure motors are turned off even if we run out of time
        robot.driveTrainMotorsZero();
    } // mainAutonomous


    private void driveToFirstTickMark() {
//        driveToPosition()
    }

} /* AutonomousRedFar */
