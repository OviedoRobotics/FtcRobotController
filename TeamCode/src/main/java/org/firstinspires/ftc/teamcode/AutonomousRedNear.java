/* FTC Team 7572 - Version 1.0 (11/07/2024)
 */
package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.BallOrder.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="Red Near", group="7592", preselectTeleOp = "Teleop-Red")
//@Disabled
public class AutonomousRedNear extends AutonomousBase {

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drivetrain.
    static final boolean DRIVE_Y = true;    // Drive forward/backward
    static final boolean DRIVE_X = false;   // Drive right/left (not DRIVE_Y)

    @Override
    public void runOpMode() throws InterruptedException {

        doSpikeMark1 = false;
        doSpikeMark2 = false;
        doSpikeMark3 = false;

        // Initialize robot hardware (autonomous mode)
        telemetry.addData("State", "Initializing (please wait)");
        telemetry.update();
        robot.init(hardwareMap,true);
        redAlliance = true;
        runningAutonomousFar = false;

        robot.limelightPipelineSwitch( 1 );
        robot.limelightStart();  // Start polling for data (skipping this has getLatestResult() return null results)

        // Wait for the game to start (driver presses PLAY).  While waiting, poll for options
        while (!isStarted()) {
            // Do we need to change any of the other autonomous options?
            processAutonomousInitMenu(false);  // not auto5 start position
            // Pause briefly before looping
            idle();
        } // !isStarted

        // Start the autonomous timer so we know how much time is remaining when cycling samples
        autonomousTimer.reset();

        // Establish our starting position on the field (in field coordinate system)
        resetGlobalCoordinatePositionAuto( 38.6, -54.3, +90.0 );

        // Drive away from the wall to a point that can see the obelisk
        driveToPosition( 34.0, -45.0, +90.0, DRIVE_SPEED_30, TURN_SPEED_15, DRIVE_THRU);
        driveToPosition( 24.2, -17.6, +20.0, DRIVE_SPEED_50, TURN_SPEED_15, DRIVE_TO);

        // Process limelight for obelisk detection
        for( int i=0; i<3; i++ ) {
            telemetry.addData("ALLIANCE", "%s", ((redAlliance)? "RED":"BLUE"));
            telemetry.addData("Odometry","x=%.2f y=%.2f  %.2f deg",
                 robotGlobalXCoordinatePosition, robotGlobalYCoordinatePosition, Math.toDegrees(robotOrientationRadians) );
            processLimelightObelisk();
            telemetry.update();
            // Pause briefly before looping
            idle();
        }

        // We're done with the obelisk; switch to the pipeline for the Goal apriltag
        robot.limelightPipelineSwitch( (redAlliance)? 7:6 );

        //---------------------------------------------------------------------------------
        // AUTONOMOUS ROUTINE:  The following method is our main autonomous.
//      unitTestOdometryDrive();
        mainAutonomous( obeliskID );
        //---------------------------------------------------------------------------------

        robot.limelightStop();
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
    /* Autonomous Red Far:                                                                        */
    /*   1 Starting point                                                                         */
    /*   2 Score preloads                                                                         */
    /*   3 Collect from tick marks (1, 2)                                                         */
    /*   4 Drive back to launch zone                                                              */
    /*   5 Score collected balls                                                                  */
    /*--------------------------------------------------------------------------------------------*/
    private void mainAutonomous(BallOrder obeliskID) {
        double shooterPowerNear = 0.45;

        // Do we start with an initial delay?
        if( startDelaySec > 0 ) {
            sleep( startDelaySec * 1000 );
        }

        //===== Score Preload Balls (from the NEAR zone) ==========
        // Enable collector/InKeeper so it's safe to spindex
        robot.intakeMotor.setPower( robot.INTAKE_FWD_COLLECT );
        // Even if we delay, we want to immediately start up getting shooter up to speed
        robot.shooterMotorsSetPower( shooterPowerNear );
        // Enable automatic shooter power/angle as we drive the next segment
        autoAimEnabled = true;
        // Drive to where we can both shoot and refresh our field position based on the AprilTag
        driveToPosition( 24.2, -17.6, -49.0, DRIVE_SPEED_30, TURN_SPEED_15, DRIVE_TO);
        autoAimEnabled = false;
        scoreThreeBallsFromField(obeliskID, PPG_23);
        // update our field position based on the AprilTag
        robot.setPinpointFieldPosition(robot.limelightFieldXpos, robot.limelightFieldYpos, robot.limelightFieldAngleDeg);

        // TODO: doSpikeMark* disabled by default at the start of runOpMode. remove that when implemented.

        // Collect and Score 3rd spike mark
        if( doSpikeMark3 ) {
            collectSpikemarkFromNear(3,redAlliance );
            scoreThreeBallsFromField(obeliskID, (redAlliance)? GPP_21:GPP_21 );
        }

        // Collect and Score 2nd spike mark
        if( doSpikeMark2 ) {
            collectSpikemarkFromNear(2,redAlliance );
            scoreThreeBallsFromField(obeliskID, (redAlliance)? PPG_23:PPG_23 );
        }

        // Collect and Score 1st spike mark
        if( doSpikeMark1 ) {
            collectSpikemarkFromNear(1,redAlliance);
            scoreThreeBallsFromField(obeliskID, (redAlliance)? PGP_22:PGP_22 );
        }

        // Drive the final position we want for MOVEMENT points
        driveToPosition(40.7, -17.6, -49.0, DRIVE_SPEED_30, TURN_SPEED_30, DRIVE_TO);

        // ensure motors are turned off even if we run out of time
        robot.driveTrainMotorsZero();
    } // mainAutonomous

} /* AutonomousRedNear */
