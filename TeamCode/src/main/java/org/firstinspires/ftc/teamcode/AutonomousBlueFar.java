/* FTC Team 7572 - Version 1.0 (11/07/2024)
 */
package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.BallOrder.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="Blue Far", group="7592", preselectTeleOp = "Teleop-Blue")
//@Disabled
public class AutonomousBlueFar extends AutonomousBase {

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
        redAlliance  = false;

        robot.limelightPipelineSwitch( 1 );
        robot.limelightStart();  // Start polling for data (skipping this has getLatestResult() return null results)

        // Wait for the game to start (driver presses PLAY).  While waiting, poll for options
        while (!isStarted()) {
            // Do we need to change any of the other autonomous options?
            processAutonomousInitMenu(false);  // not auto5 start position
            // Process limelight for obelisk detection
            processLimelightObelisk();
            // Pause briefly before looping
            idle();
        } // !isStarted

        robot.limelightStop();

        // Start the autonomous timer so we know how much time is remaining when cycling samples
        autonomousTimer.reset();

        // Establish our starting position on the field (in field coordinate system)
        resetGlobalCoordinatePositionAuto(-62.8, +14.3, 0.0 );

        //---------------------------------------------------------------------------------
        // AUTONOMOUS ROUTINE:  The following method is our main autonomous.
//      unitTestOdometryDrive();
        mainAutonomous( obeliskID );
        //---------------------------------------------------------------------------------

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
    /* Autonomous Red/Blue Far:                                                                   */
    /*   1 Starting point                                                                         */
    /*   2 Score preloads                                                                         */
    /*   3 Collect from tick marks (1, 2)                                                         */
    /*   4 Drive back to launch zone                                                              */
    /*   5 Score collected balls                                                                  */
    /*--------------------------------------------------------------------------------------------*/
    private void mainAutonomous(BallOrder obeliskID) {
        double shooterPowerFar = 0.55;
        HardwareSwyftBot.SpindexerState firstBall;
        BallOrder loadOrder;

        //===== Score Preload Balls (from the FAR zone) ==========
        // Enable collector/InKeeper so it's safe to spindex
        robot.intakeMotor.setPower( robot.INTAKE_FWD_COLLECT );
        // Even if we delay, we want to immediately start up getting shooter up to speed
        robot.shooterMotorsSetPower( shooterPowerFar );
        // Pre-index to the first spindexer position
        loadOrder = PPG_23;
        firstBall = getObeliskFirstBall(obeliskID,loadOrder);
        robot.spinServoSetPosition( firstBall );
        // Enable automatic shooter power/angle as we drive the next segment
        autoAimEnabled = true;
        // Drive out away from wall, both to allow us to rotate the turret and not have the
        // shooter drive belt touch the field wall, but also to be closer to the goal.
        // Must not go so far we are no longer within the scoring zone!
        driveToPosition(-51.8, +14.3, 0.0, DRIVE_SPEED_30, TURN_SPEED_15, DRIVE_TO);
        autoAimEnabled = false;
        // Do we pause before shooting?
        if( waitBefore1st > 0 ) {
            sleep( waitBefore1st * 500 );
        }
        scoreThreeBallsFromField(obeliskID,loadOrder);

        // Collect and Score corner balls
        if( doCorner3 ) {
            loadOrder = (redAlliance)? PPG_23:GPP_21;
            firstBall = getObeliskFirstBall(obeliskID,loadOrder);
            collectCorner3FromFar(redAlliance,firstBall);
            // Do we pause before shooting?
            if( waitBefore2nd > 0 ) {
                sleep( waitBefore2nd * 500 );
                }
            scoreThreeBallsFromField(obeliskID,loadOrder);
        }

        // Collect and Score 1st spike mark
        if( doSpikeMark1 ) {
            loadOrder = (redAlliance)? PGP_22:PGP_22;
            firstBall = getObeliskFirstBall(obeliskID,loadOrder);
            collectSpikemarkFromFar(1,redAlliance,firstBall);
            // Do we pause before shooting?
            if( doCorner3 == false ) {
                if (waitBefore2nd > 0) { sleep(waitBefore2nd * 500); }
            } else {
                if (waitBefore3rd > 0) { sleep(waitBefore3rd * 500); }
            }
            scoreThreeBallsFromField(obeliskID,loadOrder);
        }

        // Collect and Score 2nd spike mark
        if( doSpikeMark2 ) {
            loadOrder = (redAlliance)? PPG_23:GPP_21;
            firstBall = getObeliskFirstBall(obeliskID,loadOrder);
            collectSpikemarkFromFar(2,redAlliance,firstBall);
            // Do we pause before shooting?
            if( doCorner3 == false ) {
                if (waitBefore3rd > 0) { sleep(waitBefore3rd * 500); }
            }
            scoreThreeBallsFromField(obeliskID,loadOrder);
        }
        // Collect and Score 3rd spike mark
        if( doSpikeMark3 ) {
            loadOrder = (redAlliance)? GPP_21:PPG_23;
            firstBall = getObeliskFirstBall(obeliskID,loadOrder);
            collectSpikemarkFromFar(3,redAlliance,firstBall);
            scoreThreeBallsFromField(obeliskID,loadOrder);
        }

        // Drive away from the score line for the MOVEMENT points
        driveToPosition(-30.8, +14.3, 0.0, DRIVE_SPEED_30, TURN_SPEED_30, DRIVE_TO);

        // ensure motors are turned off even if we run out of time
        robot.driveTrainMotorsZero();
    } // mainAutonomous

} /* AutonomousBlueFar */
