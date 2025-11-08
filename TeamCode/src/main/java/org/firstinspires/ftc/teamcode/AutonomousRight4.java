/* FTC Team 7572 - Version 1.0 (11/07/2024)
*/
package org.firstinspires.ftc.teamcode;

import static java.lang.Math.toDegrees;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="Right: clip 4 (73 pts)", group="7592", preselectTeleOp = "Teleop")
//@Disabled
public class AutonomousRight4 extends AutonomousBase {

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drivetrain.
    static final boolean DRIVE_Y = true;    // Drive forward/backward
    static final boolean DRIVE_X = false;   // Drive right/left (not DRIVE_Y)

    double pos_y=0, pos_x=0, pos_angle=0.0;  // Allows us to specify movement INCREMENTALLY, not ABSOLUTE

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("State", "Initializing (please wait)");
        telemetry.update();

        // Initialize robot hardware (autonomous mode)
        robot.init(hardwareMap,true);

        // Robot starts facing straight away from driver
        //robot.rcStartAngleSet( 180.0 );

        // Initialize webcams using OpenCV
        telemetry.addData("State", "Initializing (please wait)");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY).  While waiting, poll for options
        redAlliance  = true;
        scorePreloadSpecimen = true;
        spikeSamples = 2;  // herd 2 of 3 (so we have time left to score another specimen
        parkLocation = PARK_OBSERVATION;

        while (!isStarted()) {
            // Check for operator input that changes Autonomous options
            captureGamepad1Buttons();
            // Do we need to preload a specimen?
            if( gamepad1_r_bumper_now && !gamepad1_r_bumper_last) {
                if( clawOpen ) {
                    clawOpen = false;
                } else {
                    clawOpen = true;
                }
            } //  gamepad1_r_bumper
            // Do we need to change any of the other autonomous options?
            processAutonomousInitMenu(false);
            // Pause briefly before looping
            idle();
        } // !isStarted

        // Ensure any movement during robot setup is reset to zero
        resetGlobalCoordinatePosition();

        // Start the autonomous timer so we know how much time is remaining when cycling samples
        autonomousTimer.reset();

        // Only do these steps if we didn't hit STOP
        if( opModeIsActive() ) {
//          createAutoStorageFolder(redAlliance, pipelineBack.leftSide);
//          pipelineBack.setStorageFolder(storageDir);
//          pipelineBack.saveSpikeMarkAutoImage();
        }

        //---------------------------------------------------------------------------------
        // UNIT TEST: The following methods verify our basic robot actions.
        // Comment them out when not being tested.
//      testGyroDrive();
//      unitTestOdometryDrive();
//      timeArmMovement();
        //---------------------------------------------------------------------------------

        //---------------------------------------------------------------------------------
        // AUTONOMOUS ROUTINE:  The following method is our main autonomous.
        // Comment it out if running one of the unit tests above.
        mainAutonomous();
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
    // TEST CODE: Verify odometry-based motion functions against a tape measure
    private void unitTestOdometryDrive() {
        // Drive forward 12"
        driveToPosition( 12.0, 0.0, 0.0, DRIVE_SPEED_20, TURN_SPEED_20, DRIVE_TO );
        // Strafe right 12"
        driveToPosition( 12.0, 12.0, 0.0, DRIVE_SPEED_20, TURN_SPEED_20, DRIVE_TO );
        // Turn 180 deg
        driveToPosition( 12.0, 12.0, 90.0, DRIVE_SPEED_20, TURN_SPEED_20, DRIVE_TO );
        // Report the final odometry position/orientation
        telemetry.addData("Final", "x=%.1f, y=%.1f, %.1f deg",
                robotGlobalXCoordinatePosition, robotGlobalYCoordinatePosition, toDegrees(robotOrientationRadians) );
        telemetry.update();
        sleep( 7000 );
    } // unitTestOdometryDrive

    /*--------------------------------------------------------------------------------------------*/
    // TEST CODE: Time how long our arm and viper slide take to get to a specified position
    private void timeArmMovement() {
       boolean tiltDone  = false;
       boolean viperDone = false;
       double  tiltTime = 0.0;
       double  viperTime = 0.0;
       // reset our timer
       autonomousTimer.reset();
       // wait for both to finish
       do {
          if( !opModeIsActive() ) break;
          // only check every 100 msec
          sleep( 100 );
          // update all our status
          performEveryLoop();
       } while( !tiltDone || !viperDone );
       // display the results
       telemetry.addData("Tilt",  "%.1f sec", tiltTime/1000.0 );   // 2.2 sec
       telemetry.addData("Viper", "%.1f sec", viperTime/1000,0 );  // 1.2 sec
       telemetry.update();
       sleep( 7000 );
    } // timeArmMovement

    /*--------------------------------------------------------------------------------------------*/
    /* Autonomous Right:                                                                          */
    /*   1 Starting Point                                                                         */
    /*   2 Hang pre-load specimen at submersible                                                  */
    /*   3 Herd samples from spike marks (left/center/wall)                                       */
    /*   4 Grab clipped specimen from observation zone wall                                       */
    /*   5 Hang specimen on high rung (repeat steps 4 & 5)                                        */
    /*   6 Park in observation zone                                                              */
    /*--------------------------------------------------------------------------------------------*/
    private void mainAutonomous() {
        // ensure motors are turned off even if we run out of time
        robot.driveTrainMotorsZero();

    } // mainAutonomous

} /* AutonomousRight4 */
