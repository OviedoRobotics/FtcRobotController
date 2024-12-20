/* FTC Team 7572 - Version 1.0 (11/07/2024)
*/
package org.firstinspires.ftc.teamcode;

import static java.lang.Math.toDegrees;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * This program implements robot movement based on Gyro heading and encoder counts.
 * It uses the Mecanumbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode and requires:
 * a) Drive motors with encoders
 * b) Encoder cables
 * c) Rev Robotics I2C IMU with name "imu"
 * d) Drive Motors have been configured such that a positive power command moves forward,
 *    and causes the encoders to count UP.
 * e) The robot must be stationary when the INIT button is pressed, to allow gyro calibration.
 *
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 *  Note: in this example, all angles are referenced to the initial coordinate frame set during the
 *  the Gyro Calibration process, or whenever the program issues a resetZAxisIntegrator() call on the Gyro.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clock Wise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 */
@Autonomous(name="Autonomous Left-Blue", group="7592", preselectTeleOp = "Teleop-Blue")
//@Disabled
public class AutonomousLeftBlue extends AutonomousBase {

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drivetrain.
    static final boolean DRIVE_Y = true;    // Drive forward/backward
    static final boolean DRIVE_X = false;   // Drive right/left (not DRIVE_Y)

    boolean clawOpen = false;
    
    double pos_y=0, pos_x=0, pos_angle=0.0;  // Allows us to specify movement INCREMENTALLY, not ABSOLUTE

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("State", "Initializing (please wait)");
        telemetry.update();

        // Initialize robot hardware (autonomous mode)
        robot.init(hardwareMap,true);

        // Initialize webcams using OpenCV
        telemetry.addData("State", "Initializing (please wait)");
        telemetry.update();
        sleep( 1000 );

        // Wait for the game to start (driver presses PLAY).  While waiting, poll for options
        redAlliance  = false;
        scorePreloadSpecimen = true;
        spikeSamples =3;
        parkLocation = PARK_SUBMERSIBLE;

        while (!isStarted()) {
            // Check for operator input that changes Autonomous options
            captureGamepad1Buttons();
            // Do we need to preload a specimen?
            if( gamepad1_r_bumper_now && !gamepad1_r_bumper_last) {
                if( clawOpen ) {
                    robot.clawStateSet( HardwareMinibot.clawStateEnum.CLAW_CLOSED );
                    clawOpen = false;
                } else {
                    robot.clawStateSet( HardwareMinibot.clawStateEnum.CLAW_OPEN );
                    clawOpen = true;
                }
            }
            // Do we need to change any of the other autonomous options?
            processAutonomousInitMenu();
            // Pause briefly before looping
            idle();
        } // !isStarted

        // Ensure any movement during robot setup is reset to zero
        resetGlobalCoordinatePosition();

        // Start the autonomous timer so we know how much time is remaining when cycling samples
        autonomousTimer.reset();

        // Only do these steps if we didn't hit STOP
        if( opModeIsActive() ) {
//          pixelNumber = 0;
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
       // start both movements
       autoTiltMotorMoveToTarget(Hardware2025Bot.TILT_ANGLE_SPECIMEN1_DEG, 1.0);
       autoViperMotorMoveToTarget(Hardware2025Bot.VIPER_EXTEND_AUTO1);
       // wait for both to finish
       do {
          if( !opModeIsActive() ) break;
          // only check every 100 msec
          sleep( 100 );
          // update all our status
          performEveryLoop();
          if( !tiltDone && !autoTiltMotorMoving() ) {
              tiltTime = autonomousTimer.milliseconds();
              tiltDone = true;
          }
          if( !viperDone && !autoViperMotorMoving() ) {
              viperTime = autonomousTimer.milliseconds();
              viperDone = true;
          }
       } while( !tiltDone || !viperDone );
       // display the results
       telemetry.addData("Tilt",  "%.1f sec", tiltTime/1000.0 );   // 2.2 sec
       telemetry.addData("Viper", "%.1f sec", viperTime/1000,0 );  // 1.2 sec
       telemetry.update();
       sleep( 7000 );
    } // timeArmMovement

    /*--------------------------------------------------------------------------------------------*/
    /* Autonomous Left:                                                                           */
    /*   1 Starting point                                                                         */
    /*   2 Place sample in upper bucket                                                           */
    /*   3 Collect right neutral sample                                                           */
    /*   4 Place sample in upper bucket                                                           */
    /*   5 Collect center neutral sample                                                          */
    /*   6 Place sample in upper bucket                                                           */
    /*   7 Collect left neutral sample                                                            */
    /*   8 Place sample in upper bucket                                                           */
    /*   9 Level one ascent                                                                       */
    /*--------------------------------------------------------------------------------------------*/
    private void mainAutonomous() {

        // Do we start with an initial delay?
        if( startDelaySec > 0 ) {
            sleep( startDelaySec * 1000 );
        }

        // Score the preloaded SPECIMEN
        if( !onlyPark && scorePreloadSpecimen ) {
            scoreSpecimenPreload();
        }

        // Score the preloaded SAMPLE
        if( !onlyPark && !scorePreloadSpecimen ) {
            scoreSamplePreload();
        }

        if( !onlyPark && (spikeSamples > 0) ) {
            if( scorePreloadSpecimen ) {
                driveToPosition(16.0, -19.0, 0.0, DRIVE_SPEED_90, TURN_SPEED_50, DRIVE_THRU);
            }
            autoTiltMotorMoveToTarget(Hardware2025Bot.TILT_ANGLE_DRIVE_DEG, 0.80 );
            autoViperMotorMoveToTarget( Hardware2025Bot.VIPER_EXTEND_AUTO_READY);
            robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_GRAB);
            robot.wristServo.setPosition(Hardware2025Bot.WRIST_SERVO_GRAB);
            // Score starting sample
            int samplesScored = 0;

            while (samplesScored < spikeSamples) {
                collectSample(samplesScored);
                scoreSample();
                samplesScored++;
            }
        }
        // Park for 3pts (level 1 ascent)
//      level1Ascent();

        // ensure motors are turned off even if we run out of time
        robot.driveTrainMotorsZero();

    } // mainAutonomous

    /*--------------------------------------------------------------------------------------------*/
    private void scoreSpecimenPreload() {
        // Drive forward to submersible
        if( opModeIsActive() ) {
            telemetry.addData("Motion", "Move to submersible");
            telemetry.update();
            // Move away from field wall (viper slide motor will hit field wall if we tilt up too soon!)
            driveToPosition( 3.00, 0.00, 0.00, DRIVE_SPEED_30, TURN_SPEED_20, DRIVE_THRU );
            // Start tilting and extending the arm, and positioning the specimen
            autoTiltMotorMoveToTarget(Hardware2025Bot.TILT_ANGLE_SPECIMEN1_DEG, 1.0);
            autoViperMotorMoveToTarget(Hardware2025Bot.VIPER_EXTEND_AUTO1);
            // Drive to the scoring position next to the submersible
            driveToPosition( 22.20, 5.90, 0.00, DRIVE_SPEED_60, TURN_SPEED_20, DRIVE_THRU );
            robot.wristServo.setPosition(Hardware2025Bot.WRIST_SERVO_BAR1);
            robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_BAR1);
            driveToPosition( 28.90, 7.20, 0.00, DRIVE_SPEED_60, TURN_SPEED_20, DRIVE_TO );
            robot.driveTrainMotorsZero();  // make double sure we're stopped
            // If we drive to the submersible faster than the arm moves, wait for the arm
            do {
                if( !opModeIsActive() ) break;
                // only check every 75 msec
                sleep( 75 );
                // update all our status
                performEveryLoop();
            } while( autoTiltMotorMoving() || autoViperMotorMoving());
        } // opModeIsActive

        // Rotate arm, viper slide, and claw down to clip the specimen
        if( opModeIsActive() ) {
            autoTiltMotorMoveToTarget(Hardware2025Bot.TILT_ANGLE_SPECIMEN2_DEG,0.80 );
            autoViperMotorMoveToTarget(Hardware2025Bot.VIPER_EXTEND_AUTO2);
            robot.wristServo.setPosition(Hardware2025Bot.WRIST_SERVO_BAR2);
            robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_BAR2);
            sleep( 1200 ); //while( autoTiltMotorMoving() || autoViperMotorMoving());
            // release the specimen
            robot.clawStateSet( HardwareMinibot.clawStateEnum.CLAW_OPEN );
        } // opModeIsActive

        //Prepare arm for what comes next (samples/parking)
        if( spikeSamples > 0 ) {
           prepareArmForSamples();
        }
        // Whether driving to park, or doing nothing, store the arm
        else {
           prepareArmForDriving();
        }

    } // scoreSpecimenPreload

    /*--------------------------------------------------------------------------------------------*/
    private void scoreSamplePreload() {
        // Drive forward to submersible
        if( opModeIsActive() ) {
            telemetry.addData("Motion", "Move to submersible");
            telemetry.update();
            // Move away from field wall (viper slide motor will hit field wall if we tilt up too soon!)
            driveToPosition( 3.0, 0.0, 0.0, DRIVE_SPEED_70, TURN_SPEED_30, DRIVE_THRU );
            // Move to basket and score preloaded sample
            scoreSample();
        } // opModeIsActive

    } // scoreSamplePreload

    /*--------------------------------------------------------------------------------------------*/
    private void prepareArmForSamples() {

        // Setup the arm for scoring samples
        if( opModeIsActive() ) {
            autoViperMotorMoveToTarget( Hardware2025Bot.VIPER_EXTEND_SECURE);
            do {
                if( !opModeIsActive() ) break;
                // wait for lift/tilt to finish...
                sleep( 50 );
                // update all our status
                performEveryLoop();
            } while( autoViperMotorMoving() );
        } // opModeIsActive

        // Position the collector
        if( opModeIsActive() ) {
            robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_GRAB);
            robot.wristServo.setPosition(Hardware2025Bot.WRIST_SERVO_GRAB);
        } // opModeIsActive

    } // prepareArmForSamples

    /*--------------------------------------------------------------------------------------------*/
    private void prepareArmForDriving() {

        // Retract any arm extension
        if( opModeIsActive() ) {
            autoViperMotorMoveToTarget( Hardware2025Bot.VIPER_EXTEND_ZERO);
//          driveToPosition( 32.60, 2.70, 52.20, DRIVE_SPEED_80, TURN_SPEED_20, DRIVE_THRU );
        } // opModeIsActivee

        // Now that we're clear from the submersible, rotate arm down and store claw
        if( opModeIsActive() ) {
            robot.clawStateSet( HardwareMinibot.clawStateEnum.CLAW_CLOSED );
            robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_INIT);
            robot.wristServo.setPosition(Hardware2025Bot.WRIST_SERVO_INIT);
            autoTiltMotorMoveToTarget(Hardware2025Bot.TILT_ANGLE_WALL_DEG, 0.80);
//          driveToPosition( 38.40, 17.50, 90.00, DRIVE_SPEED_80, TURN_SPEED_20, DRIVE_THRU );
//          driveToPosition( 47.40, 10.00, 180.00, DRIVE_SPEED_80, TURN_SPEED_20,
//                                             ((spikeSamples > 0)? DRIVE_THRU : DRIVE_TO) );
        } // opModeIsActive

    } // prepareArmForDriving

    //************************************
    // Collect sample
    //************************************
    private void collectSample(int samplesScored) {
        autoTiltMotorMoveToTarget(Hardware2025Bot.TILT_ANGLE_DRIVE_DEG, 0.80 );
        autoViperMotorMoveToTarget( Hardware2025Bot.VIPER_EXTEND_AUTO_READY);
        robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_GRAB);
        robot.wristServo.setPosition(Hardware2025Bot.WRIST_SERVO_GRAB);

        switch(samplesScored) {
            case 0:
                // Drive forward toward the wall
                driveToPosition( 17.0, -37.25, 0.0, DRIVE_SPEED_100, TURN_SPEED_50, DRIVE_TO );
                break;
            case 1:
                driveToPosition( 17.0, -47.25, 0.0, DRIVE_SPEED_100, TURN_SPEED_50, DRIVE_TO );
                break;
            case 2:
                driveToPosition( 17.5, -53.0, 7.0, DRIVE_SPEED_100, TURN_SPEED_50, DRIVE_TO );
                break;
            default:
        }
        autoTiltMotorMoveToTarget(Hardware2025Bot.TILT_ANGLE_COLLECT_DEG, 0.80 );
        do {
            if( !opModeIsActive() ) break;
            // wait for lift/tilt to finish...
            sleep( 50 );
            // update all our status
            performEveryLoop();
        } while( autoTiltMotorMoving() );
        //robot.geckoServo.setPower( -1.0 );
        autoViperMotorMoveToTarget( Hardware2025Bot.VIPER_EXTEND_AUTO_COLLECT, 0.5);
        do {
            if( !opModeIsActive() ) break;
            // wait for lift/tilt to finish...
            sleep( 50 );
            // update all our status
            performEveryLoop();
        } while( autoViperMotorMoving() );
        autoTiltMotorMoveToTarget(Hardware2025Bot.TILT_ANGLE_DRIVE_DEG, 0.80 );
        autoViperMotorMoveToTarget( Hardware2025Bot.VIPER_EXTEND_AUTO_READY);
    } // collectSample

    //************************************
    // Score Sample
    //************************************
    private void scoreSample() {
        do {
            if( !opModeIsActive() ) break;
            // wait for lift/tilt to finish...
            sleep( 50 );
            // update all our status
            performEveryLoop();
        } while( autoTiltMotorMoving() );
        autoTiltMotorMoveToTarget(Hardware2025Bot.TILT_ANGLE_AUTO_PRE_DEG, 1.0 );
        driveToPosition( 10.0, -47.5, -32.0, DRIVE_SPEED_100, TURN_SPEED_50, DRIVE_THRU );
        robot.startViperSlideExtension( Hardware2025Bot.VIPER_EXTEND_BASKET );
        driveToPosition( 3.5, -47.5, -32.0, DRIVE_SPEED_100, TURN_SPEED_50, DRIVE_TO );
        robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_SAFE);
        robot.wristServo.setPosition(Hardware2025Bot.WRIST_SERVO_AUTO_SCORE);
        do {
            if( !opModeIsActive() ) break;
            // wait for lift/tilt to finish...
            sleep( 50 );
            // update all our status
            performEveryLoop();
        } while( autoTiltMotorMoving() );
        autoTiltMotorMoveToTarget(Hardware2025Bot.TILT_ANGLE_BASKET_DEG, 0.30 );
        // Try swinging the intake back to see if it increases our scoring. Might move to after arm movement
        do {
            if( !opModeIsActive() ) break;
            // wait for lift/tilt to finish...
            sleep( 50 );
            // update all our status
            performEveryLoop();
        } while( autoTiltMotorMoving() );
        robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_SAFE);
        robot.wristServo.setPosition(Hardware2025Bot.WRIST_SERVO_SAFE);
        sleep(250);
        //robot.geckoServo.setPower( 1.0 );
        sleep(500);
        robot.elbowServo.setPosition(Hardware2025Bot.ELBOW_SERVO_GRAB);
        robot.wristServo.setPosition(Hardware2025Bot.WRIST_SERVO_GRAB);
        sleep(100);
        //robot.geckoServo.setPower( 0.0 );
        autoTiltMotorMoveToTarget(Hardware2025Bot.TILT_ANGLE_DRIVE_DEG, 0.80 );
        autoViperMotorMoveToTarget( Hardware2025Bot.VIPER_EXTEND_AUTO_READY);
    } // scoreSample

    private void level1Ascent() {
        if( opModeIsActive() ) {
            // Back up from submersible
            driveToPosition( 32.0, 6.0, 90.0, DRIVE_SPEED_50, TURN_SPEED_50, DRIVE_TO );
            // Drive forward toward the wall
            driveToPosition( 38.0, -27.0, 90.0, DRIVE_SPEED_50, TURN_SPEED_30, DRIVE_TO );
        } // opModeIsActive

        if( opModeIsActive() ) {
            // Strafe towards submersible
            driveToPosition( 64.0, -27.0, 90.0, DRIVE_SPEED_70, TURN_SPEED_50, DRIVE_TO );
            // Drive backward
            driveToPosition( 64.0, -15.0, 90.0, DRIVE_SPEED_20, TURN_SPEED_20, DRIVE_TO );
        } // opModeIsActive

        if( opModeIsActive() ) {
            autoViperMotorMoveToTarget( Hardware2025Bot.VIPER_EXTEND_GRAB);
            autoTiltMotorMoveToTarget(Hardware2025Bot.TILT_ANGLE_ASCENT1_DEG, 0.80 );
            timeDriveStraight(-DRIVE_SPEED_20,3000);
            do {
                if( !opModeIsActive() ) break;
                // wait for lift/tilt to finish...
                sleep( 150 );
                // update all our status
                performEveryLoop();
            } while( autoTiltMotorMoving() || autoViperMotorMoving() );
        } // opModeIsActive

    } // level1Ascent

} /* AutonomousLeftBlue */
