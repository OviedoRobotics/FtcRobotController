/* FTC Team 7572 - Version 1.0 (11/07/2024)
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 */
@Autonomous(name="Red Far", group="7592", preselectTeleOp = "Teleop")
//@Disabled
public class AutonomousRedFar extends AutonomousBase {

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drivetrain.
    static final boolean DRIVE_Y = true;    // Drive forward/backward
    static final boolean DRIVE_X = false;   // Drive right/left (not DRIVE_Y)

    double pos_y=robotGlobalYCoordinatePosition, pos_x=robotGlobalXCoordinatePosition, pos_angle=robotOrientationRadians;  // Allows us to specify movement ABSOLUTELY
    public double shooterPower = 0.50;
    public int flywheelFarPosAutoDelay = 400;
    public double startingFlapperPos = 0.5;
    public double startingTurretPos = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("State", "Initializing (please wait)");
        telemetry.update();

        // Initialize robot hardware (autonomous mode)
        robot.init(hardwareMap,true);

        // Start Robot Facing Horizontally to the Field
        robotOrientationRadians = 0.0;

        while (!isStarted()) {
            robot.shooterServo.setPosition(startingFlapperPos);
            robot.turretServo1.setPosition(startingTurretPos);
            // Do we need to change any of the other autonomous options?
            processAutonomousInitMenu(false);  // not auto5 start position
            // Pause briefly before looping
            idle();
        } // !isStarted

        // Wait for the game to start (driver presses PLAY).  While waiting, poll for options
        redAlliance  = true;
        scoringZones = 0;

        resetGlobalCoordinatePosition();

        // Start the autonomous timer so we know how much time is remaining when cycling samples
        autonomousTimer.reset();

        //---------------------------------------------------------------------------------
        // AUTONOMOUS ROUTINE:  The following method is our main autonomous.
        // Assume turret position, flapper, and flywheel motor power is in position
        // Time delay for flywheel to specific motor speed
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
        telemetry.addData("Target", "x=24.0, y=0.0f, 0.00 deg (100%)");
        // reset our timer and drive forward 20"
        autonomousTimer.reset();
        driveToPosition(24.0, 0.0, 0.0, DRIVE_SPEED_100, TURN_SPEED_80, DRIVE_TO);
        double driveTime = autonomousTimer.milliseconds() / 1000.0;
        performEveryLoop();  // ensure our odometry is updated
        telemetry.addData("Odometry", "x=%.2f, y=%.2f, %.2f deg", robotGlobalXCoordinatePosition, robotGlobalYCoordinatePosition, Math.toDegrees(robotOrientationRadians));
        telemetry.addData("Drive Time", "%.3f sec", driveTime);
        telemetry.update();
        sleep(30000);
    }

    /*--------------------------------------------------------------------------------------------*/
    /* Autonomous Red Far:                                                                        */
    /*   1 Starting point                                                                         */
    /*   2 Score preloads                                                                         */
    /*   3 Collect from tick marks (1, 2)                                                         */
    /*   4 Drive back to launch zone                                                              */
    /*   5 Score collected balls                                                                  */
    /*--------------------------------------------------------------------------------------------*/
    private void mainAutonomous() {

        // Do we start with an initial delay?
        if( startDelaySec > 0 ) {
            sleep( startDelaySec * 1000 );
        }

        // Score Preload Balls
        scorePreloadBalls();
//        driveToFirstTickMark();
//        scorePreloadBalls();

        // ensure motors are turned off even if we run out of time
        robot.driveTrainMotorsZero();
    } // mainAutonomous

    /*--------------------------------------------------------------------------------------------*/
    private void scorePreloadBalls() {
        // Turn on flywheel motor
        if( opModeIsActive() ) {
            telemetry.addData("Motion", "Flywheel Ramp Up");
            telemetry.update();
            robot.shooterMotor1.setPower( shooterPower );
            robot.shooterMotor2.setPower( shooterPower );
            sleep(flywheelFarPosAutoDelay); // Wait for flywheels to ramp up to speed
            for(int i=0; i<3; i++){
                launchBall();
            }
        } // opModeIsActive

    } // scoreSamplePreload

    private void launchBall(){
        robot.startInjectionStateMachine();
        do {
            if( !opModeIsActive() ) break;

            sleep(50);

            performEveryLoop();
        } while (robot.liftServoBusyU || robot.liftServoBusyD);

    }


    private void driveToFirstTickMark() {
//        driveToPosition()
    }



} /* AutonomousLeft4 */
