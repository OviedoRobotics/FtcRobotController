/* FTC Team 7572 - Version 1.0 (11/07/2024)
 */
package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.HardwareSwyftBot.SpindexerState.SPIN_P1;
import static org.firstinspires.ftc.teamcode.HardwareSwyftBot.SpindexerState.SPIN_P2;
import static org.firstinspires.ftc.teamcode.HardwareSwyftBot.SpindexerState.SPIN_P3;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.HardwareSwyftBot.SpindexerState;

import java.util.List;

/**
 */
@Autonomous(name="Red Near", group="7592", preselectTeleOp = "Teleop-Red")
//@Disabled
public class AutonomousRedNear extends AutonomousBase {

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drivetrain.
    static final boolean DRIVE_Y = true;    // Drive forward/backward
    static final boolean DRIVE_X = false;   // Drive right/left (not DRIVE_Y)

    double pos_y=robotGlobalYCoordinatePosition, pos_x=robotGlobalXCoordinatePosition, pos_angle=robotOrientationRadians;  // Allows us to specify movement ABSOLUTELY

    private Limelight3A limelight;
    private int         obeliskID=23; // if we can't see it, default to PPG (purple purple green)

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
            LLResult result = limelight.getLatestResult();
            if (result.isValid()) {
                // Access fiducial results
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    int limelightID = fr.getFiducialId();
                    // Note: common OBELISK april tags for both RED & BLUE alliance
                    //  21 = GPP (green purple purple)
                    //  22 = PGP (purple green purple)
                    //  23 = PPG (purple purple green)
                    if( (limelightID >= 21) && (limelightID <= 23) ) {
                        telemetry.addData("Obelisk", "ID: %d", limelightID);
                        obeliskID = limelightID;
                    }
                } // fiducialResults
            } // isValid
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
        // Assume turret position, flapper, and flywheel motor power is in position
//      mainAutonomous( obeliskID );
        // just park for now!
        driveToPosition(19.0, -18.0, 0.0, DRIVE_SPEED_30, TURN_SPEED_30, DRIVE_TO);

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
    private void mainAutonomous( int obeliskID ) {

        // Do we start with an initial delay?
        if( startDelaySec > 0 ) {
            sleep( startDelaySec * 1000 );
        }

        // Score Preload Balls
        scorePreloadBalls( obeliskID );
//      driveToFirstTickMark();
//      scorePreloadBalls();

        // Drive away from the score line for the MOVEMENT points
        driveToPosition(-32.0, 0.0, 0.0, DRIVE_SPEED_30, TURN_SPEED_30, DRIVE_TO);

        // ensure motors are turned off even if we run out of time
        robot.driveTrainMotorsZero();
    } // mainAutonomous

    /*--------------------------------------------------------------------------------------------*/
    private void scorePreloadBalls( int obeliskID ) {
        // Turn on flywheel motor
        if( opModeIsActive() ) {
            telemetry.addData("Motion", "Flywheel Ramp Up");
            telemetry.update();
            // Start to ramp up the shooter
            double shooterPower = 0.55;
            robot.shooterMotor1.setPower( shooterPower );
            robot.shooterMotor2.setPower( shooterPower );
            // Start with robot facing the goal (not the obelisk)
            driveToPosition(-12.0, 0.0, 0.0, DRIVE_SPEED_10, TURN_SPEED_15, DRIVE_TO);
            // Drive forward and rotate 180deg so we can shoot
//          driveToPosition(11.0, 0.0, 0.0, DRIVE_SPEED_30, TURN_SPEED_30, DRIVE_THRU);
            // Point the turret toward the goal
            robot.shooterServo.setPosition(0.5);  // NOT ACTUALLY USED
            robot.turretServo1.setPosition(0.43); // rotated left toward BLUE goal);
//          driveToPosition(28.5, 11.5, -179.0, DRIVE_SPEED_30, TURN_SPEED_30, DRIVE_TO);
            // Turn on the collector to help retain balls during spindexing
            robot.intakeMotor.setPower(0.90);
            sleep(4000 ); // Wait a bit longer for flywheels to reach speed
            // spindexer is loaded in P3=PURPLE P2=PURPLE P1=GREEN order
            //  21 = GPP (green purple purple)
            //  22 = PGP (purple green purple)
            //  23 = PPG (purple purple green)
            SpindexerState[] shootOrder = getShootOrder(obeliskID);
            for(int i=0; i<shootOrder.length; i++) {
                // rotate to the next position
                robot.spinServoSetPosition( shootOrder[i] );
                launchBall();
                if( !opModeIsActive() ) break;
                sleep(2000 );
            }
        } // opModeIsActive
    } // scoreSamplePreload

    SpindexerState[] getShootOrder(int obeliskID) {
        // Note: common OBELISK april tags for both RED & BLUE alliance
        //  21 = GPP (green purple purple)
        //  22 = PGP (purple green purple)
        //  23 = PPG (purple purple green)

        // Based on our preload pattern:
        // SPIN_P2 = purple
        // SPIN_P1 = purple
        // SPIN_P3 = green

        switch (obeliskID) {
            case 21:
                return new SpindexerState[] {SPIN_P3, SPIN_P2, SPIN_P1};
            case 22:
                return new SpindexerState[] {SPIN_P2, SPIN_P3, SPIN_P1};
            case 23:
                return new SpindexerState[] {SPIN_P2, SPIN_P1, SPIN_P3};
            default:
                return new SpindexerState[0];
        }
    }

    private void launchBall(){
        robot.startInjectionStateMachine();
        do {
            sleep(50);
            if( !opModeIsActive() ) break;
            performEveryLoop();
        } while (robot.liftServoBusyU || robot.liftServoBusyD);
    }

    private void driveToFirstTickMark() {
//        driveToPosition()
    }

} /* AutonomousRedNear */
