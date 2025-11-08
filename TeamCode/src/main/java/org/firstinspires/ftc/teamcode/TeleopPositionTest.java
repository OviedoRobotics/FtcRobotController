/* FTC Team 7572 - Version 1.0 (10/13/2025) */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * TeleOp Servo Test Program
 */
@TeleOp(name="Teleop-PositionTest", group="Test")
//@Disabled
public class TeleopPositionTest extends LinearOpMode {
    boolean gamepad1_triangle_last,   gamepad1_triangle_now   = false;  //
    boolean gamepad1_circle_last,     gamepad1_circle_now     = false;  //
    boolean gamepad1_cross_last,      gamepad1_cross_now      = false;  //
    boolean gamepad1_square_last,     gamepad1_square_now     = false;  //
    boolean gamepad1_dpad_up_last,    gamepad1_dpad_up_now    = false;  // gamepad1.dpad_up used live/realtime
    boolean gamepad1_dpad_down_last,  gamepad1_dpad_down_now  = false;  //   (see processDpadDriveMode() below)
    boolean gamepad1_dpad_left_last,  gamepad1_dpad_left_now  = false;
    boolean gamepad1_dpad_right_last, gamepad1_dpad_right_now = false;
    boolean gamepad1_l_bumper_last,   gamepad1_l_bumper_now   = false;
    boolean gamepad1_r_bumper_last,   gamepad1_r_bumper_now   = false;
    boolean gamepad1_touchpad_last,   gamepad1_touchpad_now   = false;
    boolean gamepad1_l_trigger_last,  gamepad1_l_trigger_now  = false;
    boolean gamepad1_r_trigger_last,  gamepad1_r_trigger_now  = false;

    int     selectedMechanism = 0;  // 0=shooter servo; 1=shooter motor, 2=turret servo(s), 3=spin servo; 4=left/inject servo
    double  servoStepSize = 0.01;
    double  shooterPos, turretPos, spinPos, liftPos;
    double  shooterPower = 0.50;

    long    nanoTimeCurr=0, nanoTimePrev=0;
    double  elapsedTime, elapsedHz;

    /* Declare OpMode members. */
    HardwareSwyftBot robot = new HardwareSwyftBot();

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("State", "Initializing (please wait)");
        telemetry.update();

        // Initialize robot hardware (autonomous=true initializes servos)
        robot.init(hardwareMap,true);

        // Preload each variable with the initialization position
        shooterPos = robot.SHOOTER_SERVO_INIT;
        robot.shooterServo.setPosition(shooterPos);

        turretPos = robot.TURRET_SERVO_INIT;
        robot.turretServo1.setPosition(turretPos);
//      robot.turretServo2.setPosition(turretPos);

        // Don't start up the shooter motor until user selects it for modification
//      robot.shooterMotor1.setPower( shooterPower );
//      robot.shooterMotor2.setPower( shooterPower );
    
        spinPos = robot.SPIN_SERVO_P2;
        robot.spinServo.setPosition(spinPos);

        liftPos = robot.LIFT_SERVO_INIT;
        robot.liftServo.setPosition(liftPos);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("State", "Ready");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            // Refresh gamepad button status
            captureGamepad1Buttons();

            // Bulk-refresh the Control/Expansion Hub device status (motor status, digital I/O) -- FASTER!
            robot.readBulkData();

            //================ Update telemetry with current state ================
            telemetry.addData("Use CROSS to toggle between mechanisms", " " );
            telemetry.addData("Use left/right BUMPERS to adjust setting lower/higher", " " );
            switch( selectedMechanism ) { // 0=shooter servo
                case 0 :
                    telemetry.addData("SELECTED:", "shooterServo" );
                    telemetry.addData("Shoooter Servo Position", "%.3f", robot.shooterServo.getPosition() );
                    break;
                case 1 :
                    telemetry.addData("SELECTED:", "shooterMotor" );
                    telemetry.addData("Upper Motor Power", "%.2f", robot.shooterMotor1.getPower() );
                    telemetry.addData("Lower Motor Power", "%.2f", robot.shooterMotor2.getPower() );
                    break;
                case 2 :
                    telemetry.addData("SELECTED:", "turretServo1/2" );
                    telemetry.addData("Turret Servo1 Set Position", "%.3f", turretPos );

                    telemetry.addData("Turret Servo1 Get Position", "%.3f", robot.turretServo1.getPosition() );
                    //telemetry.addData("Turret Servo2 Position", "%.3f", robot.turretServo2.getPosition() );
                    break;
                case 3 :
                    telemetry.addData("SELECTED:", "spinServo" );
                    telemetry.addData("Spindexer Servo Position", "%.3f", robot.spinServo.getPosition() );
                    break;
                case 4 :
                    telemetry.addData("SELECTED:", "liftServo" );
                    telemetry.addData("Injector Servo Position", "%.3f", robot.liftServo.getPosition() );
                    break;
                default :
                    selectedMechanism = 0;
                    break;
            } // switch()

            //================ CROSS SWITCHES WHICH SERVO WE'RE CONTROLLING ================
            if( gamepad1_cross_now && !gamepad1_cross_last)
            {
                selectedMechanism += 1;
                if( selectedMechanism > 4 ) selectedMechanism = 0;
            } // cross

            //================ LEFT BUMPER DECREASES SERVO POSITION ================
            if( gamepad1_l_bumper_now && !gamepad1_l_bumper_last)
            {
                switch( selectedMechanism ) {
                    case 0 :
                        shooterPos -= servoStepSize;
                        if( shooterPos < 0.0 ) shooterPos = 0.0;
                        if( shooterPos > 1.0 ) shooterPos = 1.0;
                        robot.shooterServo.setPosition(shooterPos);
                        break;
                    case 1 :
                        shooterPower -= 0.05;
                        if( shooterPower < 0.0 ) shooterPower = 0.0;
                        if( shooterPower > 1.0 ) shooterPower = 1.0;
                        robot.shooterMotor1.setPower( shooterPower );
                        robot.shooterMotor2.setPower( shooterPower );
                        break;
                    case 2 :
                        turretPos -= 0.02;
                        if( turretPos < 0.0 ) turretPos = 0.0;
                        if( turretPos > 1.0 ) turretPos = 1.0;
                        robot.turretServo1.setPosition(turretPos);
//                      robot.turretServo2.setPosition(turretPos);
                        break;
                    case 3 :
                        spinPos -= servoStepSize;
                        if( spinPos < 0.0 ) spinPos = 0.0;
                        if( spinPos > 1.0 ) spinPos = 1.0;
                        robot.spinServo.setPosition(spinPos);
                        break;
                    case 4 :
                        liftPos -= servoStepSize;
                        if( liftPos < 0.0 ) liftPos = 0.0;
                        if( liftPos > 1.0 ) liftPos = 1.0;
                        robot.liftServo.setPosition(liftPos);
                        break;
                    default :
                        break;
                } // switch()
            } // left bumper

            //================ RIGHT BUMPER INCREASES SERVO POSITION ================
            else if( gamepad1_r_bumper_now && !gamepad1_r_bumper_last)
            {
                switch( selectedMechanism ) {
                    case 0 :
                        shooterPos += servoStepSize;
                        if( shooterPos < 0.0 ) shooterPos = 0.0;
                        if( shooterPos > 1.0 ) shooterPos = 1.0;
                        robot.shooterServo.setPosition(shooterPos);
                        break;
                    case 1 :
                        shooterPower += 0.05;
                        if( shooterPower < 0.0 ) shooterPower = 0.0;
                        if( shooterPower > 1.0 ) shooterPower = 1.0;
                        robot.shooterMotor1.setPower( shooterPower );
                        robot.shooterMotor2.setPower( shooterPower );
                        break;
                    case 2 :
                        turretPos += 0.02;
                        if( turretPos < 0.0 ) turretPos = 0.0;
                        if( turretPos > 1.0 ) turretPos = 1.0;
                        robot.turretServo1.setPosition(turretPos);
//                      robot.turretServo2.setPosition(turretPos);
                        break;
                    case 3 :
                        spinPos += servoStepSize;
                        if( spinPos < 0.0 ) spinPos = 0.0;
                        if( spinPos > 1.0 ) spinPos = 1.0;
                        robot.spinServo.setPosition(spinPos);
                        break;
                    case 4 :
                        liftPos += servoStepSize;
                        if( liftPos < 0.0 ) liftPos = 0.0;
                        if( liftPos > 1.0 ) liftPos = 1.0;
                        robot.liftServo.setPosition(liftPos);
                        break;
                    default :
                        break;
                } // switch()
            } // right bumper

            // Compute current cycle time
            nanoTimePrev = nanoTimeCurr;
            nanoTimeCurr = System.nanoTime();
            elapsedTime  = (nanoTimeCurr - nanoTimePrev)/ 1000000.0;   // msec
            elapsedHz    =  1000.0 / elapsedTime;

            // Update telemetry data
            telemetry.addData("CycleTime", "%.1f msec (%.1f Hz)", elapsedTime, elapsedHz );
            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
        } // opModeIsActive

    } // runOpMode

    /*---------------------------------------------------------------------------------*/
    void captureGamepad1Buttons() {
        gamepad1_triangle_last   = gamepad1_triangle_now;    gamepad1_triangle_now   = gamepad1.triangle;
        gamepad1_circle_last     = gamepad1_circle_now;      gamepad1_circle_now     = gamepad1.circle;
        gamepad1_cross_last      = gamepad1_cross_now;       gamepad1_cross_now      = gamepad1.cross;
        gamepad1_square_last     = gamepad1_square_now;      gamepad1_square_now     = gamepad1.square;
        gamepad1_dpad_up_last    = gamepad1_dpad_up_now;     gamepad1_dpad_up_now    = gamepad1.dpad_up;
        gamepad1_dpad_down_last  = gamepad1_dpad_down_now;   gamepad1_dpad_down_now  = gamepad1.dpad_down;
        gamepad1_dpad_left_last  = gamepad1_dpad_left_now;   gamepad1_dpad_left_now  = gamepad1.dpad_left;
        gamepad1_dpad_right_last = gamepad1_dpad_right_now;  gamepad1_dpad_right_now = gamepad1.dpad_right;
        gamepad1_l_bumper_last   = gamepad1_l_bumper_now;    gamepad1_l_bumper_now   = gamepad1.left_bumper;
        gamepad1_r_bumper_last   = gamepad1_r_bumper_now;    gamepad1_r_bumper_now   = gamepad1.right_bumper;
//      gamepad1_touchpad_last   = gamepad1_touchpad_now;    gamepad1_touchpad_now   = gamepad1.touchpad;
        gamepad1_l_trigger_last  = gamepad1_l_trigger_now;   gamepad1_l_trigger_now  = (gamepad1.left_trigger >= 0.5);
        gamepad1_r_trigger_last  = gamepad1_r_trigger_now;   gamepad1_r_trigger_now  = (gamepad1.right_trigger >= 0.5);
    } // captureGamepad1Buttons

} // TeleopPositionTest
