/* FTC Team 7572 - Version 1.1 (12/11/2025) */
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

    // 0=shooter servo
    // 1=shooter motor
    // 2=turret servo(s)
    // 3=spin servo
    // 4=lift/injecter servo
    // 5=lEyelidServo
    // 6=rEyelidServo 
    int       selectedMechanism = 0;
    double [] stepSizes = { 0.1, 0.01, 0.001, 0.0001 };
    int       stepIndex = 0;
    double    shooterPos, turretPos, spinPos, liftPos, lEyelidPos, rEyelidPos;
    double    shooterPower = 0.50;

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
        robot.turretServo.setPosition(turretPos);

        // Don't start up the shooter motor until user selects it for modification
//      robot.shooterMotorsSetPower( shooterPower );
    
        spinPos = robot.SPIN_SERVO_P2;
        robot.spinServo.setPosition(spinPos);

        liftPos = robot.LIFT_SERVO_INIT;
        robot.liftServo.setPosition(liftPos);
        
        lEyelidPos = robot.L_EYELID_SERVO_INIT;
        robot.lEyelidServo.setPosition(lEyelidPos);
        
        rEyelidPos = robot.R_EYELID_SERVO_INIT;
        robot.rEyelidServo.setPosition(rEyelidPos);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("State", "Ready");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            // Bulk-refresh the Control/Expansion Hub device status (motor status, digital I/O) -- FASTER!
            robot.readBulkData();

            //================ Update telemetry with current state ================
            telemetry.addData("Use CROSS to toggle between mechanisms", " " );
            telemetry.addData("Use left/right BUMPERS to adjust setting lower/higher", " " );
            telemetry.addData("Step Size", "%.4f (O button)", stepSizes[stepIndex]);
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
                    telemetry.addData("SELECTED:", "turretServo" );
                    telemetry.addData("Turret Servo1 Set Position", "%.3f", turretPos );
                    telemetry.addData("Turret Servo1 Get Position", "%.3f", robot.turretServo.getPosition() );
                    break;
                case 3 :
                    telemetry.addData("SELECTED:", "spinServo" );
                    telemetry.addData("Spindexer Servo Command", "%.3f", robot.spinServo.getPosition() );
                    telemetry.addData("Spindexer Servo Feedback", "%.3f deg", robot.getSpindexerAngle() );
                    break;
                case 4 :
                    telemetry.addData("SELECTED:", "liftServo" );
                    telemetry.addData("Injector Servo Command", "%.3f", robot.liftServo.getPosition() );
                    telemetry.addData("Injector Servo Feedback", "%.3f deg", robot.getInjectorAngle() );                                      
                    break;
                case 5 :
                    telemetry.addData("SELECTED:", "lEyelidServo" );
                    telemetry.addData("Left Eyelid Position", "%.3f", robot.lEyelidServo.getPosition() );
                    break;
                case 6 :
                    telemetry.addData("SELECTED:", "rEyelidServo" );
                    telemetry.addData("Right Eyelid Position", "%.3f", robot.rEyelidServo.getPosition() );
                    break;
                default :
                    selectedMechanism = 0;
                    break;
            } // switch()

            //================ CIRCLE SWITCHES OUR SERVO POSITION STEP SIZE ================
            if( gamepad1.circleWasPressed() ) {
                stepIndex = (stepIndex + 1) % stepSizes.length;
            }

            //================ CROSS SWITCHES WHICH SERVO WE'RE CONTROLLING ================
            if( gamepad1.crossWasPressed() )
            {
                selectedMechanism += 1;
                if( selectedMechanism > 6 ) selectedMechanism = 0;
            } // cross

            //================ LEFT BUMPER DECREASES SERVO POSITION ================
            if( gamepad1.leftBumperWasPressed() )
            {
                switch( selectedMechanism ) {
                    case 0 :
                        shooterPos -= stepSizes[stepIndex];
                        if( shooterPos < 0.0 ) shooterPos = 0.0;
                        if( shooterPos > 1.0 ) shooterPos = 1.0;
                        robot.shooterServo.setPosition(shooterPos);
                        break;
                    case 1 :
                        shooterPower -= stepSizes[stepIndex];
                        if( shooterPower < 0.0 ) shooterPower = 0.0;
                        if( shooterPower > 1.0 ) shooterPower = 1.0;
                        robot.shooterMotorsSetPower( shooterPower );
                        break;
                    case 2 :
                        turretPos -= stepSizes[stepIndex];
                        if( turretPos < 0.0 ) turretPos = 0.0;
                        if( turretPos > 1.0 ) turretPos = 1.0;
                        robot.turretServo.setPosition(turretPos);
                        break;
                    case 3 :
                        spinPos -= stepSizes[stepIndex];
                        if( spinPos < 0.0 ) spinPos = 0.0;
                        if( spinPos > 1.0 ) spinPos = 1.0;
                        robot.spinServo.setPosition(spinPos);
                        break;
                    case 4 :
                        liftPos -= stepSizes[stepIndex];
                        if( liftPos < 0.0 ) liftPos = 0.0;
                        if( liftPos > 1.0 ) liftPos = 1.0;
                        robot.liftServo.setPosition(liftPos);
                        break;
                    case 5 :
                        lEyelidPos -= stepSizes[stepIndex];
                        if( lEyelidPos < 0.0 ) lEyelidPos = 0.0;
                        if( lEyelidPos > 1.0 ) lEyelidPos = 1.0;
                        robot.lEyelidServo.setPosition(lEyelidPos);
                        break;
                    case 6 :
                        rEyelidPos -= stepSizes[stepIndex];
                        if( rEyelidPos < 0.0 ) rEyelidPos = 0.0;
                        if( rEyelidPos > 1.0 ) rEyelidPos = 1.0;
                        robot.rEyelidServo.setPosition(rEyelidPos);
                        break;
                    default :
                        break;
                } // switch()
            } // left bumper

            //================ RIGHT BUMPER INCREASES SERVO POSITION ================
            else if( gamepad1.rightBumperWasReleased() )
            {
                switch( selectedMechanism ) {
                    case 0 :
                        shooterPos += stepSizes[stepIndex];
                        if( shooterPos < 0.0 ) shooterPos = 0.0;
                        if( shooterPos > 1.0 ) shooterPos = 1.0;
                        robot.shooterServo.setPosition(shooterPos);
                        break;
                    case 1 :
                        shooterPower += 0.05;
                        if( shooterPower < 0.0 ) shooterPower = 0.0;
                        if( shooterPower > 1.0 ) shooterPower = 1.0;
                        robot.shooterMotorsSetPower( shooterPower );
                        break;
                    case 2 :
                        turretPos += 0.02;
                        if( turretPos < 0.0 ) turretPos = 0.0;
                        if( turretPos > 1.0 ) turretPos = 1.0;
                        robot.turretServo.setPosition(turretPos);
                        break;
                    case 3 :
                        spinPos += stepSizes[stepIndex];
                        if( spinPos < 0.0 ) spinPos = 0.0;
                        if( spinPos > 1.0 ) spinPos = 1.0;
                        robot.spinServo.setPosition(spinPos);
                        break;
                    case 4 :
                        liftPos += stepSizes[stepIndex];
                        if( liftPos < 0.0 ) liftPos = 0.0;
                        if( liftPos > 1.0 ) liftPos = 1.0;
                        robot.liftServo.setPosition(liftPos);
                        break;
                    case 5 :
                        lEyelidPos += stepSizes[stepIndex];
                        if( lEyelidPos < 0.0 ) lEyelidPos = 0.0;
                        if( lEyelidPos > 1.0 ) lEyelidPos = 1.0;
                        robot.lEyelidServo.setPosition(lEyelidPos);
                        break;
                    case 6 :
                        rEyelidPos += stepSizes[stepIndex];
                        if( rEyelidPos < 0.0 ) rEyelidPos = 0.0;
                        if( rEyelidPos > 1.0 ) rEyelidPos = 1.0;
                        robot.rEyelidServo.setPosition(rEyelidPos);
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

} // TeleopPositionTest
