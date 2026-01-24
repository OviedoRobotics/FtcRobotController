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
    // 5=goBilda RGB LED    
    int       selectedMechanism = 1;
    double [] stepSizes = { 0.1, 0.01, 0.001, 0.0001 };
    int       stepIndex = 0;
    double    turretPos, spinPos, liftPos, ledValue;
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

        turretPos = robot.TURRET_SERVO_INIT;
        robot.turretServoSetPosition(turretPos);

        // Don't start up the shooter motor until user selects it for modification
//      robot.shooterMotorsSetPower( shooterPower );
    
        spinPos = robot.SPIN_SERVO_P2;
        robot.spinServo.setPosition(spinPos);

        liftPos = robot.LIFT_SERVO_INIT;
        robot.liftServo.setPosition(liftPos);

        if( robot.isRobot2 ) {
            ledValue = robot.LED_INIT;
            robot.ledServo.setPosition(ledValue);
        }

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
            switch( selectedMechanism ) {
                case 1 :
                    telemetry.addData("SELECTED:", "shooterMotor" );
                    telemetry.addData("Upper Motor Power", "%.2f",    robot.shooterMotor1.getPower() );
                    telemetry.addData("Upper Motor Velocity", "%.2f", robot.shooterMotor1.getVelocity() );
                    telemetry.addData("Lower Motor Power", "%.2f",    robot.shooterMotor2.getPower() );
                    telemetry.addData("Lower Motor Velocity", "%.2f", robot.shooterMotor2.getVelocity() );
                    break;
                case 2 :
                    telemetry.addData("SELECTED:", "turretServo" );
                    telemetry.addData("Turret Servo Set Position", "%.3f", turretPos );
                    telemetry.addData("Turret Servo Get Position", "%.3f", robot.turretServo.getPosition() );
                    telemetry.addData("Turret Servo1 Angle Feedback", "%.3f", robot.getTurretPosition(true) );
                    telemetry.addData("Turret Servo2 Angle Feedback", "%.3f", robot.getTurretPosition(false) );
                    break;
                case 3 :
                    telemetry.addData("SELECTED:", "spinServo" );
                    telemetry.addData("Spindexer Servo Command", "%.3f", robot.spinServo.getPosition() );
                    telemetry.addData("Spindexer Servo Feedback", "%.3f (%.3f deg)", robot.getSpindexerPos(),
                                                                                            robot.getSpindexerAngle() );
                    break;
                case 4 :
                    telemetry.addData("SELECTED:", "liftServo" );
                    telemetry.addData("Injector Servo Command", "%.3f", robot.liftServo.getPosition() );
                    telemetry.addData("Injector Servo Feedback", "%.3f deg", robot.getInjectorAngle() );                                      
                    break;
                case 5 :
                    telemetry.addData("SELECTED:", "ledServo" );
                    if( robot.isRobot2 ) {
                        telemetry.addData("goBilda LED setting", "%.3f", robot.ledServo.getPosition());
                    }
                    break;
                default :
                    selectedMechanism = 1;
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
                if( selectedMechanism > 5 ) selectedMechanism = 1;
            } // cross

            //================ LEFT BUMPER DECREASES SERVO POSITION ================
            if( gamepad1.leftBumperWasPressed() )
            {
                switch( selectedMechanism ) {
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
                        robot.turretServoSetPosition(turretPos);
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
                        ledValue -= stepSizes[stepIndex];
                        if( ledValue < 0.0 ) ledValue = 0.0;
                        if( ledValue > 1.0 ) ledValue = 1.0;
                        if( robot.isRobot2 ) {
                            robot.ledServo.setPosition(ledValue);
                        }
                        break;
                    default :
                        break;
                } // switch()
            } // left bumper

            //================ RIGHT BUMPER INCREASES SERVO POSITION ================
            else if( gamepad1.rightBumperWasReleased() )
            {
                switch( selectedMechanism ) {
                    case 1 :
                        shooterPower += stepSizes[stepIndex];
                        if( shooterPower < 0.0 ) shooterPower = 0.0;
                        if( shooterPower > 1.0 ) shooterPower = 1.0;
                        robot.shooterMotorsSetPower( shooterPower );
                        break;
                    case 2 :
                        turretPos += stepSizes[stepIndex];
                        if( turretPos < 0.0 ) turretPos = 0.0;
                        if( turretPos > 1.0 ) turretPos = 1.0;
                        robot.turretServoSetPosition(turretPos);
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
                        ledValue += stepSizes[stepIndex];
                        if( ledValue < 0.0 ) ledValue = 0.0;
                        if( ledValue > 1.0 ) ledValue = 1.0;
                        if( robot.isRobot2 ) {
                            robot.ledServo.setPosition(ledValue);
                        }
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
